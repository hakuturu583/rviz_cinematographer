/** @file
 *
 * Subscribes to images and generates a video.
 *
 * @author Jan Razlaw
 */

#include "rviz2_cinematographer_video_recorder/video_recorder.h"

#include <algorithm>
#include <functional>

// ament_index_cpp provides get_package_share_path() since Jazzy and removed
// get_package_share_directory.hpp in Rolling, so pick whichever is available.
#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>

namespace
{
std::string getPackageShareDirectory(const std::string& package_name)
{
#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
  return ament_index_cpp::get_package_share_path(package_name).string();
#else
  return ament_index_cpp::get_package_share_directory(package_name);
#endif
}
}  // namespace

namespace rviz2_cinematographer_video_recorder
{

VideoRecorder::VideoRecorder(const rclcpp::NodeOptions& options)
  : rclcpp::Node("video_recorder", options)
    , max_queue_size_(50)
    , process_one_image_duration_(0.0)
    , running_(true)
    , finish_requested_(false)
    , path_to_output_("")
    , codec_(cv::VideoWriter::fourcc('D', 'I', 'V', 'X'))
    , target_fps_(60)
    , add_watermark_(true)
    , is_watermark_resized_(false)
{
  record_finished_pub_ = create_publisher<rviz2_cinematographer_msgs::msg::Finished>("/video_recorder/record_finished", rclcpp::QoS(1));
  wait_pub_ = create_publisher<rviz2_cinematographer_msgs::msg::Wait>("/video_recorder/wait_duration", rclcpp::QoS(1));

  record_params_sub_ = create_subscription<rviz2_cinematographer_msgs::msg::Record>(
    "/rviz/record", rclcpp::QoS(1),
    std::bind(&VideoRecorder::recordParamsCallback, this, std::placeholders::_1));
  rendering_finished_sub_ = create_subscription<rviz2_cinematographer_msgs::msg::Finished>(
    "/rviz/finished_rendering_trajectory", rclcpp::QoS(1),
    std::bind(&VideoRecorder::renderingFinishedCallback, this, std::placeholders::_1));

  // image_transport takes node interfaces and an rclcpp::QoS since Lyrical and dropped the
  // rclcpp::Node* overloads in Rolling. QoS(10) matches the former rmw_qos_profile_default.
#if __has_include(<image_transport/node_interfaces.hpp>)
  image_sub_ = image_transport::create_subscription(*this, "/rviz/view_image",
                                                    std::bind(&VideoRecorder::imageCallback, this, std::placeholders::_1),
                                                    "raw", rclcpp::QoS(10));
#else
  image_sub_ = image_transport::create_subscription(this, "/rviz/view_image",
                                                    std::bind(&VideoRecorder::imageCallback, this, std::placeholders::_1),
                                                    "raw");
#endif

  // thread processing the queued images
  process_images_thread_ = std::thread(&VideoRecorder::processImages, this);
}

VideoRecorder::~VideoRecorder()
{
  running_ = false;
  if(process_images_thread_.joinable())
    process_images_thread_.join();

  std::lock_guard<std::mutex> lock(params_mutex_);
  if(output_video_.isOpened())
    output_video_.release();
}

void VideoRecorder::recordParamsCallback(const rviz2_cinematographer_msgs::msg::Record::ConstSharedPtr record_params)
{
  std::lock_guard<std::mutex> lock(params_mutex_);

  int max_fps = 120;
  if(record_params->compress)
    codec_ = cv::VideoWriter::fourcc('D', 'I', 'V', 'X');
  else
  {
    // lossless codec as documented (the ROS 1 version used the lossy MPEG-1 codec PIM1 here)
    codec_ = cv::VideoWriter::fourcc('F', 'F', 'V', '1');
    max_fps = 60;
  }

  target_fps_ = std::max(1, std::min(max_fps, static_cast<int>(record_params->frames_per_second)));

  path_to_output_ = record_params->path_to_output;
  add_watermark_ = record_params->add_watermark;

  if(add_watermark_)
  {
    // load watermark
    std::string path_to_watermark;
    try
    {
      path_to_watermark = getPackageShareDirectory("rviz2_cinematographer_video_recorder");
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Can't find path to rviz2_cinematographer_video_recorder to load watermark: %s", e.what());
    }

    if(!path_to_watermark.empty())
    {
      path_to_watermark += "/watermark/watermark.png";
      original_watermark_ = cv::imread(path_to_watermark, cv::IMREAD_UNCHANGED);
      if(original_watermark_.empty())
        RCLCPP_ERROR_STREAM(get_logger(), "Could not load watermark from : " << path_to_watermark);
      is_watermark_resized_ = false;
    }
  }
}

void
VideoRecorder::renderingFinishedCallback(const rviz2_cinematographer_msgs::msg::Finished::ConstSharedPtr rendering_finished)
{
  // the processing thread releases the video writer as soon as the queue is empty
  if(rendering_finished->is_finished)
    finish_requested_ = true;
}

void VideoRecorder::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& input_image)
{
  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to convert sensor_msgs::msg::Image to cv_bridge::CvImage : cv_bridge exception: %s", e.what());
    return;
  }

  size_t queue_size = 0;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    image_queue_.push(cv_image);
    queue_size = image_queue_.size();
  }

  if(static_cast<int>(queue_size) >= max_queue_size_)
  {
    RCLCPP_DEBUG(get_logger(), "Max queue size exceeded. Sending wait message.");
    // publish that input has to wait until some images are processed
    std::chrono::duration<double> wait_duration = process_one_image_duration_ * (max_queue_size_ - (max_queue_size_ / 5));
    rviz2_cinematographer_msgs::msg::Wait wait_duration_msg;
    wait_duration_msg.seconds = static_cast<float>(wait_duration.count());
    wait_pub_->publish(wait_duration_msg);
  }
}

void VideoRecorder::finishRecording()
{
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    if(output_video_.isOpened())
      output_video_.release();
  }

  // publish that recording is finished
  rviz2_cinematographer_msgs::msg::Finished record_finished;
  record_finished.is_finished = true;
  record_finished_pub_->publish(record_finished);
}

void VideoRecorder::processImages()
{
  const std::chrono::milliseconds idle_sleep(33); // ~30 hz
  while(running_ && rclcpp::ok())
  {
    cv_bridge::CvImagePtr cv_ptr;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if(!image_queue_.empty())
      {
        cv_ptr = image_queue_.front();
        image_queue_.pop();
      }
    }

    if(cv_ptr)
    {
      auto start = std::chrono::steady_clock::now();

      std::lock_guard<std::mutex> lock(params_mutex_);

      cv::Size img_size(cv_ptr->image.cols, cv_ptr->image.rows);

      if(!output_video_.isOpened())
        if(!output_video_.open(path_to_output_, codec_, target_fps_, img_size, true))
          RCLCPP_ERROR_STREAM(get_logger(), "Could not open the output video to write file in : " << path_to_output_);

      if(output_video_.isOpened())
      {
        if(add_watermark_ && !original_watermark_.empty())
        {
          // resize watermark only once per recording to better fit the video image size
          if(!is_watermark_resized_)
          {
            original_watermark_.copyTo(resized_watermark_);
            resizeWatermark(resized_watermark_, cv_ptr->image.cols);
            is_watermark_resized_ = true;
          }

          // add watermark
          addWatermark(cv_ptr->image, resized_watermark_);
        }
        output_video_.write(cv_ptr->image);
      }

      process_one_image_duration_ = std::chrono::steady_clock::now() - start;
    }
    else if(finish_requested_)
    {
      finish_requested_ = false;
      finishRecording();
    }
    else
    {
      std::this_thread::sleep_for(idle_sleep);
    }
  }
}

void VideoRecorder::resizeWatermark(cv::Mat& watermark, const int image_width)
{
  float watermark_resize_factor = (0.5f * image_width) / watermark.cols;
  if(watermark_resize_factor < 1.f)
    cv::resize(watermark, watermark, cv::Size(), watermark_resize_factor, watermark_resize_factor);
}

void VideoRecorder::addWatermark(cv::Mat& image, const cv::Mat& watermark)
{
  if(watermark.channels() != 4 || image.channels() != 3)
    return;
  if(watermark.rows > image.rows || watermark.cols > image.cols)
    return;

  int origin_watermark_row = image.rows - watermark.rows;
  int origin_watermark_col = image.cols - watermark.cols;
  int image_row = origin_watermark_row;
  int image_col = origin_watermark_col;
  float alpha = 0.8f;
  for(int watermark_row = 0; watermark_row < watermark.rows; watermark_row++, image_row++)
  {
    image_col = origin_watermark_col;
    for(int watermark_col = 0; watermark_col < watermark.cols; watermark_col++, image_col++)
    {
      // overlay if pixel in watermark is not transparent
      unsigned char pixel_alpha = watermark.at<cv::Vec4b>(watermark_row, watermark_col)[3];
      if(pixel_alpha != 0)
        for(int i = 0; i < 3; ++i)
          image.at<cv::Vec3b>(image_row, image_col)[i] = cv::saturate_cast<uchar>(
            alpha * image.at<cv::Vec3b>(image_row, image_col)[i] +
            (1.f - alpha) * watermark.at<cv::Vec4b>(watermark_row, watermark_col)[i]);

    }
  }
}

}  // namespace rviz2_cinematographer_video_recorder

#include <rclcpp_components/register_node_macro.hpp>
// Register this node as a component. This also generates the standalone executable "video_recorder_node".
RCLCPP_COMPONENTS_REGISTER_NODE(rviz2_cinematographer_video_recorder::VideoRecorder)
