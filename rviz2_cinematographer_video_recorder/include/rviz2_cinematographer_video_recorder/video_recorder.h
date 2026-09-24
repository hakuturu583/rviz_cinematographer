/** @file
 *
 * Subscribes to images and generates a video.
 *
 * @author Jan Razlaw
 */

#ifndef RVIZ2_CINEMATOGRAPHER_VIDEO_RECORDER_VIDEO_RECORDER_H
#define RVIZ2_CINEMATOGRAPHER_VIDEO_RECORDER_VIDEO_RECORDER_H

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <rviz2_cinematographer_msgs/msg/record.hpp>
#include <rviz2_cinematographer_msgs/msg/finished.hpp>
#include <rviz2_cinematographer_msgs/msg/wait.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <image_transport/image_transport.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace rviz2_cinematographer_video_recorder
{

/**
 * @brief Node that subscribes to images and writes them into a video file.
 *
 * Can be used as a composable node (component) or as a standalone executable.
 */
class VideoRecorder : public rclcpp::Node
{
public:

  /**
   * @brief Constructor. Sets up subscribers, publishers and the image processing thread.
   *
   * @param[in] options   node options.
   */
  explicit VideoRecorder(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /** @brief Destructor. Stops the processing thread and releases the video writer. */
  ~VideoRecorder() override;

protected:

  /** @brief Sets requested recording parameters.
   *
   * @params[in] record_params  specifies that a record should be made and the parameters that should be used.
   */
  void recordParamsCallback(const rviz2_cinematographer_msgs::msg::Record::ConstSharedPtr record_params);

  /** @brief Awaits a message indicating that the image stream ended to stop recording.
   *
   * Requests the processing thread to release the video writer as soon as the image queue is processed and
   * to publish that the recording is finished.
   *
   * @params[in] rendering_finished  true if image stream ended.
   */
  void renderingFinishedCallback(const rviz2_cinematographer_msgs::msg::Finished::ConstSharedPtr rendering_finished);

  /** @brief Stores subscribed images in queue and publishes a message if queue is too large.
   *
   * If queue's size exceeds max_queue_size, the duration it takes to process most of the queue is computed and
   * published. This message can be used by the source of the image stream to wait for the estimated duration.
   *
   * @params[in] input_image  subscribed image.
   */
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& input_image);

  /** @brief Feeds images from queue to video writer, optionally adding a watermark. */
  void processImages();

  /** @brief Releases the video writer and publishes that the recording is finished. */
  void finishRecording();

  /** @brief Resizes watermark to be at most half as wide as the input images.
   *
   * @params[in,out]    watermark       the watermark being resized.
   * @params[in]        image_width     the width of the input images.
   */
  void resizeWatermark(cv::Mat& watermark, const int image_width);

  /** @brief Adds watermark to the image.
   *
   * @params[in,out]    image       the image being watermarked.
   * @params[in]        watermark   the watermark.
   */
  void addWatermark(cv::Mat& image, const cv::Mat& watermark);

protected:

  rclcpp::Subscription<rviz2_cinematographer_msgs::msg::Record>::SharedPtr record_params_sub_;
  rclcpp::Subscription<rviz2_cinematographer_msgs::msg::Finished>::SharedPtr rendering_finished_sub_;

  image_transport::Subscriber image_sub_;

  std::mutex queue_mutex_;
  std::queue<cv_bridge::CvImagePtr> image_queue_;
  int max_queue_size_;
  std::chrono::duration<double> process_one_image_duration_;

  std::thread process_images_thread_;
  std::atomic<bool> running_;
  std::atomic<bool> finish_requested_;

  rclcpp::Publisher<rviz2_cinematographer_msgs::msg::Finished>::SharedPtr record_finished_pub_;
  rclcpp::Publisher<rviz2_cinematographer_msgs::msg::Wait>::SharedPtr wait_pub_;

  /// Protects the recording parameters and the video writer.
  std::mutex params_mutex_;
  cv::VideoWriter output_video_;
  std::string path_to_output_;
  int codec_;
  int target_fps_;
  bool add_watermark_;
  cv::Mat original_watermark_;
  cv::Mat resized_watermark_;
  bool is_watermark_resized_;
};

}  // namespace rviz2_cinematographer_video_recorder

#endif // RVIZ2_CINEMATOGRAPHER_VIDEO_RECORDER_VIDEO_RECORDER_H
