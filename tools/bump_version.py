#!/usr/bin/env python3
"""Bumps the release version of every package in this repository together.

All packages are released as one bloom repository, so their package.xml versions have
to agree -- the release workflow refuses to publish a tag that doesn't match them.
Editing them by hand is how they drift, so the release job calls this instead.

    tools/bump_version.py patch          # 0.2.0 -> 0.2.1
    tools/bump_version.py minor          # 0.2.0 -> 0.3.0
    tools/bump_version.py major          # 0.2.0 -> 1.0.0
    tools/bump_version.py 0.4.2          # set exactly
    tools/bump_version.py current        # print the version, change nothing

Besides package.xml, each CHANGELOG.rst gets a section for the new version, which
bloom copies into the Debian changelog. If a package already has a "Forthcoming"
section (e.g. written with catkin_generate_changelog) it is renamed to the new version
and kept as is. Otherwise the section is generated from the commits since the previous
release tag that touched the package; packages with no such commits get "No changes",
matching what catkin_prepare_release does. A changelog that already has a section for
the target version is left alone, so setting the current version (the first release)
only validates.

The single line printed to stdout is the new version, which the workflow reads back to
form the tag. Nothing else is printed there, so it can be captured directly.
"""

from __future__ import annotations

import datetime
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
VERSION_RE = re.compile(r"\d+\.\d+\.\d+")
# Tags created by the release workflow are the bare version, the ROS/bloom convention.
TAG_PATTERN = "[0-9]*.[0-9]*.[0-9]*"
# Commits the release workflow makes itself; they don't belong in a changelog.
RELEASE_COMMIT_PREFIX = "chore(release):"


def packages() -> list[Path]:
    """Package directories, discovered rather than listed so a new package can't be missed."""
    dirs = sorted(p.parent for p in ROOT.glob("*/package.xml"))
    if not dirs:
        raise SystemExit(f"found no packages under {ROOT}")
    return dirs


def read_version(package: Path) -> str:
    match = re.search(r"<version>\s*([^<\s]+)\s*</version>", (package / "package.xml").read_text())
    if match is None:
        raise SystemExit(f"could not find <version> in {package / 'package.xml'}")
    return match.group(1)


def current_version() -> str:
    versions = {package.name: read_version(package) for package in packages()}
    if len(set(versions.values())) != 1:
        listing = ", ".join(f"{name}={version}" for name, version in versions.items())
        raise SystemExit(f"package versions disagree: {listing}")
    return next(iter(versions.values()))


def next_version(current: str, spec: str) -> str:
    if spec == "current":
        return current
    if spec not in {"major", "minor", "patch"}:
        if not VERSION_RE.fullmatch(spec):
            raise SystemExit(f"expected major|minor|patch|current or an X.Y.Z version, got {spec!r}")
        return spec
    major, minor, patch = (int(part) for part in current.split("."))
    if spec == "major":
        return f"{major + 1}.0.0"
    if spec == "minor":
        return f"{major}.{minor + 1}.0"
    return f"{major}.{minor}.{patch + 1}"


def set_version(package: Path, version: str) -> None:
    path = package / "package.xml"
    text, count = re.subn(r"<version>[^<]*</version>", f"<version>{version}</version>", path.read_text(), count=1)
    if count != 1:
        raise SystemExit(f"failed to rewrite <version> in {path}")
    path.write_text(text)


def git(*args: str) -> str:
    return subprocess.run(["git", *args], cwd=ROOT, check=True, capture_output=True, text=True).stdout


def previous_tag() -> str | None:
    try:
        return git("describe", "--tags", "--abbrev=0", "--match", TAG_PATTERN, "HEAD").strip() or None
    except subprocess.CalledProcessError:
        return None


def last_change(path: Path) -> str | None:
    """The commit that last touched `path`, the fallback start before the first tag."""
    sha = git("log", "-1", "--format=%H", "--", str(path.relative_to(ROOT))).strip()
    return sha or None


def changes(package: Path, since: str | None) -> tuple[list[str], list[str]]:
    """Commit subjects and authors touching `package` after `since`, oldest first."""
    rev_range = [f"{since}..HEAD"] if since else ["HEAD"]
    out = git("log", "--no-merges", "--reverse", "--format=%s%x1f%an", *rev_range,
              "--", str(package.relative_to(ROOT)))
    subjects, authors = [], []
    for line in out.splitlines():
        subject, _, author = line.partition("\x1f")
        if subject.startswith(RELEASE_COMMIT_PREFIX) or author.endswith("[bot]"):
            continue
        subjects.append(subject)
        if author not in authors:
            authors.append(author)
    return subjects, authors


def section_header(version: str) -> str:
    title = f"{version} ({datetime.date.today().isoformat()})"
    return f"{title}\n{'-' * len(title)}\n"


def update_changelog(package: Path, version: str, since_tag: str | None) -> None:
    path = package / "CHANGELOG.rst"
    if not path.exists():
        title = f"Changelog for package {package.name}"
        path.write_text(f"{'^' * len(title)}\n{title}\n{'^' * len(title)}\n\n")
    text = path.read_text()

    if re.search(rf"(?m)^{re.escape(version)} \(", text):
        return  # already has a section for this version

    forthcoming = re.search(r"(?m)^Forthcoming\n-+\n", text)
    if forthcoming:
        path.write_text(text[:forthcoming.start()] + section_header(version) + text[forthcoming.end():])
        return

    subjects, authors = changes(package, since_tag or last_change(path))
    body = "".join(f"* {subject}\n" for subject in subjects) or "* No changes\n"
    if authors:
        body += f"* Contributors: {', '.join(sorted(authors))}\n"
    section = section_header(version) + body + "\n"

    # The new section goes right after the title block (^^^ / title / ^^^ / blank line).
    title_block = re.match(r"(\^+\n[^\n]*\n\^+\n\n?)", text)
    if title_block is None:
        raise SystemExit(f"unexpected header in {path}")
    head = title_block.group(1)
    if not head.endswith("\n\n"):
        head += "\n"
    path.write_text(head + section + text[title_block.end():])


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    current = current_version()
    version = next_version(current, sys.argv[1])
    if sys.argv[1] != "current":
        since_tag = previous_tag()
        for package in packages():
            set_version(package, version)
            update_changelog(package, version, since_tag)
    print(version)


if __name__ == "__main__":
    main()
