#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace {

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

volatile std::sig_atomic_t g_stop_requested = 0;

struct Options {
  std::string camera = "0";
  int columns = 0;
  int rows = 0;
  int width = 640;
  int height = 480;
  int target_count = 20;
  double save_interval_seconds = 2.0;
  fs::path output_dir = "calibration_images";
};

void handleSignal(int) {
  g_stop_requested = 1;
}

void printUsage(const char* program) {
  std::cout
      << "用法：" << program
      << " --columns N --rows N [选项]\n"
      << "无屏幕采集棋盘格标定图片；检测到完整棋盘格后自动保存。\n\n"
      << "选项：\n"
      << "  --camera SOURCE        摄像头编号或设备路径，默认 0\n"
      << "  --columns N            棋盘格内角点列数（至少 3）\n"
      << "  --rows N               棋盘格内角点行数（至少 3）\n"
      << "  --width N              请求采集宽度，默认 640\n"
      << "  --height N             请求采集高度，默认 480\n"
      << "  --target-count N       采集张数，默认 20\n"
      << "  --save-interval SEC    两次保存的最小间隔秒数，默认 2\n"
      << "  --output-dir PATH      图片目录根路径，默认 calibration_images\n"
      << "  --help                 显示帮助\n\n"
      << "示例：\n"
      << "  " << program
      << " --camera 0 --columns 9 --rows 6 --target-count 20\n"
      << "  " << program
      << " --camera /dev/video0 --columns 9 --rows 6 --output-dir images\n";
}

std::string takeValue(int& index, int argc, char* argv[], const std::string& name) {
  if (index + 1 >= argc) {
    throw std::invalid_argument("参数缺少取值：" + name);
  }
  ++index;
  return argv[index];
}

int parseInteger(const std::string& text, const std::string& name, int minimum) {
  std::size_t parsed = 0;
  long value = 0;
  try {
    value = std::stol(text, &parsed);
  } catch (const std::exception&) {
    throw std::invalid_argument(name + " 必须是整数");
  }
  if (parsed != text.size() || value < minimum ||
      value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(name + " 必须是大于等于 " +
                                 std::to_string(minimum) + " 的整数");
  }
  return static_cast<int>(value);
}

double parsePositiveDouble(const std::string& text, const std::string& name) {
  std::size_t parsed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &parsed);
  } catch (const std::exception&) {
    throw std::invalid_argument(name + " 必须是数字");
  }
  if (parsed != text.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " 必须是有限正数");
  }
  return value;
}

Options parseArguments(int argc, char* argv[], bool& show_help) {
  Options options;
  bool has_columns = false;
  bool has_rows = false;
  show_help = false;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      show_help = true;
      return options;
    }
    if (argument == "--camera") {
      options.camera = takeValue(index, argc, argv, argument);
    } else if (argument == "--columns") {
      options.columns = parseInteger(
          takeValue(index, argc, argv, argument), argument, 3);
      has_columns = true;
    } else if (argument == "--rows") {
      options.rows = parseInteger(
          takeValue(index, argc, argv, argument), argument, 3);
      has_rows = true;
    } else if (argument == "--width") {
      options.width = parseInteger(
          takeValue(index, argc, argv, argument), argument, 1);
    } else if (argument == "--height") {
      options.height = parseInteger(
          takeValue(index, argc, argv, argument), argument, 1);
    } else if (argument == "--target-count") {
      options.target_count = parseInteger(
          takeValue(index, argc, argv, argument), argument, 1);
    } else if (argument == "--save-interval") {
      options.save_interval_seconds = parsePositiveDouble(
          takeValue(index, argc, argv, argument), argument);
    } else if (argument == "--output-dir") {
      options.output_dir = takeValue(index, argc, argv, argument);
    } else {
      throw std::invalid_argument("未知参数：" + argument);
    }
  }

  if (!has_columns || !has_rows) {
    throw std::invalid_argument("必须指定 --columns 和 --rows");
  }
  return options;
}

bool isCameraIndex(const std::string& value) {
  return !value.empty() && std::all_of(
      value.begin(), value.end(), [](unsigned char character) {
        return std::isdigit(character) != 0;
      });
}

fs::path createSessionDirectory(const fs::path& output_root) {
  std::error_code error;
  fs::create_directories(output_root, error);
  if (error) {
    throw fs::filesystem_error(
        "无法创建图片根目录", output_root, error);
  }

  const auto now = std::chrono::system_clock::now();
  const std::time_t current_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &current_time);
#else
  localtime_r(&current_time, &local_time);
#endif

  std::ostringstream timestamp;
  timestamp << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  const std::string base_name = timestamp.str();

  for (int suffix = 0; suffix < 100000; ++suffix) {
    const std::string name = suffix == 0
        ? base_name
        : base_name + "_" + std::to_string(suffix);
    const fs::path candidate = output_root / name;
    error.clear();
    if (fs::create_directory(candidate, error)) {
      return candidate;
    }
    if (error && error != std::errc::file_exists) {
      throw fs::filesystem_error("无法创建图片会话目录", candidate, error);
    }
  }

  throw std::runtime_error("无法找到可用的图片会话目录名称");
}

bool detectCheckerboard(
    const cv::Mat& frame, const cv::Size& pattern_size,
    std::vector<cv::Point2f>& corners) {
  cv::Mat gray;
  if (frame.channels() == 1) {
    gray = frame;
  } else if (frame.channels() == 3) {
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  } else if (frame.channels() == 4) {
    cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
  } else {
    throw std::runtime_error("摄像头图像必须是灰度、BGR 或 BGRA 格式");
  }

  const int flags = cv::CALIB_CB_ADAPTIVE_THRESH |
                    cv::CALIB_CB_NORMALIZE_IMAGE;
  const bool found = cv::findChessboardCorners(gray, pattern_size, corners, flags);
  if (!found || corners.size() != static_cast<std::size_t>(
                                      pattern_size.width * pattern_size.height)) {
    return false;
  }

  const cv::TermCriteria criteria(
      cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
  cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
  return true;
}

fs::path imagePath(const fs::path& directory, int image_number) {
  std::ostringstream filename;
  filename << "frame_" << std::setw(4) << std::setfill('0')
           << image_number << ".png";
  return directory / filename.str();
}

int runCapture(const Options& options) {
  cv::VideoCapture capture;
  if (isCameraIndex(options.camera)) {
    capture.open(parseInteger(options.camera, "--camera", 0));
  } else {
    capture.open(options.camera);
  }
  if (!capture.isOpened()) {
    std::cerr << "无法打开摄像头：" << options.camera << '\n';
    return 1;
  }

  capture.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);

  cv::Mat frame;
  if (!capture.read(frame) || frame.empty()) {
    std::cerr << "无法从摄像头读取图像：" << options.camera << '\n';
    return 1;
  }

  const fs::path session_directory = createSessionDirectory(options.output_dir);
  std::cout << "摄像头：" << options.camera << "，实际分辨率："
            << frame.cols << 'x' << frame.rows << '\n'
            << "图片目录：" << session_directory.string() << '\n'
            << "无屏幕采集：目标 " << options.target_count << " 张，保存间隔至少 "
            << options.save_interval_seconds << " 秒。\n"
            << "请持续改变棋盘格的位置、距离和倾角；按 Ctrl+C 可提前结束。\n";

  const cv::Size pattern_size(options.columns, options.rows);
  const auto save_interval = std::max(
      std::chrono::milliseconds(1),
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(options.save_interval_seconds)));
  auto next_save_time = Clock::time_point::min();
  auto last_status_time = Clock::now();
  int saved_count = 0;

  while (saved_count < options.target_count && !g_stop_requested) {
    cv::Mat oriented_frame;
    cv::rotate(frame, oriented_frame, cv::ROTATE_180);

    std::vector<cv::Point2f> corners;
    const bool found = detectCheckerboard(oriented_frame, pattern_size, corners);
    const auto current_time = Clock::now();

    if (found && current_time >= next_save_time) {
      const fs::path path = imagePath(session_directory, saved_count + 1);
      if (!cv::imwrite(path.string(), oriented_frame)) {
        std::cerr << "保存图片失败：" << path.string() << '\n';
        return 1;
      }

      ++saved_count;
      next_save_time = current_time + save_interval;
      last_status_time = current_time;
      std::cout << "已保存 " << saved_count << '/' << options.target_count
                << "：" << path.string()
                << "；请改变棋盘姿态后再继续采集。\n";
      if (saved_count == options.target_count) {
        std::cout << "已达到目标张数，采集完成。\n";
        break;
      }
    } else if (current_time - last_status_time >= std::chrono::seconds(5)) {
      if (found) {
        std::cout << "已检测到棋盘格，正在等待下一保存间隔；请改变棋盘姿态。\n";
      } else {
        std::cout << "暂未检测到完整棋盘格，请将棋盘移入镜头并调整角度。\n";
      }
      last_status_time = current_time;
    }

    if (!capture.read(frame) || frame.empty()) {
      std::cerr << "摄像头读取图像失败。\n";
      return 1;
    }
  }

  if (g_stop_requested) {
    std::cout << "采集已中断，已保存 " << saved_count << " 张图片。\n";
  }
  return 0;
}

}  // 匿名命名空间

int main(int argc, char* argv[]) {
  bool show_help = false;
  Options options;
  try {
    options = parseArguments(argc, argv, show_help);
    if (show_help) {
      printUsage(argv[0]);
      return 0;
    }
  } catch (const std::exception& error) {
    std::cerr << "参数错误：" << error.what() << '\n';
    printUsage(argv[0]);
    return 2;
  }

  std::signal(SIGINT, handleSignal);
  try {
    return runCapture(options);
  } catch (const cv::Exception& error) {
    std::cerr << "OpenCV 采集失败：" << error.what() << '\n';
    return 1;
  } catch (const fs::filesystem_error& error) {
    std::cerr << "文件操作失败：" << error.what() << '\n';
    return 1;
  } catch (const std::exception& error) {
    std::cerr << "采集失败：" << error.what() << '\n';
    return 1;
  }
}
