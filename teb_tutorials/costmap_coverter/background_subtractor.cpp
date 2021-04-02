#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/cvv/cvv.hpp>
#include <costmap_converter/costmap_to_dynamic_obstacles/background_subtractor.h>

BackgroundSubtractor::BackgroundSubtractor(const Params &parameters): params_(parameters)
{
}

void BackgroundSubtractor::apply(const cv::Mat& image, cv::Mat& fg_mask, int shift_x, int shift_y)
{
  current_frame_ = image;

  // 用当前代价地图给快速滤波地图（occupancy_grid_fast_）和慢速滤波地图（occupancy_grid_slow_）
  if (occupancy_grid_fast_.empty() && occupancy_grid_slow_.empty())
  {
    occupancy_grid_fast_ = current_frame_;
    occupancy_grid_slow_ = current_frame_;
    previous_shift_x_ = shift_x;
    previous_shift_y_ = shift_y;
    return;
  }

  // Shift previous occupancy grid to new location (match current_frame_)
  int shift_relative_to_previous_pos_x_ = shift_x - previous_shift_x_;
  int shift_relative_to_previous_pos_y_ = shift_y - previous_shift_y_;
  previous_shift_x_ = shift_x;
  previous_shift_y_ = shift_y;

  // 将快速滤波地图和慢速滤波地图平移shift_relative_to_previous_pos使其栅格与当前代价地图一一对应
  transformToCurrentFrame(shift_relative_to_previous_pos_x_, shift_relative_to_previous_pos_y_);

  // cvv::debugFilter(occupancy_grid_fast_, currentFrame_, CVVISUAL_LOCATION);

  // Calculate normalized sum (mean) of nearest neighbors
  int size = 3; // 3, 5, 7, ....
  cv::Mat nearest_neighbor_mean_fast(occupancy_grid_fast_.size(), CV_8UC1);
  cv::Mat nearest_neighbor_mean_slow(occupancy_grid_slow_.size(), CV_8UC1);
  //方框滤波，这步在论文里没有
  cv::boxFilter(occupancy_grid_fast_, nearest_neighbor_mean_fast, -1, cv::Size(size, size), cv::Point(-1, -1), true,
                cv::BORDER_REPLICATE);
  cv::boxFilter(occupancy_grid_slow_, nearest_neighbor_mean_slow, -1, cv::Size(size, size), cv::Point(-1, -1), true,
                cv::BORDER_REPLICATE);

  // occupancy_grid_fast_ = beta*(alpha_fast*current_frame_ + (1.0-alpha_fast)*occupancy_grid_fast_) + (1-beta)*nearest_neighbor_mean_fast;
  // 快速一阶滤波
  cv::addWeighted(current_frame_, params_.alpha_fast, occupancy_grid_fast_, (1 - params_.alpha_fast), 0, occupancy_grid_fast_);
  // 额外操作，可能使算法更鲁棒
  cv::addWeighted(occupancy_grid_fast_, params_.beta, nearest_neighbor_mean_fast, (1 - params_.beta), 0, occupancy_grid_fast_);
  // occupancy_grid_slow_ = beta*(alpha_slow*current_frame_ + (1.0-alpha_slow)*occupancy_grid_slow) + (1-beta)*nearest_neighbor_mean_slow;
  // 慢速一阶滤波
  cv::addWeighted(current_frame_, params_.alpha_slow, occupancy_grid_slow_, (1 - params_.alpha_slow), 0, occupancy_grid_slow_);
  // 额外操作，可能使算法更鲁棒
  cv::addWeighted(occupancy_grid_slow_, params_.beta, nearest_neighbor_mean_slow, (1 - params_.beta), 0, occupancy_grid_slow_);

  // 1) Obstacles should be detected after a minimum response of the fast filter
  //    occupancy_grid_fast_ > min_occupancy_probability
  // 阈值判断得到动态物体地图，此时还不是二值地图
  cv::threshold(occupancy_grid_fast_, occupancy_grid_fast_, params_.min_occupancy_probability, 0 /*unused*/, cv::THRESH_TOZERO);
  cv::threshold(occupancy_grid_fast_ - occupancy_grid_slow_, fg_mask, params_.min_sep_between_fast_and_slow_filter, 255,
                cv::THRESH_BINARY);
  // 3) Dismiss static obstacles
  //    nearest_neighbor_mean_slow < max_occupancy_neighbors
  // 利用相邻的像素判断，在论文中省略了，原论文中有
  cv::threshold(nearest_neighbor_mean_slow, nearest_neighbor_mean_slow, params_.max_occupancy_neighbors, 255, cv::THRESH_BINARY_INV);
  cv::bitwise_and(nearest_neighbor_mean_slow, fg_mask, fg_mask);

  //visualize("Current frame", currentFrame_);
  cv::Mat setBorderToZero = cv::Mat(current_frame_.size(), CV_8UC1, 0.0);
  int border = 5;
  setBorderToZero(cv::Rect(border, border, current_frame_.cols-2*border, current_frame_.rows-2*border)) = 255;

  cv::bitwise_and(setBorderToZero, fg_mask, fg_mask);

  // cv::imwrite("/home/albers/Desktop/currentFrame.png", currentFrame_);
  // visualize("Foreground mask", fgMask);

  // Closing Operation
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                              cv::Size(2*params_.morph_size + 1, 2*params_.morph_size + 1),
                                              cv::Point(params_.morph_size, params_.morph_size));
  // 膨胀
  cv::dilate(fg_mask, fg_mask, element);
  cv::dilate(fg_mask, fg_mask, element);
  // 腐蚀
  cv::erode(fg_mask, fg_mask, element);
}

void BackgroundSubtractor::transformToCurrentFrame(int shift_x, int shift_y)
{
  // TODO: initialize with current_frame (first observed image) rather than zeros

  // Translate (shift) image in costmap coordinates
  // in cv::Mat: shift X -> to the left; shift y -> to the top
  cv::Mat temp_fast, temp_slow;
  cv::Mat translation_mat = (cv::Mat_<double>(2, 3, CV_64F) << 1, 0, -shift_x, 0, 1, -shift_y);
  cv::warpAffine(occupancy_grid_fast_, temp_fast, translation_mat, occupancy_grid_fast_.size()); // can't operate in-place
  cv::warpAffine(occupancy_grid_slow_, temp_slow, translation_mat, occupancy_grid_slow_.size()); // can't operate in-place

  // cvv::debugFilter(occupancy_grid_fast_, temp_fast);

  occupancy_grid_fast_ = temp_fast;
  occupancy_grid_slow_ = temp_slow;
}

void BackgroundSubtractor::visualize(const std::string& name, const cv::Mat& image)
{
  if (!image.empty())
  {
    cv::Mat im = image.clone();
    cv::flip(im, im, 0);
    cv::imshow(name, im);
    cv::waitKey(1);
  }
}

void BackgroundSubtractor::updateParameters(const Params &parameters)
{
  params_ = parameters;
}
