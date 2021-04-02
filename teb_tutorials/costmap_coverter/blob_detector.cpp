#include <costmap_converter/costmap_to_dynamic_obstacles/blob_detector.h>
#include <opencv2/opencv.hpp>
#include <iostream>

BlobDetector::BlobDetector(const SimpleBlobDetector::Params& parameters) : params_(parameters) {}

cv::Ptr<BlobDetector> BlobDetector::create(const cv::SimpleBlobDetector::Params& params)
{
  return cv::Ptr<BlobDetector> (new BlobDetector(params)); // compatibility with older versions
  //return cv::makePtr<BlobDetector>(params);
}

void BlobDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat&)
{
  // TODO: support mask
  contours_.clear();

  keypoints.clear();
  cv::Mat grayscale_image;
  //转成灰度图
  if (image.channels() == 3)
    cv::cvtColor(image, grayscale_image, cv::COLOR_BGR2GRAY);
  else
    grayscale_image = image;

  if (grayscale_image.type() != CV_8UC1)
  {
    //CV_Error(cv::Error::StsUnsupportedFormat, "Blob detector only supports 8-bit images!");
    std::cerr << "Blob detector only supports 8-bit images!\n";
  }

  std::vector<std::vector<Center>> centers;
  std::vector<std::vector<cv::Point>> contours;
  // 该循环只执行一次，可以通过设置thresholdStep多执行几次，得到更多动态物体，但默认只执行一次
  for (double thresh = params_.minThreshold; thresh < params_.maxThreshold; thresh += params_.thresholdStep)
  {
    cv::Mat binarized_image;
    //转成二值地图
    cv::threshold(grayscale_image, binarized_image, thresh, 255, cv::THRESH_BINARY);

    std::vector<Center> cur_centers;
    std::vector<std::vector<cv::Point>> cur_contours, new_contours;
    // 查找轮廓和质心，centers保存物体的中心，cur_contours保存物体轮廓，他们外层的size对应物体个数
    findBlobs(grayscale_image, binarized_image, cur_centers, cur_contours);
    std::vector<std::vector<Center>> new_centers;
    for (std::size_t i = 0; i < cur_centers.size(); ++i)
    {
      bool isNew = true;
      // 因为外层循环只执行一次，故该循环一次也不会执行，忽略
      for (std::size_t j = 0; j < centers.size(); ++j)
      {
        double dist = cv::norm(centers[j][centers[j].size() / 2].location - cur_centers[i].location);
        isNew = dist >= params_.minDistBetweenBlobs && dist >= centers[j][centers[j].size() / 2].radius &&
                dist >= cur_centers[i].radius;
        if (!isNew)
        {
          centers[j].push_back(cur_centers[i]);

          size_t k = centers[j].size() - 1;
          while (k > 0 && centers[j][k].radius < centers[j][k - 1].radius)
          {
            centers[j][k] = centers[j][k - 1];
            k--;
          }
          centers[j][k] = cur_centers[i];

          break;
        }
      }
      if (isNew)
      {
        new_centers.push_back(std::vector<Center>(1, cur_centers[i]));
        new_contours.push_back(cur_contours[i]);
      }
    }
    std::copy(new_centers.begin(), new_centers.end(), std::back_inserter(centers));
    std::copy(new_contours.begin(), new_contours.end(), std::back_inserter(contours));
  }

  for (size_t i = 0; i < centers.size(); ++i)
  {
    if (centers[i].size() < params_.minRepeatability)
      continue;
    cv::Point2d sum_point(0, 0);
    double normalizer = 0;
    // 因为外层循环只执行一次，故该循环只执行一次
    for (std::size_t j = 0; j < centers[i].size(); ++j)
    {
      sum_point += centers[i][j].confidence * centers[i][j].location;
      normalizer += centers[i][j].confidence;
    }
    sum_point *= (1. / normalizer);
    cv::KeyPoint kpt(sum_point, (float)(centers[i][centers[i].size() / 2].radius));
    keypoints.push_back(kpt);
    contours_.push_back(contours[i]);
  }
}

// 查找轮廓和质心，centers保存物体的中心，cur_contours保存物体轮廓，他们外层的size对应物体个数
void BlobDetector::findBlobs(const cv::Mat& image, const cv::Mat& binary_image, std::vector<Center>& centers,
                             std::vector<std::vector<cv::Point>>& cur_contours) const
{
  (void)image;
  centers.clear();
  cur_contours.clear();

  std::vector<std::vector<cv::Point>> contours;
  cv::Mat tmp_binary_image = binary_image.clone();
  cv::findContours(tmp_binary_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  for (std::size_t contour_idx = 0; contour_idx < contours.size(); ++contour_idx)
  {
    Center center;
    center.confidence = 1;
    cv::Moments moms = cv::moments(cv::Mat(contours[contour_idx]));

    // moms滤波结束开始，滤波过程可以忽略
    if (params_.filterByArea)
    {
      double area = moms.m00;
      if (area < params_.minArea || area >= params_.maxArea)
        continue;
    }

    if (params_.filterByCircularity)
    {
      double area = moms.m00;
      double perimeter = cv::arcLength(cv::Mat(contours[contour_idx]), true);
      double ratio = 4 * CV_PI * area / (perimeter * perimeter);
      if (ratio < params_.minCircularity || ratio >= params_.maxCircularity)
        continue;
    }

    if (params_.filterByInertia)
    {
      double denominator = std::sqrt(std::pow(2 * moms.mu11, 2) + std::pow(moms.mu20 - moms.mu02, 2));
      const double eps = 1e-2;
      double ratio;
      if (denominator > eps)
      {
        double cosmin = (moms.mu20 - moms.mu02) / denominator;
        double sinmin = 2 * moms.mu11 / denominator;
        double cosmax = -cosmin;
        double sinmax = -sinmin;

        double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
        double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
        ratio = imin / imax;
      }
      else
      {
        ratio = 1;
      }

      if (ratio < params_.minInertiaRatio || ratio >= params_.maxInertiaRatio)
        continue;

      center.confidence = ratio * ratio;
    }
    // moms滤波结束

    // 轮廓点滤波
    if (params_.filterByConvexity)
    {
      std::vector<cv::Point> hull;
      cv::convexHull(cv::Mat(contours[contour_idx]), hull);
      double area = cv::contourArea(cv::Mat(contours[contour_idx]));
      double hullArea = cv::contourArea(cv::Mat(hull));
      double ratio = area / hullArea;
      if (ratio < params_.minConvexity || ratio >= params_.maxConvexity)
        continue;
    }

    if (moms.m00 == 0.0)
      continue;

    center.location = cv::Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

    if (params_.filterByColor)
    {
      if (binary_image.at<uchar>(cvRound(center.location.y), cvRound(center.location.x)) != params_.blobColor)
        continue;
    }

    // 计算动态物体的半径
    {
      std::vector<double> dists;
      for (std::size_t point_idx = 0; point_idx < contours[contour_idx].size(); ++point_idx)
      {
        cv::Point2d pt = contours[contour_idx][point_idx];
        dists.push_back(cv::norm(center.location - pt));
      }
      std::sort(dists.begin(), dists.end());
      center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
    }

    // 经过了层层滤波的动态物体才会被保留下来
    centers.push_back(center);
    cur_contours.push_back(contours[contour_idx]);
  }
}

void BlobDetector::updateParameters(const Params& parameters)
{
  params_ = parameters;
}
