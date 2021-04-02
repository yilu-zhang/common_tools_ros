// Based on https://github.com/Smorodov/Multitarget-tracker/tree/master/Tracker, GPLv3
// Refer to README.md in this directory.

#include <costmap_converter/costmap_to_dynamic_obstacles/multitarget_tracker/Ctracker.h>

// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
CTracker::CTracker(const Params &parameters)
    : params(parameters),
      NextTrackID(0)
{
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
void CTracker::Update(const std::vector<Point_t>& detectedCentroid, const std::vector< std::vector<cv::Point> >& contours)
{
  // Each contour has a centroid
  assert(detectedCentroid.size() == contours.size());

  // -----------------------------------
  // If there is no tracks yet, then every cv::Point begins its own track.
  // -----------------------------------
  if (tracks.size() == 0)
  {
    // 初始化或物体全被跟丢时才进入
    for (size_t i = 0; i < detectedCentroid.size(); ++i)
    {
      tracks.push_back(
          std::unique_ptr<CTrack>(new CTrack(detectedCentroid[i], contours[i], params.dt, NextTrackID++)));
    }
  }

  size_t N = tracks.size();//t-1时刻物体个数
  size_t M = detectedCentroid.size();//t时刻物体个数

  assignments_t assignment;

  if (!tracks.empty())
  {
    // Distance matrix of N-th Track to the M-th detectedCentroid
    distMatrix_t Cost(N * M);

    // 计算物体之间距离，tracks为t-1时刻跟踪到物体位置，detectedCentroid为t时刻跟踪到物体位置
    for (size_t i = 0; i < tracks.size(); i++)
    {
      for (size_t j = 0; j < detectedCentroid.size(); j++)
      {
        Cost[i + j * N] = tracks[i]->CalcDist(detectedCentroid[j]);
      }
    }

    // -----------------------------------
    // Solving assignment problem (tracks and predictions of Kalman filter)
    // -----------------------------------
    AssignmentProblemSolver APS;
    // 使用匈牙利算法进行匹配，匹配结果存在assignment，下标为t-1时刻物体索引，内容为t时刻与之匹配内容索引，-1表示没有匹配
    APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);

    // -----------------------------------
    // clean assignment from pairs with large distance
    // -----------------------------------
    for (size_t i = 0; i < assignment.size(); i++)
    {
      if (assignment[i] != -1)
      {
          //距离超过阈值params.dist_thresh认为是无效匹配
        if (Cost[i + assignment[i] * N] > params.dist_thresh)
        {
          assignment[i] = -1;
          tracks[i]->skipped_frames = 1;
        }
      }
      else
      {
        // If track have no assigned detect, then increment skipped frames counter.
        tracks[i]->skipped_frames++;
      }
    }

    // -----------------------------------
    // If track didn't get detects long time, remove it.
    // -----------------------------------
    for (int i = 0; i < static_cast<int>(tracks.size()); i++)
    {
        //物体超过params.max_allowed_skipped_frames未被跟踪物体被清除
      if (tracks[i]->skipped_frames > params.max_allowed_skipped_frames)
      {
        tracks.erase(tracks.begin() + i);
        assignment.erase(assignment.begin() + i);
        i--;
      }
    }
  }

  // -----------------------------------
  // t时刻没有与t-1时刻匹配上的物体作为新物体添加到tracks，以便下一次跟踪
  // -----------------------------------
  for (size_t i = 0; i < detectedCentroid.size(); ++i)
  {
    if (find(assignment.begin(), assignment.end(), i) == assignment.end())
    {
      tracks.push_back(std::unique_ptr<CTrack>(new CTrack(detectedCentroid[i], contours[i], params.dt, NextTrackID++)));
    }
  }

  // Update Kalman Filters state

  for (size_t i = 0; i < assignment.size(); i++)
  {
    // If track updated less than one time, than filter state is not correct.

    if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
    {
      tracks[i]->skipped_frames = 0;//物体被跟踪后skipped_frames重新计数
      // 用t时刻匹配到物体位置更新卡尔曼滤波器，
      tracks[i]->Update(detectedCentroid[assignment[i]], contours[assignment[i]], true, params.max_trace_length);
    }
    else // if not continue using predictions
    {
        //无匹配，使用t-1时刻物体的位置更新卡尔曼滤波器
      tracks[i]->Update(Point_t(), std::vector<cv::Point>(), false, params.max_trace_length);
    }
  }
}

void CTracker::updateParameters(const Params &parameters)
{
  params = parameters;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTracker::~CTracker(void) {}
