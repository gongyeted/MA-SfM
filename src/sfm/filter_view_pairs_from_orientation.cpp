#include "sfm/filter_view_pairs_from_orientation.h"

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <unordered_set>

#include "math/rotation.h"
#include "math/util.h"
#include "rotation_estimation/rotation_estimator.h"
#include "sfm/twoview_info.h"
#include "util/hash.h"
#include "util/map_util.h"
#include "util/types.h"

namespace DAGSfM {

namespace {

bool AngularDifferenceIsAcceptable(
    const Eigen::Vector3d& orientation1, const Eigen::Vector3d& orientation2,
    const Eigen::Vector3d& relative_orientation,
    const double sq_max_relative_rotation_difference_radians) {
  const Eigen::Vector3d composed_relative_rotation =
      MultiplyRotations(orientation2, -orientation1);
  const Eigen::Vector3d loop_rotation =
      MultiplyRotations(-relative_orientation, composed_relative_rotation);
  const double sq_rotation_angular_difference_radians =
      loop_rotation.squaredNorm();
  // LOG(INFO)<<"loop rotation: "<<loop_rotation[0]<<","<<loop_rotation[1]<<","<<loop_rotation[2]
  //         <<" sq_rotation_angular_difference_radians: "<<sq_rotation_angular_difference_radians
  //         <<" max sq: "<<sq_max_relative_rotation_difference_radians;
  return sq_rotation_angular_difference_radians <=
         sq_max_relative_rotation_difference_radians;
}

//my change for comparing error
bool AngularDifferenceIsAcceptable(
    const Eigen::Vector3d& orientation1, const Eigen::Vector3d& orientation2,
    const Eigen::Vector3d& relative_orientation,
    const double sq_max_relative_rotation_difference_radians,
    double & sq_rotation_angular_difference_radian) {
  const Eigen::Vector3d composed_relative_rotation =
      MultiplyRotations(orientation2, -orientation1);
  const Eigen::Vector3d loop_rotation =
      MultiplyRotations(-relative_orientation, composed_relative_rotation);
  const double sq_rotation_angular_difference_radians =
      loop_rotation.squaredNorm();

  // LOG(INFO)<<"orientation1: "<<orientation1;
  // LOG(INFO)<<"orientation2: "<<orientation2;
  // LOG(INFO)<<"relative_orientation: "<<relative_orientation;
  // LOG(INFO)<<sq_rotation_angular_difference_radians;

  sq_rotation_angular_difference_radian = sq_rotation_angular_difference_radians;
  // LOG(INFO)<<"loop rotation: "<<loop_rotation[0]<<","<<loop_rotation[1]<<","<<loop_rotation[2]
  //         <<" sq_rotation_angular_difference_radians: "<<sq_rotation_angular_difference_radians
  //         <<" max sq: "<<sq_max_relative_rotation_difference_radians;
  return sq_rotation_angular_difference_radians <=
         sq_max_relative_rotation_difference_radians;
}

}  // namespace

void FilterViewPairsFromOrientation(
    const std::unordered_map<image_t, Eigen::Vector3d>& orientations,
    const double max_relative_rotation_difference_degrees,
    std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
    Database& database) {
  CHECK_GE(max_relative_rotation_difference_degrees, 0.0);

  // Precompute the squared threshold in radians.
  const double max_relative_rotation_difference_radians =
      DegToRad(max_relative_rotation_difference_degrees);
  const double sq_max_relative_rotation_difference_radians =
      max_relative_rotation_difference_radians *
      max_relative_rotation_difference_radians;

  std::unordered_set<ImagePair> view_pairs_to_remove;

  for (auto& view_pair : view_pairs) {
    const Eigen::Vector3d* orientation1 =
        FindOrNull(orientations, view_pair.first.first);
    const Eigen::Vector3d* orientation2 =
        FindOrNull(orientations, view_pair.first.second);

    // If the view pair contains a view that does not have an orientation then
    // remove it.
    if (orientation1 == nullptr || orientation2 == nullptr) {
      LOG(WARNING)
          << "View pair (" << view_pair.first.first << ", "
          << view_pair.first.second
          << ") contains a view that does not exist! Removing the view pair.";
      view_pairs_to_remove.insert(view_pair.first);
      continue;
    }

    // Remove the view pair if the relative rotation estimate is not within the
    // tolerance.
    if (!AngularDifferenceIsAcceptable(
            *orientation1, *orientation2, view_pair.second.rotation_2,
            sq_max_relative_rotation_difference_radians)) {
      view_pairs_to_remove.insert(view_pair.first);
    } else {
      // Update relative rotations.
      view_pair.second.rotation_2 = geometry::RelativeRotationFromTwoRotations(
          *orientation1, *orientation2);
    }
  }

  // Remove all the "bad" relative poses.
  for (const ImagePair view_id_pair : view_pairs_to_remove) {
    view_pairs.erase(view_id_pair);
    database.DeleteMatches(view_id_pair.first, view_id_pair.second);
    database.DeleteInlierMatches(view_id_pair.first, view_id_pair.second);
  }
  VLOG(1) << "Removed " << view_pairs_to_remove.size()
          << " view pairs by rotation filtering.";
}


// bool FilterSubclusterEdgesFromOrientation(
//     const std::unordered_map<image_t, Eigen::Vector3d>& orientations_0,
//     const std::unordered_map<image_t, Eigen::Vector3d>& orientations_1,
//     const std::unordered_map<image_t, Eigen::Vector3d>& orientations_merged,
//     const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs_0,
//     const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs_1,
//     const double max_relative_rotation_difference_degrees) {
//   CHECK_GE(max_relative_rotation_difference_degrees, 0.0);

//   // Precompute the squared threshold in radians.
//   const double max_relative_rotation_difference_radians =
//       DegToRad(max_relative_rotation_difference_degrees);
//   const double sq_max_relative_rotation_difference_radians =
//       max_relative_rotation_difference_radians *
//       max_relative_rotation_difference_radians;

//   bool all_edges_pass = true;
//   //遍历subcluster1中的edges
//   for (auto view_pair : view_pairs_0)
//   {
//     image_t img0 = view_pair.first.first;
//     image_t img1 = view_pair.first.second;
//     //读取subcluster内部rotation结果，并计算相对旋转作为参考值
//     const Eigen::Vector3d* orientation_0 =
//         FindOrNull(orientations_0, img0);
//     const Eigen::Vector3d* orientation_1 =
//         FindOrNull(orientations_0, img1);
//     const Eigen::Vector3d composed_relative_rotation =
//       MultiplyRotations(*orientation_1, -*orientation_0);
//     //读取subcluster合并后的rotation结果
//     const Eigen::Vector3d* orientation_2 =
//         FindOrNull(orientations_merged, img0);
//     const Eigen::Vector3d* orientation_3 =
//         FindOrNull(orientations_merged, img1);
//     //验证相对旋转变化大小
//     if (!AngularDifferenceIsAcceptable( *orientation_2, *orientation_3, composed_relative_rotation,
//             sq_max_relative_rotation_difference_radians))
//     {
//       LOG(INFO)<<"rotation between img "<<img0<<" and img "<<img1<<" changed too much";
//       return false;
//     }
        

//     // if (!AngularDifferenceIsAcceptable(
//     //         *orientation_2, *orientation_3, composed_relative_rotation,
//     //         sq_max_relative_rotation_difference_radians)) {
//     //   view_pairs_to_remove.insert(view_pair.first);
//     // } else {
//     //   // Update relative rotations.
//     //   view_pair.second.rotation_2 = geometry::RelativeRotationFromTwoRotations(
//     //       *orientation1, *orientation2);
//     // }
//   }
//   return true;
  
// }

bool FilterSubclusterEdgesFromOrientation(
    const std::unordered_map<image_t, Eigen::Vector3d>& robust_orientations,
    const std::unordered_map<image_t, Eigen::Vector3d>& merged_orientations,
    const std::unordered_map<ImagePair, TwoViewInfo>& robust_view_pairs,
    const double max_relative_rotation_difference_degrees) {
  CHECK_GE(max_relative_rotation_difference_degrees, 0.0);

  // Precompute the squared threshold in radians.
  const double max_relative_rotation_difference_radians =
      DegToRad(max_relative_rotation_difference_degrees);
  const double sq_max_relative_rotation_difference_radians =
      max_relative_rotation_difference_radians *
      max_relative_rotation_difference_radians;

  bool all_edges_pass = true;
  //遍历之前已经比较稳定可靠的edges
  for (auto view_pair : robust_view_pairs)
  {
    image_t img0 = view_pair.first.first;
    image_t img1 = view_pair.first.second;
    //读取当前可靠rotations结果，并计算相对旋转作为参考值
    const Eigen::Vector3d* orientation_0 =
        FindOrNull(robust_orientations, img0);
    const Eigen::Vector3d* orientation_1 =
        FindOrNull(robust_orientations, img1);
    const Eigen::Vector3d composed_relative_rotation =
      MultiplyRotations(*orientation_1, -*orientation_0);
    //读取merged待定rotations结果
    const Eigen::Vector3d* orientation_2 =
        FindOrNull(merged_orientations, img0);
    const Eigen::Vector3d* orientation_3 =
        FindOrNull(merged_orientations, img1);
    

    //验证并保存相对旋转变化大小
    if (!AngularDifferenceIsAcceptable( *orientation_2, *orientation_3, composed_relative_rotation,
            sq_max_relative_rotation_difference_radians))
    {
      LOG(INFO)<<"rotation between img "<<img0<<" and img "<<img1<<" changed too much";
      return false;
    }
  }
  return true;  
}



bool FilterSubclusterEdgesFromOrientation(
    const std::unordered_map<image_t, Eigen::Vector3d>& robust_orientations,
    const std::unordered_map<image_t, Eigen::Vector3d>& merged_orientations,
    const std::unordered_map<ImagePair, TwoViewInfo>& robust_view_pairs,
    const double max_relative_rotation_difference_degrees,
    std::vector<std::pair<ImagePair, double>>& relative_rotation_difference_degrees) {
  CHECK_GE(max_relative_rotation_difference_degrees, 0.0);

  // Precompute the squared threshold in radians.
  const double max_relative_rotation_difference_radians =
      DegToRad(max_relative_rotation_difference_degrees);
  const double sq_max_relative_rotation_difference_radians =
      max_relative_rotation_difference_radians *
      max_relative_rotation_difference_radians;

  bool all_edges_pass = true;
  //遍历之前已经比较稳定可靠的edges
  for (auto view_pair : robust_view_pairs)
  {
    image_t img0 = view_pair.first.first;
    image_t img1 = view_pair.first.second;
    //读取当前可靠rotations结果，并计算相对旋转作为参考值
    const Eigen::Vector3d* orientation_0 =
        FindOrNull(robust_orientations, img0);
    const Eigen::Vector3d* orientation_1 =
        FindOrNull(robust_orientations, img1);
    const Eigen::Vector3d composed_relative_rotation =
      MultiplyRotations(*orientation_1, -*orientation_0);
    //读取merged待定rotations结果
    const Eigen::Vector3d* orientation_2 =
        FindOrNull(merged_orientations, img0);
    const Eigen::Vector3d* orientation_3 =
        FindOrNull(merged_orientations, img1);
    

    //验证并保存相对旋转变化大小
    double sq_relative_radian;
    bool differenceAcceptable = AngularDifferenceIsAcceptable( *orientation_2, *orientation_3, composed_relative_rotation,
            sq_max_relative_rotation_difference_radians, sq_relative_radian);
    CHECK_NE(sq_relative_radian, 0);
    CHECK_NE(RadToDeg(sq_relative_radian), 0);
    relative_rotation_difference_degrees.push_back(std::make_pair(view_pair.first,RadToDeg(sq_relative_radian)));

    if (!differenceAcceptable)
    {
      LOG(INFO)<<"rotation between img "<<img0<<" and img "<<img1<<" changed too much";
      return false;
    }
  }
  return true;  
}

}  // namespace DAGSfM