#include "controllers/sfm_aligner.h"

#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/solver.h>
#include <glog/logging.h>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "base/similarity_transform.h"
#include "base/track_selection.h"
#include "estimators/ransac_similarity.h"
#include "estimators/sim3.h"
#include "estimators/similarity_transform.h"
#include "math/util.h"
#include "optim/bundle_adjustment.h"
#include "sfm/incremental_triangulator.h"
#include "solver/l1_solver.h"
#include "util/misc.h"
#include "util/reconstruction_io.h"
#include "util/timer.h"

namespace DAGSfM {
namespace {

double CheckReprojError(const vector<Eigen::Vector3d>& src_observations,
                        const vector<Eigen::Vector3d>& dst_observations,
                        const double& scale, const Eigen::Matrix3d& R,
                        const Eigen::Vector3d& t) {
  double reproj_err = 0.0;
  const int size = src_observations.size();
  for (int i = 0; i < size; i++) {
    Eigen::Vector3d reproj_obv = scale * R * src_observations[i] + t;
    reproj_err += (reproj_obv - dst_observations[i]).norm();
  }

  LOG(INFO) << "Mean Reprojection Error: " << reproj_err / size << " ("
            << reproj_err << "/" << size << ")";
  return reproj_err / size;
}

void FindSimilarityTransform(const std::vector<Eigen::Vector3d>& observations1,
                             const std::vector<Eigen::Vector3d>& observations2,
                             const double threshold, const double p,
                             Eigen::Matrix3d& R, Eigen::Vector3d& t,
                             double& scale, double& msd) {
  std::vector<Eigen::Vector3d> inliers1, inliers2;

  if (observations1.size() > 5) {
    // LOG(INFO) << "Finding Similarity by RANSAC";
    RansacSimilarity(observations1, observations2, inliers1, inliers2, R, t,
                     scale, threshold, p);
    VLOG(2) << "inliers size: " << inliers1.size();
    // Re-compute similarity by inliers
    Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(3, inliers1.size()),
                    x2 = Eigen::MatrixXd::Zero(3, inliers2.size());
    for (uint i = 0; i < inliers1.size(); i++) {
      x1.col(i) = inliers1[i];
      x2.col(i) = inliers2[i];
    }
    DAGSfM::FindRTS(x1, x2, &scale, &t, &R);
    // Optional non-linear refinement of the found parameters
    DAGSfM::Refine_RTS(x1, x2, &scale, &t, &R);

    if (inliers1.size() < 4) {
      msd = numeric_limits<double>::max();
      return;
    }
    // else msd = CheckReprojError(inliers1, inliers2, scale, R, t);
  }

  if (observations1.size() <= 5 || inliers1.size() <= 5) {
    Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(3, observations1.size()),
                    x2 = Eigen::MatrixXd::Zero(3, observations2.size());
    for (uint i = 0; i < observations1.size(); i++) {
      x1.col(i) = observations1[i];
      x2.col(i) = observations2[i];
    }
    DAGSfM::FindRTS(x1, x2, &scale, &t, &R);
    DAGSfM::Refine_RTS(x1, x2, &scale, &t, &R);

    // msd = CheckReprojError(observations1, observations2, scale, R, t);
  }

    //此处源码没有考虑observations1.size=0的情况，此时msd计算公式中分母为0，msd=nan，不为最大值，反而导致其没有被筛掉
  //my change
  if(observations1.size()==0||observations2.size()==0)
  {
    msd = numeric_limits<double>::max();
    return;
  }   

  msd = CheckReprojError(observations1, observations2, scale, R, t);
}

void FindCommon3DPoints(const std::vector<image_t>& common_reg_images,
                        const Reconstruction& recon1,
                        const Reconstruction& recon2,
                        std::vector<Eigen::Vector3d>& src_points,
                        std::vector<Eigen::Vector3d>& ref_points) {
  // std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> common_3D_points;
  std::unordered_set<image_t> common_image_ids(common_reg_images.begin(),
                                               common_reg_images.end());
  // LOG(INFO) << "Begin find common 3D points";
  for (const auto& point3D : recon2.Points3D()) {
    const Eigen::Vector3d point3D2 = point3D.second.XYZ();

    for (const auto& track_el : point3D.second.Track().Elements()) {
      if (common_image_ids.count(track_el.image_id) > 0) {
        const auto& point2D =
            recon1.Image(track_el.image_id).Point2D(track_el.point2D_idx);
        if (point2D.HasPoint3D()) {
          const Eigen::Vector3d point3D1 =
              recon1.Point3D(point2D.Point3DId()).XYZ();
          // common_3D_points.emplace_back(point3D1, point3D2);
          src_points.emplace_back(point3D1);
          ref_points.emplace_back(point3D2);
        }
      }
    }
  }
  // LOG(INFO) << "Find " << ref_points.size() << " common 3D points.";
}

}  // namespace

SfMAligner::SfMAligner(const std::vector<Reconstruction*>& reconstructions,
                       const AlignOptions& options)
    : options_(options), reconstructions_(reconstructions) {
  // some logic or parameters check
  CHECK_GT(reconstructions.size(), 0);
  CHECK_GT(reconstructions_.size(), 0);

  for (uint i = 0; i < reconstructions_.size(); i++) {
    LOG(INFO) << "Node id: " << i;
    CHECK_NOTNULL(reconstructions_[i]);
    LOG(INFO) << "Total images number: " << reconstructions_[i]->NumImages();
  }
}

const std::vector<BitmapColor<float>> SfMAligner::ColorContainers = {
    BitmapColor<float>(255, 25.5, 0), BitmapColor<float>(0, 255, 255),
    BitmapColor<float>(255, 102, 0),  BitmapColor<float>(153, 51, 204),
    BitmapColor<float>(0, 255, 51),   BitmapColor<float>(255, 0, 204),
    BitmapColor<float>(255, 255, 0),  BitmapColor<float>(255, 153, 255),
    BitmapColor<float>(255, 51, 0),   BitmapColor<float>(0, 204, 255),
    BitmapColor<float>(255, 204, 255)};

bool SfMAligner::Align() {
  Timer timer;

  // 1. Constructing a graph from reconstructions,
  // each node is a reconstruction, edges represent the connections between
  // reconstructions (by the means of common images or common 3D points), the
  // weight of edge represents the mean reprojection error.
  LOG(INFO) << "Constructing Reconstructions Graph...";
  timer.Start();
  ConstructReconsGraph();
  timer.Pause();
  summary_.construct_recon_graph_time = timer.ElapsedSeconds();
  recons_graph_.ShowInfo();
  CHECK_EQ(recons_graph_.GetNodesNum(), reconstructions_.size());

  // The reconstruction graph should be at least a spanning tree,
  // or we couldn't stitch all reconstructions together due to
  // too large alignment error or disconnected components.
  if (recons_graph_.GetEdgesNum() < recons_graph_.GetNodesNum() - 1) {
    LOG(WARNING) << "Can't align all reconstructions together due to "
                 << "too large alignment error or disconnected components."
                 << "We would just merge local maps in the largest connected "
                    "components.";
  }
  const Graph<Node, Edge> largest_cc = recons_graph_.ExtractLargestCC();
  std::vector<size_t> vec_largest_cc_nodes;
  vec_largest_cc_nodes.reserve(largest_cc.GetNodes().size());
  for (auto node_it : largest_cc.GetNodes()) {
    vec_largest_cc_nodes.push_back(node_it.first);
  }

  // 2. Constructing a minimum spanning tree, thus we can select the
  // most accurate n - 1 edges for accurate alignment.
  LOG(INFO) << "Finding Minimum Spanning Tree...";
  timer.Start();
  std::vector<Edge> mst_edges = largest_cc.Kruskal();

  Graph<Node, Edge> mst;
  for (const auto edge : mst_edges) {
    mst.AddEdge(edge);
  }

  if (mst_edges.size() < largest_cc.GetNodesNum() - 1) {
    LOG(WARNING) << "Invalid MST";
    mst.ShowInfo();
    return false;
  }
  mst.ShowInfo();
  timer.Pause();
  summary_.construct_mst_time = timer.ElapsedSeconds();

  // 3. Finding an anchor node, an anchor node is a reference reconstruction
  // that all other reconstructions should be aligned to.
  LOG(INFO) << "Finding Anchor Node...";
  timer.Start();
  FindAnchorNode(&mst);
  timer.Pause();
  summary_.find_anchor_node_time = timer.ElapsedSeconds();

  // 4. Compute the final transformation to anchor node for each cluster
  LOG(INFO) << "Computing Final Similarity Transformations...";
  timer.Start();
  for (auto i : vec_largest_cc_nodes) {
    if (static_cast<int>(i) != anchor_node_.id) {
      this->ComputePath(i, anchor_node_.id);
    }
  }
  sim3_to_anchor_[anchor_node_.id] = Sim3();
  timer.Pause();
  summary_.compute_final_transformation_time = timer.ElapsedSeconds();

  // 5. Merging all other reconstructions to anchor node
  LOG(INFO) << "Merging Reconstructions...";
  timer.Start();
  this->MergeReconstructions(vec_largest_cc_nodes);
  timer.Pause();
  summary_.merging_time = timer.ElapsedSeconds();

  return true;
}


bool SfMAligner::MyAlign() {
  Timer timer;

  // 1. Constructing a graph from reconstructions,
  // each node is a reconstruction, edges represent the connections between
  // reconstructions (by the means of common images or common 3D points), the
  // weight of edge represents the mean reprojection error.
  LOG(INFO) << "Constructing Reconstructions Graph...";
  timer.Start();
  // ConstructReconsGraph();
  ConstructReconsGraphWithOverlap();
  timer.Pause();
  summary_.construct_recon_graph_time = timer.ElapsedSeconds();
  recons_graph_.ShowInfo();
  CHECK_EQ(recons_graph_.GetNodesNum(), reconstructions_.size());

  // The reconstruction graph should be at least a spanning tree,
  // or we couldn't stitch all reconstructions together due to
  // too large alignment error or disconnected components.
  if (recons_graph_.GetEdgesNum() < recons_graph_.GetNodesNum() - 1) {
    LOG(WARNING) << "Can't align all reconstructions together due to "
                 << "too large alignment error or disconnected components."
                 << "We would just merge local maps in the largest connected "
                    "components.";
  }
  const Graph<Node, Edge> largest_cc = recons_graph_.ExtractLargestCC();
  // std::vector<size_t> vec_largest_cc_nodes;
  // vec_largest_cc_nodes.reserve(largest_cc.GetNodes().size());

  LOG(INFO)<<"connected nodes size: "<<largest_cc.GetNodes().size();
  std::set<size_t> connected_recons_ids;
  // std::vector<size_t> sorted_connected_recons_ids;
  for (auto node_it : largest_cc.GetNodes()) {
    // vec_largest_cc_nodes.push_back(node_it.first);
    connected_recons_ids.insert(node_it.first);
    // sorted_connected_recons_ids.push_back(node_it.first);
    // LOG(INFO)<<node_it.first;
  }
  for(auto node_it : connected_recons_ids)
  {
    LOG(INFO)<<node_it;
  }


  // // 2. Constructing a minimum spanning tree, thus we can select the
  // // most accurate n - 1 edges for accurate alignment.
  // LOG(INFO) << "Finding Minimum Spanning Tree...";
  // timer.Start();
  // std::vector<Edge> mst_edges = largest_cc.Kruskal();


  // Graph<Node, Edge> mst;
  // for (const auto edge : mst_edges) {
  //   mst.AddEdge(edge);
  // }

  // if (mst_edges.size() < largest_cc.GetNodesNum() - 1) {
  //   LOG(WARNING) << "Invalid MST";
  //   mst.ShowInfo();
  //   return false;
  // }
  // mst.ShowInfo();
  // timer.Pause();
  // summary_.construct_mst_time = timer.ElapsedSeconds();

  // // 3. Finding an anchor node, an anchor node is a reference reconstruction
  // // that all other reconstructions should be aligned to.
  // LOG(INFO) << "Finding Anchor Node...";
  // timer.Start();
  // FindAnchorNode(&mst);
  // timer.Pause();
  // summary_.find_anchor_node_time = timer.ElapsedSeconds();
  
  // LOG(INFO)<<"ReturnAllConnectedNodes...";
  // std::vector<size_t>direct_connected_nodes = recons_graph_.ReturnAllConnectedNodes(anchor_node_.id);

  LOG(INFO)<<"FindBestEdgeToMerge...";
  int src, dst;
  // FindBestEdgeToMerge(src,dst);
  recons_graph_.FindBestEdgeToMerge(connected_recons_ids,src,dst);
  LOG(INFO)<<"dicide to merge "<<src<<" to "<<dst;
  anchor_node_.id = dst;

  // this->ComputePath(src, anchor_node_.id);
  // sim3_to_anchor_.resize(max(src,dst));
  // sim3_to_anchor_[anchor_node_.id] = Sim3();
  // sim3_to_anchor_[src] = sim3_graph_[src][dst];
  Sim3 pose = sim3_graph_[src][dst];

  // LOG(INFO)<<pose.R(0,0)<<" "<<pose.R(0,1)<<" "<<pose.R(0,2);

  LOG(INFO) << "Merging One Reconstruction...";
  timer.Start();
  // size_t node_id = direct_connected_nodes[0];
  this->MergeOneReconstruction(src,pose);
  timer.Pause();
  summary_.merging_time = timer.ElapsedSeconds();


  // vector<Reconstruction*>::iterator itr=reconstructions_.begin() + src;
  // reconstructions_.erase(itr);

  std::vector<Reconstruction*> connected_reconstructions;
  for(int i=0; i<reconstructions_.size(); i++)
  {
    if(connected_recons_ids.count(i)!=0 && i!=src)
    {
      connected_reconstructions.push_back(reconstructions_[i]);
    }
  }
  reconstructions_ = connected_reconstructions;



  // // 4. Compute the final transformation to anchor node for each cluster
  // LOG(INFO) << "Computing Final Similarity Transformations...";
  // timer.Start();
  // for (auto i : vec_largest_cc_nodes) {
  //   if (static_cast<int>(i) != anchor_node_.id && connected_nodes_set.count(i)!=0) { //仅仅计算直接相连的components
  //     this->ComputePath(i, anchor_node_.id);
  //   }
  // }
  // sim3_to_anchor_[anchor_node_.id] = Sim3();
  // timer.Pause();
  // summary_.compute_final_transformation_time = timer.ElapsedSeconds();

  // // 5. Merging all other reconstructions to anchor node
  // LOG(INFO) << "Merging Reconstructions...";
  // timer.Start();
  // this->MergeReconstructions(vec_largest_cc_nodes);
  // timer.Pause();
  // summary_.merging_time = timer.ElapsedSeconds();

  //   // 5. Merging all other reconstructions to anchor node
  // LOG(INFO) << "Merging One Reconstruction...";
  // timer.Start();
  // size_t node_id = direct_connected_nodes[0];
  // this->MergeOneReconstruction(node_id);
  // timer.Pause();
  // summary_.merging_time = timer.ElapsedSeconds();






  return true;
}

Node SfMAligner::GetAnchorNode() const { return anchor_node_; }

std::vector<Sim3> SfMAligner::GetSim3ToAnchor() const {
  return sim3_to_anchor_;
}

const std::unordered_set<image_t>& SfMAligner::GetSeparators() const {
  return separators_;
}


void SfMAligner::FindBestEdgeToMerge(int& src, int& dst) {


  std::vector<EdgeForMerge> edges;
  for (uint i = 0; i < reconstructions_.size(); i++) {
    Reconstruction* recon1 = reconstructions_[i];

    for (uint j = i + 1; j < reconstructions_.size(); j++) {

      Reconstruction* recon2 = reconstructions_[j];
      const double weight = ComputeEdgeWeight(i, j);
        
      if (weight != std::numeric_limits<double>::max()) {
        LOG(INFO) << "weight between ("<<i<<","<<j<<"): " << weight;
        // recons_graph_.AddEdge(Edge(i, j, (float)weight));
        
        int common_img_count = 0;
        for (const auto& image_id : recon2->RegImageIds()) {
          if (recon1->ExistsImage(image_id)) {
            common_img_count++;
          }
        }

        EdgeForMerge edge(i,j,recon1->NumRegImages(),recon2->NumRegImages(),common_img_count,weight);
        edge.ComputeEdgeMetric();
        edges.push_back(edge);
      }
    }
  }

  auto const cmp = [](EdgeForMerge const edge1, EdgeForMerge const edge2){
    return edge1.metric_ > edge2.metric_;
  };
  sort(edges.begin(),edges.end(),cmp);
  
  if(edges[0].src_size_<edges[0].dst_size_)
  {
    src = edges[0].src_;
    dst = edges[0].dst_;
  }
  else
  {
    src = edges[0].dst_;
    dst = edges[0].src_;
  }
  LOG(INFO)<<"best edge: ("<<src<<", "<<dst<<") with "<<edges[0].common_img_size_<<" weight: "<<edges[0].weight_;



  // std::vector<std::pair<std::pair<size_t,size_t>>, float> edges_metrics;

  // for (auto it = edges_.begin(); it != edges_.end(); ++it) {
  //   auto em = it->second;
  //   if(static_cast<int>(it->first) == idx)
  //   {
  //     for(auto em_it = em.begin(); em_it != em.end(); em_it++){
  //       connected_nodes_weights.push_back(std::make_pair(em_it->first,em_it->second.weight));
  //     }
  //   }
  //   else
  //   {
  //     for (auto em_it = em.begin(); em_it != em.end(); em_it++) {
  //       if (static_cast<int>(em_it->first) == idx) {
  //         connected_nodes_weights.push_back(std::make_pair(it->first,em_it->second.weight));
  //       }
  //     }
  //   }
  // }
  // const auto cmp = [](const std::pair<size_t,float>pair1,const std::pair<size_t,float>pair2){
  //   return pair1.second<pair2.second;
  // };
  // sort(connected_nodes_weights.begin(),connected_nodes_weights.end(),cmp);
  // for(int i=0; i<connected_nodes_weights.size(); i++)
  // {
  //   connected_nodes.push_back(connected_nodes_weights[i].first);
  // }
  // return connected_nodes;
}


void SfMAligner::ConstructReconsGraphWithOverlap() {
  // 1. Add nodes
  for (size_t i = 0; i < reconstructions_.size(); i++) {
    Node node(i);
    // node.recon = reconstructions_[i];
    recons_graph_.AddNode(node);
  }

  // 2. Add edges
  for (uint i = 0; i < reconstructions_.size(); i++) {
    for (uint j = i + 1; j < reconstructions_.size(); j++) {
      // const double weight = ComputeEdgeWeight(i, j);
      int overlap_size = 0;
      const double weight = ComputeEdgeWeightWithOverlap(i, j, overlap_size);
        
      if (weight != std::numeric_limits<double>::max()) {
        // double metric = (double)overlap_size/weight; //overlap较多的优先merge，会导致最后的merge part全是没啥overlap的
        double metric = 1.0d/weight;
        LOG(INFO) << "weight between ("<<i<<","<<j<<"): " << weight<<" overlap: "<<overlap_size<<" with metric: "<<metric;
        // recons_graph_.AddEdge(Edge(i, j, (float)weight));
        recons_graph_.AddEdge(Edge(i, j, (float)metric));
      }
    }
  }
}


void SfMAligner::ConstructReconsGraph() {
  // 1. Add nodes
  for (size_t i = 0; i < reconstructions_.size(); i++) {
    Node node(i);
    // node.recon = reconstructions_[i];
    recons_graph_.AddNode(node);
  }

  // 2. Add edges
  for (uint i = 0; i < reconstructions_.size(); i++) {
    for (uint j = i + 1; j < reconstructions_.size(); j++) {
      const double weight = ComputeEdgeWeight(i, j);
        
      if (weight != std::numeric_limits<double>::max()) {
        LOG(INFO) << "graph weight between ("<<i<<","<<j<<"): " << weight;
        recons_graph_.AddEdge(Edge(i, j, (float)weight));
      }
    }
  }
}

double SfMAligner::ComputeEdgeWeight(const uint i, const uint j) {
  const Reconstruction& recon1 = *reconstructions_[i];
  const Reconstruction& recon2 = *reconstructions_[j];
  double weight = std::numeric_limits<double>::max();

  Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity(3, 3),
                  R2 = Eigen::Matrix3d::Identity(3, 3);
  Eigen::Vector3d t1 = Eigen::Vector3d::Zero(), t2 = Eigen::Vector3d::Zero();
  double s1 = 1.0, s2 = 1.0;

  // Find common registered images
  std::vector<image_t> common_reg_images = recon1.FindCommonRegImageIds(recon2);
  for (auto image_id : common_reg_images) {
    separators_.insert(image_id);
  }
  std::vector<Eigen::Vector3d> src_points, ref_points;
  // for (const auto common_id : common_reg_images) {
  //     src_points.push_back(recon1.Image(common_id).ProjectionCenter());
  //     ref_points.push_back(recon2.Image(common_id).ProjectionCenter());
  // }
  FindCommon3DPoints(common_reg_images, recon1, recon2, src_points, ref_points);
  if(common_reg_images.size()>=2)
    LOG(INFO) << "Common registerd images number: " << common_reg_images.size();

  if (common_reg_images.size() < 2) {
    // LOG(WARNING) << "Not found enough common registered images.";
    return std::numeric_limits<double>::max();
  } else {
    double msd1 = 0.0, msd2 = 0.0;
    FindSimilarityTransform(src_points, ref_points, options_.threshold,
                            options_.confidence, R1, t1, s1, msd1); //1配2
    FindSimilarityTransform(ref_points, src_points, options_.threshold,
                            options_.confidence, R2, t2, s2, msd2); //2配1

    weight = std::max(msd1, msd2);

    if (weight != numeric_limits<double>::max()) {
      sim3_graph_[i][j] = Sim3(R1, t1, s1);
      sim3_graph_[j][i] = Sim3(R2, t2, s2);
    }
  }
  // else if (common_reg_images.size() < 3) {
  //     std::vector<Eigen::Matrix3d> src_rotations, ref_rotations;
  //     for (const auto common_id : common_reg_images) {
  //         src_rotations.push_back(recon1.Image(common_id).RotationMatrix());
  //         ref_rotations.push_back(recon2.Image(common_id).RotationMatrix());
  //     }
  //     ComputeSimilarityByCameraMotions(src_points, ref_points,
  //                                      src_rotations, ref_rotations, R1, t1,
  //                                      s1);
  //     ComputeSimilarityByCameraMotions(ref_points, src_points,
  //                                      ref_rotations, src_rotations, R2, t2,
  //                                      s2);

  //     double msd1 = CheckReprojError(src_points, ref_points, s1, R1, t1),
  //            msd2 = CheckReprojError(ref_points, src_points, s2, R2, t2);

  //     weight = std::max(msd1, msd2);

  //     if (weight != numeric_limits<double>::max()) {
  //         sim3_graph_[i][j] = Sim3(R1, t1, s1);
  //         sim3_graph_[j][i] = Sim3(R2, t2, s2);
  //     }
  // }

  return (weight > options_.max_reprojection_error)
             ? std::numeric_limits<double>::max()
             : weight;
}


double SfMAligner::ComputeEdgeWeightWithOverlap(const uint i, const uint j, int& common_img_size) {
  const Reconstruction& recon1 = *reconstructions_[i];
  const Reconstruction& recon2 = *reconstructions_[j];
  double weight = std::numeric_limits<double>::max();

  Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity(3, 3),
                  R2 = Eigen::Matrix3d::Identity(3, 3);
  Eigen::Vector3d t1 = Eigen::Vector3d::Zero(), t2 = Eigen::Vector3d::Zero();
  double s1 = 1.0, s2 = 1.0;

  // Find common registered images
  std::vector<image_t> common_reg_images = recon1.FindCommonRegImageIds(recon2);
  common_img_size = common_reg_images.size();

  for (auto image_id : common_reg_images) {
    separators_.insert(image_id);
  }
  std::vector<Eigen::Vector3d> src_points, ref_points;
  // for (const auto common_id : common_reg_images) {
  //     src_points.push_back(recon1.Image(common_id).ProjectionCenter());
  //     ref_points.push_back(recon2.Image(common_id).ProjectionCenter());
  // }
  FindCommon3DPoints(common_reg_images, recon1, recon2, src_points, ref_points);
  if(common_reg_images.size()>=2)
    LOG(INFO) << "Common registerd images number: " << common_reg_images.size();

  if (common_reg_images.size() < 2) {
    // LOG(WARNING) << "Not found enough common registered images.";
    return std::numeric_limits<double>::max();
  } else {
    double msd1 = 0.0, msd2 = 0.0;
    FindSimilarityTransform(src_points, ref_points, options_.threshold,
                            options_.confidence, R1, t1, s1, msd1); //1配2
    FindSimilarityTransform(ref_points, src_points, options_.threshold,
                            options_.confidence, R2, t2, s2, msd2); //2配1

    weight = std::max(msd1, msd2);

    if (weight != numeric_limits<double>::max()) {
      sim3_graph_[i][j] = Sim3(R1, t1, s1);
      sim3_graph_[j][i] = Sim3(R2, t2, s2);
    }
  }
  // else if (common_reg_images.size() < 3) {
  //     std::vector<Eigen::Matrix3d> src_rotations, ref_rotations;
  //     for (const auto common_id : common_reg_images) {
  //         src_rotations.push_back(recon1.Image(common_id).RotationMatrix());
  //         ref_rotations.push_back(recon2.Image(common_id).RotationMatrix());
  //     }
  //     ComputeSimilarityByCameraMotions(src_points, ref_points,
  //                                      src_rotations, ref_rotations, R1, t1,
  //                                      s1);
  //     ComputeSimilarityByCameraMotions(ref_points, src_points,
  //                                      ref_rotations, src_rotations, R2, t2,
  //                                      s2);

  //     double msd1 = CheckReprojError(src_points, ref_points, s1, R1, t1),
  //            msd2 = CheckReprojError(ref_points, src_points, s2, R2, t2);

  //     weight = std::max(msd1, msd2);

  //     if (weight != numeric_limits<double>::max()) {
  //         sim3_graph_[i][j] = Sim3(R1, t1, s1);
  //         sim3_graph_[j][i] = Sim3(R2, t2, s2);
  //     }
  // }

  return (weight > options_.max_reprojection_error)
             ? std::numeric_limits<double>::max()
             : weight;
}

void SfMAligner::FindAnchorNode(Graph<Node, Edge>* graph) {
  paths_.resize(recons_graph_.GetNodesNum());
  sim3_to_anchor_.resize(recons_graph_.GetNodesNum());

  // The anchor is found by merging all leaf nodes to their adjacent nodes,
  // until one node or two nodes left. If two nodes left, we choose the
  // reconstruction that has the largest size as the anchor.
  int layer = 1;
  uint anchor_index = 0;

  while (graph->GetNodesNum() > 1) {
    LOG(INFO) << "Merging the " << layer++ << "-th layer leaf nodes";

    graph->CountOutDegrees();
    graph->CountInDegrees();
    graph->CountDegrees();
    std::unordered_map<size_t, size_t> degrees = graph->GetDegrees();

    // Finding all leaf nodes. Leaf node in graph has degree equals to 1.
    std::vector<size_t> indexes;
    if (graph->GetNodesNum() == 2) {
      indexes.push_back(degrees.begin()->first);
    } else {
      for (auto it = degrees.begin(); it != degrees.end(); ++it) {
        LOG(INFO) << "node: " << it->first << ", "
                  << "degree: " << it->second;
        if (it->second == 1) indexes.push_back(it->first);
      }
    }
    if (indexes.empty()) break;

    for (auto idx : indexes) {
      // if (idx == -1) break;
      const Edge& edge = graph->FindConnectedEdge(idx);

      LOG(INFO) << "Find node [degree = 1]: " << idx;
      LOG(INFO) << edge.src << "->" << edge.dst << ": " << edge.weight;

      // src is the node with degree = 1
      uint src = (idx == edge.src) ? edge.src : edge.dst;
      uint dst = (idx == edge.src) ? edge.dst : edge.src;

      LOG(INFO) << "Merge Clusters: " << src << "->" << dst << ": "
                << edge.weight;
      anchor_index = dst;
      const Sim3 sim = sim3_graph_[src][dst];
      paths_[src].insert(std::make_pair(dst, sim));

      graph->DeleteNode(src);
      graph->DeleteEdge(src, dst);
      graph->DeleteEdge(dst, src);
      graph->ShowInfo();
    }
  }

  anchor_node_.id = anchor_index;
}

void SfMAligner::ComputePath(int src, int dst) {
  LOG(INFO) << "Computing Path: " << src << "->" << dst;
  std::queue<int> qu;
  qu.push(src);

  Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double s = 1.0;

  Sim3 sim(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Identity(), 1.0);
  LOG(INFO) << "v: " << src;
  while (!qu.empty()) {
    int u = qu.front();
    qu.pop();
    auto it = paths_[u].begin();
    int v = it->first;
    LOG(INFO) << "v: " << v;
    s = it->second.s * s;
    r = it->second.R * r.eval();
    t = it->second.s * it->second.R * t.eval() + it->second.t;
    if (v == dst) {
      sim.s = s;
      sim.R = r;
      sim.t = t;
      sim3_to_anchor_[src] = sim;
      return;
    } else
      qu.push(v);
  }
  LOG(INFO) << "\n";
}

void SfMAligner::MergeOneReconstruction(size_t node_id, Sim3 sim3){

  // Sim3 sim3 = sim3_to_anchor_[node_id];
  Eigen::Matrix3x4d alignment;
  alignment.block(0, 0, 3, 3) = sim3.s * sim3.R;
  alignment.block(0, 3, 3, 1) = sim3.t;
  LOG(INFO)<<"merge reconstruction ["<<node_id<<"] to anchor node: "<<anchor_node_.id;
  reconstructions_[anchor_node_.id]->Merge(*reconstructions_[node_id], alignment);

  Reconstruction* recon =  reconstructions_[anchor_node_.id];
   
  const std::vector<image_t>& reg_image_ids = recon->RegImageIds();

  CHECK_GE(reg_image_ids.size(), 2) << "At least two images must be "
                                       "registered for global "
                                       "bundle-adjustment";

  // Avoid degeneracies in bundle adjustment.
  recon->FilterObservationsWithNegativeDepth();

  // Configure bundle adjustment.
  BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);
  }

  // // Fix the existing images, if option specified.
  // if (options.fix_existing_images) {
  //   for (const image_t image_id : reg_image_ids) {
  //     if (existing_image_ids_.count(image_id)) {
  //       ba_config.SetConstantPose(image_id);
  //     }
  //   }
  // }

  // Fix 7-DOFs of the bundle adjustment problem.
  ba_config.SetConstantPose(reg_image_ids[0]);
  // if (!options.fix_existing_images ||
  //     !existing_image_ids_.count(reg_image_ids[1])) {
  //   ba_config.SetConstantTvec(reg_image_ids[1], {0});
  // }

  BundleAdjustmentOptions ba_options;
  // Run bundle adjustment.
  BundleAdjuster bundle_adjuster(ba_options, ba_config);
  bundle_adjuster.Solve(recon);
  // if (!bundle_adjuster.Solve(recon)) {
  //   return false;
  // }

  // Normalize scene for numerical stability and
  // to avoid large scale changes in viewer.
  recon->Normalize();





  // seq_merg_reconstructions_.clear();
  // seq_merg_reconstructions_.reserve(node_ids.size());
  seq_merg_reconstructions_.push_back(*reconstructions_[anchor_node_.id]);

  // LOG(INFO)<<"anchor node id: "<<anchor_node_.id;
  // for (auto id : node_ids) {
  //   if (static_cast<int>(id) == anchor_node_.id) {
  //     continue;
  //   }

  //   Sim3 sim3 = sim3_to_anchor_[id];
  //   Eigen::Matrix3x4d alignment;
  //   alignment.block(0, 0, 3, 3) = sim3.s * sim3.R;
  //   alignment.block(0, 3, 3, 1) = sim3.t;

  //   LOG(INFO)<<"before merging node "<<id<<" to anchor node "<<anchor_node_.id;
  //   LOG(INFO)<<"node cameras:"<<reconstructions_[id]->NumCameras()<<" images:"<<reconstructions_[id]->NumImages()<<" points:"<<reconstructions_[id]->NumPoints3D();
  //   LOG(INFO)<<"anchor cameras:"<<reconstructions_[anchor_node_.id]->NumCameras()<<" images:"<<reconstructions_[anchor_node_.id]->NumImages()<<" points:"<<reconstructions_[anchor_node_.id]->NumPoints3D();
  //   LOG(INFO)<<seq_merg_reconstructions_.size()<<" Merging node id: "<<id;
  //   reconstructions_[anchor_node_.id]->Merge(*reconstructions_[id], alignment);
  //   LOG(INFO)<<"saving seq merging results";
  //   LOG(INFO)<<"anchor cameras:"<<reconstructions_[anchor_node_.id]->NumCameras()<<" images:"<<reconstructions_[anchor_node_.id]->NumImages()<<" points:"<<reconstructions_[anchor_node_.id]->NumPoints3D();
  //   seq_merg_reconstructions_.push_back(*reconstructions_[anchor_node_.id]);
  // }
  // reconstructions_.clear();
  // for(int i=0;i<seq_merg_reconstructions_.size();i++)
  // {
  //   reconstructions_.push_back(&seq_merg_reconstructions_[i]);
  // }
  // reconstructions_=seq_merg_reconstructions;//saving all seq merges

  // LOG(INFO)<<"check reconstructions_";
  // for(int i=0;i<reconstructions_.size();i++)
  // {
  //   LOG(INFO)<<"cameras:"<<reconstructions_[i]->NumCameras()<<" images:"<<reconstructions_[i]->NumImages()<<" points:"<<reconstructions_[i]->NumPoints3D();
  // }
}



void SfMAligner::MergeReconstructions(std::vector<size_t>& node_ids) {
  if (options_.assign_color_for_clusters) {
    for (size_t i = 0; i < reconstructions_.size(); i++) {
      const int color_id = i % SfMAligner::ColorContainers.size();
      reconstructions_[i]->AssignColorsForAllPoints(
          SfMAligner::ColorContainers[color_id]);
      // // Assign cluster id for each image.
      // const std::vector<image_t> reg_image_ids =
      // reconstructions_[i]->RegImageIds(); for (auto image_id : reg_image_ids)
      // {
      //     Image& image = reconstructions_[i]->Image(image_id);
      //     image.SetClusterId(i);
      // }
    }
  }

  // std::vector<Reconstruction> seq_merg_reconstructions;
  seq_merg_reconstructions_.clear();
  seq_merg_reconstructions_.reserve(node_ids.size());
  seq_merg_reconstructions_.push_back(*reconstructions_[anchor_node_.id]);

  LOG(INFO)<<"anchor node id: "<<anchor_node_.id;
  for (auto id : node_ids) {
    if (static_cast<int>(id) == anchor_node_.id) {
      continue;
    }

    Sim3 sim3 = sim3_to_anchor_[id];
    Eigen::Matrix3x4d alignment;
    alignment.block(0, 0, 3, 3) = sim3.s * sim3.R;
    alignment.block(0, 3, 3, 1) = sim3.t;

    LOG(INFO)<<"before merging node "<<id<<" to anchor node "<<anchor_node_.id;
    LOG(INFO)<<"node cameras:"<<reconstructions_[id]->NumCameras()<<" images:"<<reconstructions_[id]->NumImages()<<" points:"<<reconstructions_[id]->NumPoints3D();
    LOG(INFO)<<"anchor cameras:"<<reconstructions_[anchor_node_.id]->NumCameras()<<" images:"<<reconstructions_[anchor_node_.id]->NumImages()<<" points:"<<reconstructions_[anchor_node_.id]->NumPoints3D();
    LOG(INFO)<<seq_merg_reconstructions_.size()<<" Merging node id: "<<id;
    reconstructions_[anchor_node_.id]->Merge(*reconstructions_[id], alignment);
    LOG(INFO)<<"saving seq merging results";
    LOG(INFO)<<"anchor cameras:"<<reconstructions_[anchor_node_.id]->NumCameras()<<" images:"<<reconstructions_[anchor_node_.id]->NumImages()<<" points:"<<reconstructions_[anchor_node_.id]->NumPoints3D();
    seq_merg_reconstructions_.push_back(*reconstructions_[anchor_node_.id]);
  }
  reconstructions_.clear();
  for(int i=0;i<seq_merg_reconstructions_.size();i++)
  {
    reconstructions_.push_back(&seq_merg_reconstructions_[i]);
  }
  // reconstructions_=seq_merg_reconstructions;//saving all seq merges

  LOG(INFO)<<"check reconstructions_";
  for(int i=0;i<reconstructions_.size();i++)
  {
    LOG(INFO)<<"cameras:"<<reconstructions_[i]->NumCameras()<<" images:"<<reconstructions_[i]->NumImages()<<" points:"<<reconstructions_[i]->NumPoints3D();
  }

}

bool ComputeSimilarityByCameraMotions(
    std::vector<Eigen::Vector3d>& camera_centers1,
    std::vector<Eigen::Vector3d>& camera_centers2,
    std::vector<Eigen::Matrix3d>& camera_rotations1,
    std::vector<Eigen::Matrix3d>& camera_rotations2,
    Eigen::Matrix3d& relative_r, Eigen::Vector3d& relative_t, double& scale) {
  // my hybrid approach by combining "Divide and Conquer: Efficient Large-Scale
  // Structure from Motion Using Graph Partitioning" and RANSAC

  const uint n = camera_centers1.size();
  std::vector<Eigen::Vector3d> ts1(n);
  std::vector<Eigen::Vector3d> ts2(n);

  for (uint i = 0; i < n; i++) {
    ts1[i] = -camera_rotations1[i] * camera_centers1[i];
    ts2[i] = -camera_rotations2[i] * camera_centers2[i];
  }

  // compute relative scale from a->b
  std::vector<double> scales;
  for (uint i = 0; i < n; i++) {
    Eigen::Vector3d center_a1 = camera_centers1[i];
    Eigen::Vector3d center_b1 = camera_centers2[i];
    for (uint j = i + 1; j < n; j++) {
      Eigen::Vector3d center_a2 = camera_centers1[j];
      Eigen::Vector3d center_b2 = camera_centers2[j];
      double scale_ab =
          (center_b1 - center_b2).norm() / (center_a1 - center_a2).norm();
      scales.push_back(scale_ab);
    }
  }
  // retrieve the median of scales, according to
  // the equation (5) of the paper "Divide and Conquer: Efficient Large-Scale
  // Structure from Motion Using Graph Partitioning"
  std::sort(scales.begin(), scales.end());
  scale = scales[scales.size() / 2];

  // compute relative rotation & relative translation from a->b
  std::vector<Correspondence3D> corres3d;
  std::vector<CorrespondenceEuc> input_datas;
  for (uint i = 0; i < camera_centers1.size(); i++) {
    corres3d.emplace_back(camera_centers1[i], camera_centers2[i]);
    input_datas.push_back(make_pair(Euclidean3D(camera_rotations1[i], ts1[i]),
                                    Euclidean3D(camera_rotations2[i], ts2[i])));
  }
  EuclideanEstimator euc_estimator(scale, corres3d);

  Euclidean3D euc3d;
  RansacParameters params;
  params.rng =
      std::make_shared<RandomNumberGenerator>((unsigned int)time(NULL));
  params.error_thresh = 0.002;
  params.max_iterations = 1000;

  Prosac<EuclideanEstimator> prosac_euc3(params, euc_estimator);
  prosac_euc3.Initialize();
  RansacSummary summary;
  prosac_euc3.Estimate(input_datas, &euc3d, &summary);

  relative_r = euc3d.R;
  relative_t = euc3d.t;

  return true;
}

}  // namespace DAGSfM