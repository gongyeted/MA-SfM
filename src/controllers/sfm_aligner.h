#ifndef SRC_CONTROLLERS_SFM_ALIGNER_H_
#define SRC_CONTROLLERS_SFM_ALIGNER_H_

#include <memory>
#include <vector>

#include "base/reconstruction.h"
#include "base/similarity_transform.h"
#include "estimators/sim3.h"
#include "graph/graph.h"
#include "optim/bundle_adjustment.h"
#include "sfm/twoview_info.h"
#include "util/bitmap.h"

using namespace colmap;
using namespace DAGSfM::graph;

namespace DAGSfM {

using SimilarityTransformGraph =
    std::unordered_map<size_t, std::unordered_map<size_t, Sim3>>;

class SfMAligner {
 public:
  const static std::vector<BitmapColor<float>> ColorContainers;
  struct AlignOptions {
    bool merge_largest_cc = true;

    double threshold = 0.1;

    double confidence = 0.99;

    double max_reprojection_error = 1.8;

    bool assign_color_for_clusters = false;

    std::string image_path = "";
  };

  struct Summary {
    double construct_recon_graph_time;

    double construct_mst_time;

    double find_anchor_node_time;

    double compute_final_transformation_time;

    double merging_time;
  };

  struct EdgeForMerge{

    size_t src_;
    size_t dst_;
    int src_size_;
    int dst_size_;
    int common_img_size_;
    float weight_;
    float metric_;

    
    EdgeForMerge(size_t src, size_t dst, 
              int src_size, int dst_size, 
              int common_img_size, float weight){
      src_ = src;
      dst_ = dst;
      src_size_ = src_size;
      dst_size_ = dst_size;
      common_img_size_ = common_img_size;
      weight_ = weight;
    }

    void ComputeEdgeMetric(){
      metric_ = (float)common_img_size_/weight_;
    }
  };


  SfMAligner(const std::vector<Reconstruction*>& reconstructions,
             const AlignOptions& options);

  bool Align();

  bool MyAlign();

  Node GetAnchorNode() const;

  std::vector<Sim3> GetSim3ToAnchor() const;

  const std::unordered_set<image_t>& GetSeparators() const;

  std::vector<Reconstruction*> reconstructions_;
  std::vector<Reconstruction> seq_merg_reconstructions_;

 private:
  AlignOptions options_;

  SfMAligner::Summary summary_;

  // std::vector<Reconstruction*> reconstructions_;


  Graph<Node, Edge> recons_graph_;

  Node anchor_node_;

  // The similarity transformation to anchor node, which is used for final
  // transformation
  std::vector<Sim3> sim3_to_anchor_;

  std::vector<std::unordered_map<size_t, Sim3>> paths_;

  // similarity graph, each cluster is represented as a node,
  // and each edge is the relative similarity transformation
  SimilarityTransformGraph sim3_graph_;

  std::unordered_set<image_t> separators_;
  // std::unordered_set<image_t> seperators_for_this_recon_;

  void ConstructReconsGraph();
  void ConstructReconsGraphWithOverlap();

  void FindBestEdgeToMerge(int& src, int& dst);

  double ComputeEdgeWeight(const uint i, const uint j);
  double ComputeEdgeWeightWithOverlap(const uint i, const uint j, int& common_img_size);

  void FindAnchorNode(Graph<Node, Edge>* graph);

  void ComputePath(int src, int dst);

  void MergeReconstructions(std::vector<size_t>& node_ids);

  void MergeOneReconstruction(size_t node_id, Sim3 sim3);
};

bool ComputeSimilarityByCameraMotions(
    std::vector<Eigen::Vector3d>& camera_centers1,
    std::vector<Eigen::Vector3d>& camera_centers2,
    std::vector<Eigen::Matrix3d>& camera_rotations1,
    std::vector<Eigen::Matrix3d>& camera_rotations2,
    Eigen::Matrix3d& relative_r, Eigen::Vector3d& relative_t, double& scale);

}  // namespace DAGSfM

#endif