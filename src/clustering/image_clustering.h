// BSD 3-Clause License

// Copyright (c) 2020, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef SRC_CLUSTERING_IMAGE_CLUSTERING_H_
#define SRC_CLUSTERING_IMAGE_CLUSTERING_H_

#include <algorithm>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "clustering/cluster.h"
#include "graph/graph.h"
#include "util/hash.h"
#include "util/timer.h"
#include "util/types.h"

#include "rotation_estimation/nonlinear_rotation_estimator.h"
#include "rotation_estimation/robust_rotation_estimator.h"
#include "rotation_estimation/rotation_estimator.h"
#include "rotation_estimation/lagrange_dual_rotation_estimator.h"
#include "sfm/filter_view_pairs_from_orientation.h"

using namespace colmap;

namespace DAGSfM {

using Edges = std::unordered_map<ImagePair, int>;
struct ImageCluster {
  int cluster_id = 0;
  std::vector<image_t> image_ids;
  Edges edges;
  bool is_condition_satisfy = false;
  bool completed = false;

  float ComputeSelfSimilarityScore(int& max_matches);

  void ShowInfo() const {
    std::string info = "Cluster " + std::to_string(cluster_id) + ": [";
    info += ("node: " + std::to_string(image_ids.size()));
    // for (auto image_id : image_ids) {
    //   std::cout << image_id << " ";
    // }
    // std::cout << "\n";

    info += (", edges: " + std::to_string(edges.size()) + "]");
    LOG(INFO) << info;
  }
};

class ImageClustering {
 public:
  struct Options {
    // The upper bound of images number
    uint num_images_ub = 100;

    uint metric_num_imgs_ub = 5;//实际上只有当cluster内数量大于等于2*5时才会将其分割成两个cluster

    uint cluster_subcluster_num_ub = 5000;//200;//900;//50;//900;//50;//30; //20//cluster中包含的subcluster数量上限，最终包含的影像数 = cluster_subcluster_num_ub * metric_num_imgs_ub

    // The number of overlapping images between child clusters.
    // uint image_overlap = 50;
    uint image_overlap = 15; //for subclusters

    // completeness ratio for selecting expanded clusters
    float completeness_ratio = 0.5;

    float relax_ratio = 1.3;

    // the number of cluster we partition each time
    int branching_factor = 2;

    // Maximum number of edges for building connections between clusters
    uint max_num_cluster_pairs = 0;

    bool is_output_igraph = true;

    std::string cluster_type = "NCUT";

    std::string graph_dir = "";

    bool Check() const;
  };

  struct Summary {
    // The number of grpah cut
    uint total_cutting_num = 0;

    // The number of graph expansion
    uint total_expansion_num = 0;

    // Total time took for graph cut
    double total_cutting_time = 0;

    // Total time took for graph expansion
    double total_expansion_time = 0;

    // Total time time for images clustering
    double total_time = 0;

    // Original number of images
    uint original_images_num = 0;

    // Total number of images after images clustering
    uint clustered_images_num = 0;

    // Original number of images' edges
    uint original_edges_num = 0;

    // Total number of edges after images clustering
    uint clustered_edges_num = 0;

    // Total iteration number
    uint total_iters_num = 0;
  };

  ImageClustering(const Options& options, const ImageCluster& root_cluster);

  virtual void CutByMetric(std::unordered_map<int, int>& labels, int& cluster_num); //my code
  virtual void CutSubClusters(std::unordered_map<int, int>& labels, int& cluster_num);
  virtual void CutUnconnectedClusters( const std::vector<image_t>& image_ids,
                                        const std::vector<std::pair<int, int>>& edges,
                                        std::unordered_map<size_t, std::vector<image_t>>& components); //cut cluster to unconnected components
  virtual void Cut();
  virtual void GroupingImageEdges(std::vector<std::vector<image_t>> subclusters_imgs, std::unordered_map<int, int> labels); //my code
  // virtual void GroupingImageEdges(std::unordered_map<int, int> labels, int cluster_num); //my code
  virtual void FilterImageEdges(std::unordered_map<size_t, float>& self_similarity_scores, 
                                std::unordered_map<ImagePair, float>& cross_similarity_ratios);//my code 计算subcluster类内聚合度和类间相似度，依据固定阈值划分为强edges和弱edges
  virtual std::unordered_map<int, Edges> GetBestEdges(std::vector<ImageCluster>& subclusters,  
            std::unordered_map<ImagePair, std::vector<graph::Edge>>& inter_subcluster_edges, 
            const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs, 
            const double max_relative_rotation_difference_degrees);


  virtual void Expand();
  virtual void ExpandByStrongEdges();

  virtual void ExpandAllEdges();

  virtual void CutAndExpand();

  std::vector<ImageCluster> BiCut(const ImageCluster& cluster);

  const std::vector<ImageCluster>& GetIntraClusters() const;
  const std::vector<ImageCluster>& GetInterClusters() const;
  const ImageCluster& GetRootCluster() const;
    std::unordered_map<ImagePair, std::vector<graph::Edge>> GetAllInterClusterEdges();

  void OutputClusteringSummary() const;
  

//  private:
public:
  Options options_;

  Summary summary_;

  Timer timer_;

  const ImageCluster root_cluster_;

  // clusters that have no connection with each other
  std::vector<ImageCluster> intra_clusters_; //my: before expansion

  // clusters that shares common images
  std::vector<ImageCluster> inter_clusters_; //my: after expansion

  // The discarded edges after image clustering
  graph::LargerEdgePriorityQueue<graph::Edge> discarded_edges_;
  std::unordered_map<ImagePair, std::vector<graph::Edge>> clusters_lost_edges_;
  std::unordered_map<ImagePair, std::vector<graph::Edge>> subclusters_strong_edges_;//此处的imagePair实际上是subcluster pair, 存放较为可靠的（strong）edges（ratio>0.02）
  std::unordered_map<ImagePair, std::vector<graph::Edge>> subclusters_all_edges_;//此处的imagePair实际上是subcluster pair，存放所有edges
  // std::unordered_map<ImagePair, std::vector<graph::Edge>> subclusters_best_edges_; //此处的imagePair实际上是subcluster pair，存放每个cluster间超过前70%匹配数的edges



  double AnalyzeDegree(const std::vector<std::pair<int, int>>& image_pairs,
                       const std::vector<int>& weights) const;

  std::unique_ptr<Cluster> CreateCluster() const;

  std::unique_ptr<Cluster> CreateCluster(
      const std::vector<std::pair<int, int>>& image_pairs,
      const std::vector<int>& weights);

  bool IsSatisfyCompletenessRatio(const ImageCluster& cluster);
  int ClusterSatisfyCompletenessRatio(const graph::Edge& edge);

  uint CommonImagesNum(const ImageCluster& cluster1,
                       const ImageCluster& cluster2) const;

  bool IsRemainingClusters() const;

  void AddLostEdgesBetweenClusters(ImageCluster& cluster1,
                                   ImageCluster& cluster2,
                                   std::vector<graph::Edge>& lost_edges);

  void AnalyzeStatistic();
  void MyOutputClustersExpanded(std::string graph_dir);
};

}  // namespace DAGSfM

#endif