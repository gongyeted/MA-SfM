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

#include "clustering/image_clustering.h"

#include <glog/logging.h>

#include "clustering/community_detection_cluster.h"
#include "clustering/hybrid_cluster.h"
#include "clustering/kmeans_cluster.h"
#include "clustering/ncut_cluster.h"
#include "clustering/spectral_cluster.h"
#include "clustering/metric_cluster.h"
#include "graph/graph.h"
#include "util/map_util.h"
#include "util/misc.h"
#include "util/random.h"

namespace DAGSfM {

bool ImageClustering::Options::Check() const {
  CHECK_GT(image_overlap, 2);
  CHECK_LE(completeness_ratio, 1.0);
  CHECK_GT(num_images_ub, 0);
  CHECK_GT(branching_factor, 0);
  if (is_output_igraph && graph_dir.size() == 0) {
    return false;
  }
  return true;
}

ImageClustering::ImageClustering(const Options& options,
                                 const ImageCluster& root_cluster)
    : options_(options), root_cluster_(root_cluster) {
  CHECK(options_.Check());
  summary_.original_images_num = root_cluster_.image_ids.size();
  summary_.original_edges_num = root_cluster_.edges.size();
}


void ImageClustering::CutByMetric(std::unordered_map<int, int>& labels, int& cluster_num)
{
  timer_.Start();
  const uint num_clusters =
      (root_cluster_.image_ids.size()-1) / options_.num_images_ub +1;//大于num_images_ub开始拆分
  LOG(INFO)<<"num clusters: "<<num_clusters;
  CHECK_GE(num_clusters, 1);
  std::vector<image_t> image_ids = root_cluster_.image_ids;

  std::vector<std::pair<int, int>> image_pairs;
  std::vector<int> weights;
  image_pairs.reserve(root_cluster_.edges.size());
  weights.reserve(root_cluster_.edges.size());

  // if(root_cluster_.image_ids.size()==7)
  // {
  //   LOG(INFO)<<"notice here";
  //   for(int i=0; i<7; i++)
  //   {
  //     LOG(INFO)<<image_ids[i];
  //   }
  // }

  for (auto edge : root_cluster_.edges) {
    image_pairs.push_back(std::make_pair(edge.first.first, edge.first.second));
    weights.push_back(edge.second);

      // if(root_cluster_.image_ids.size()==7)
      // {
      //   LOG(INFO) <<"("<<edge.first.first<<", "<<edge.first.second<<") weight: "<<edge.second;
      // }
      
  }

  LOG(INFO) << "Images Metric Clustering Started ";


// //   // Clustering images
//   image_pairs.clear();
//   weights.clear();

//     image_pairs.push_back(std::make_pair(3407, 3408));
//   weights.push_back(6683);
//     image_pairs.push_back(std::make_pair(3409, 3410));
//   weights.push_back(4642);
//     image_pairs.push_back(std::make_pair(1729, 1730));
//   weights.push_back(3530);
//   image_pairs.push_back(std::make_pair(3407, 3410));
//   weights.push_back(4009);
//     image_pairs.push_back(std::make_pair(3409, 3411));
//   weights.push_back(3764);
//     image_pairs.push_back(std::make_pair(1730, 3407));
//   weights.push_back(3818);

// //   // Clustering images
//   image_pairs.clear();
//   weights.clear();
//   image_pairs.push_back(std::make_pair(1729, 1730));
//   weights.push_back(3530);
//     image_pairs.push_back(std::make_pair(3409,3410));
//   weights.push_back(4642);
//     image_pairs.push_back(std::make_pair(3407,3408));
//   weights.push_back(6683);
//     image_pairs.push_back(std::make_pair(3407,3410));
//   weights.push_back(4009);
//     image_pairs.push_back(std::make_pair(1730,3407));
//   weights.push_back(3818);
//     image_pairs.push_back(std::make_pair(3409,3411));
//   weights.push_back(3764);

  // ImageCluster image_cluster_t;
  // image_cluster_t.image_ids.push_back(1729);
  // image_cluster_t.image_ids.push_back(1730);
  // image_cluster_t.image_ids.push_back(3407);
  // image_cluster_t.image_ids.push_back(3408);
  // image_cluster_t.image_ids.push_back(3409);
  // image_cluster_t.image_ids.push_back(3410);
  // image_cluster_t.image_ids.push_back(3411);
  // image_cluster_t.edges[std::make_pair(3407, 3410)] = 4009;
  // image_cluster_t.edges[std::make_pair(3409, 3411)] = 3764;
  // image_cluster_t.edges[std::make_pair(1730, 3407)] = 3818;
  // image_cluster_t.edges[std::make_pair(3407, 3408)] = 6683;
  // image_cluster_t.edges[std::make_pair(3409, 3410)] = 4642;
  // image_cluster_t.edges[std::make_pair(1729, 1730)] = 3530;
  // image_clustering_ = std::unique_ptr<ImageClustering>(
  //     new ImageClustering(metric_cluster_options, image_cluster_t));
  // // image_clustering_->Cut();
  //         std::unordered_map<int, int> labels_t;
  //       int subcluster_num_t = 0;
  // image_clustering_->CutByMetric(labels_t,subcluster_num_t);
  
  // exit(EXIT_SUCCESS);

  // for(int i=0; i<image_pairs.size(); i++)
  // {
  //   LOG(INFO)<<image_pairs[i].first<<","<<image_pairs[i].second<<","<<weights[i];
  // }


  const std::unique_ptr<Cluster> cluster = this->CreateCluster();
  CHECK_NOTNULL(cluster.get());
  // LOG(INFO) << "cluster num: " << num_clusters;
  cluster->InitIGraph(image_pairs, weights);
  // labels = cluster->ComputeCluster(image_ids, image_pairs, weights);
  labels = cluster->ComputeCluster(image_pairs, weights, num_clusters);
  // if (options_.is_output_igraph) {
    // LOG(INFO) << "Output igraph to " << options_.graph_dir;
    // CreateDirIfNotExists(options_.graph_dir);
    // cluster->OutputIGraph(options_.graph_dir);
    // cluster->MyOutputClusterBeforeExpand(options_.graph_dir,labels); // mycode
  // }
  cluster_num = num_clusters; //cluster->ClusterNum();
  LOG(INFO) << "Cutting Complete";
  // LOG(INFO) << "Cutting Complete, grouping images...";

  // if(root_cluster_.image_ids.size()==7)
  // {
  //   for(const auto label:labels)
  //   {
  //     LOG(INFO)<<label.first<<" label:"<<label.second;
  //   }
  // }

  // exit(EXIT_SUCCESS);

  // // Collect nodes according to the partition result
  // intra_clusters_.resize(cluster->ClusterNum());
  // for (const auto label : labels) {
  //   intra_clusters_[label.second].image_ids.push_back(label.first);
  // }
  // for (uint i = 0; i < intra_clusters_.size(); i++) {
  //   intra_clusters_[i].cluster_id = i;
  // }

  // // Grouping edges
  // LOG(INFO) << "Grouping edges...";
  // for (size_t k = 0; k < image_pairs.size(); k++) {
  //   const image_t i = image_pairs[k].first;
  //   const image_t j = image_pairs[k].second;
  //   const int cluster_id1 = labels[i];
  //   const int cluster_id2 = labels[j];

  //   if (cluster_id1 == cluster_id2) {
  //     intra_clusters_[cluster_id1].edges.insert(
  //         std::make_pair(ImagePair(i, j), weights[k]));
  //   } else {
  //     const ImagePair view_pair = cluster_id1 < cluster_id2
  //                                     ? ImagePair(cluster_id1, cluster_id2)
  //                                     : ImagePair(cluster_id2, cluster_id1);
  //     clusters_lost_edges_[view_pair].push_back(graph::Edge(i, j, weights[k]));
  //   }
  // }

  timer_.Pause();
  summary_.total_cutting_num = 1;
  summary_.total_cutting_time = timer_.ElapsedSeconds();
}


void ImageClustering::FilterImageEdges(std::unordered_map<size_t, float>& self_similarity_scores, 
                                      std::unordered_map<ImagePair, float>& cross_similarity_ratios)
{
  // std::unordered_map<ImagePair,float> cluster_edges;
  // std::unordered_map<size_t, std::pair<size_t, float>> cluster_edges;
  
  std::unordered_map<size_t, int> self_max_matches;
  std::unordered_map<ImagePair, float> cross_similarity_scores;

  // LOG(INFO) << "Filter image edges...";
  // for(int i=0;i<intra_clusters_.size();i++)
  // {
  //   ImageCluster cluster1 = intra_clusters_[i];
  //   int max_matches = 0;
  //   float self_similarity_score = cluster1.ComputeSelfSimilarityScore(max_matches);
  //   self_similarity_scores.insert(std::make_pair(cluster1.cluster_id, self_similarity_score));
  //   self_max_matches.insert(std::make_pair(cluster1.cluster_id, max_matches));
    
  //   for(int j=i+1 ; j<intra_clusters_.size(); j++)
  //   {
  //     ImageCluster cluster2 = intra_clusters_[j];
  //     ImagePair cluster_pair = cluster1.cluster_id < cluster2.cluster_id 
  //                                 ? ImagePair(cluster1.cluster_id, cluster2.cluster_id)
  //                                 : ImagePair(cluster2.cluster_id, cluster1.cluster_id);
  //     std::vector<graph::Edge> edges = subclusters_all_edges_[cluster_pair];
  //     if(edges.empty())
  //     {
  //       // cluster_edges.insert(std::make_pair(cluster_pair.first,std::make_pair(cluster_pair.second,0.0)));
  //       // cluster_edges.insert(std::make_pair(cluster_pair.second,std::make_pair(cluster_pair.first,0.0)));
  //       continue;
  //     }
  //     size_t sum_weight = 0;
  //     // if(cluster1.cluster_id==0&&cluster2.cluster_id==40)
  //     // {
  //     //   LOG(INFO)<<"edges size: "<<edges.size();
  //     // }
  //     for(auto edge : edges)
  //     {

  //       size_t weight = edge.weight;
  //       ////my change in v13
  //       // if(weight<20) //跳过匹配数过小的edge，因为不可靠
  //       //   continue;
  //       sum_weight += weight;  

  //       if((cluster1.cluster_id==1&&cluster2.cluster_id==3) ||(cluster1.cluster_id==2&&cluster2.cluster_id==5))
  //       // if(cluster1.cluster_id==1306&&cluster2.cluster_id==1511)
  //       {
  //         size_t img1 = edge.src;
  //         size_t img2 = edge.dst;
  //         LOG(INFO)<<"attention here: ["<<cluster1.cluster_id<<","<<cluster2.cluster_id<<"] ("<<img1<<","<<img2<<") "<<weight; 
  //       }        
  //     }
  //     // if(cluster1.cluster_id==0&&cluster2.cluster_id==40)
  //     // {
  //     //   LOG(INFO)<<"sum weight:"<<sum_weight;
  //     // }
  //     // LOG(INFO)<<"size1 "<<cluster1.image_ids.size()<<" size2 "<<cluster2.image_ids.size();
  //     // float cross_similarity_score = float(sum_weight)/sqrt((float)(cluster1.image_ids.size()*cluster2.image_ids.size()));
  //     float cross_similarity_score = float(sum_weight)/(float)(cluster1.image_ids.size()*cluster2.image_ids.size());
  //     // LOG(INFO)<<"cross score: "<<cross_similarity_score;
  //     cross_similarity_scores.insert(std::make_pair(cluster_pair,cross_similarity_score));
  //   }
  // }

  LOG(INFO) << "Filter image edges...";
  for(int i=0;i<intra_clusters_.size();i++)
  {
    ImageCluster cluster1 = intra_clusters_[i];
    int max_matches = 0;
    float self_similarity_score = cluster1.ComputeSelfSimilarityScore(max_matches);
    self_similarity_scores.insert(std::make_pair(cluster1.cluster_id, self_similarity_score));
    self_max_matches.insert(std::make_pair(cluster1.cluster_id, max_matches));
  }


  for(int i=0;i<intra_clusters_.size();i++)
  {  
    ImageCluster cluster1 = intra_clusters_[i];
    for(int j=i+1 ; j<intra_clusters_.size(); j++)
    {
      ImageCluster cluster2 = intra_clusters_[j];
      ImagePair cluster_pair = cluster1.cluster_id < cluster2.cluster_id 
                                  ? ImagePair(cluster1.cluster_id, cluster2.cluster_id)
                                  : ImagePair(cluster2.cluster_id, cluster1.cluster_id);
      std::vector<graph::Edge> edges = subclusters_all_edges_[cluster_pair];
      if(edges.empty())
      {
        // cluster_edges.insert(std::make_pair(cluster_pair.first,std::make_pair(cluster_pair.second,0.0)));
        // cluster_edges.insert(std::make_pair(cluster_pair.second,std::make_pair(cluster_pair.first,0.0)));
        continue;
      }
      float sum_weight = 0;
      float mean_max_matches = 0.5f * (float)(self_max_matches[cluster1.cluster_id] + self_max_matches[cluster2.cluster_id]); //平均最大匹配数
      // float smaller_max_matches = std::min(self_max_matches[cluster1.cluster_id], self_max_matches[cluster2.cluster_id]); 
      
      // if(cluster1.cluster_id==0&&cluster2.cluster_id==40)
      // {
      //   LOG(INFO)<<"edges size: "<<edges.size();
      // }

      size_t max_weight = 0;
      image_t src = 0, dst = 0;
      for(auto edge : edges)
      {

        size_t weight = edge.weight;
        
        // CHECK_NE(weight,0);
        if(weight>max_weight)
        {
          max_weight = weight;
          src = edge.src;
          dst = edge.dst;
        }
          

        // ////my change in v13
        // // if(weight<20) //跳过匹配数过小的edge，因为不可靠
        // //   continue;
        // sum_weight += (float)weight/(float)mean_max_matches;  

        // if((cluster1.cluster_id==1&&cluster2.cluster_id==3) ||(cluster1.cluster_id==2&&cluster2.cluster_id==5))
        // // if(cluster1.cluster_id==1306&&cluster2.cluster_id==1511)
        // {
        //   size_t img1 = edge.src;
        //   size_t img2 = edge.dst;
        //   LOG(INFO)<<"attention here: ["<<cluster1.cluster_id<<","<<cluster2.cluster_id<<"] ("<<img1<<","<<img2<<") "<<weight; 
        // }        
      }
      // CHECK_EQ(max_weight,0);
      // if(cluster1.cluster_id==0&&cluster2.cluster_id==40)
      // {
      //   LOG(INFO)<<"sum weight:"<<sum_weight;
      // }
      // LOG(INFO)<<"size1 "<<cluster1.image_ids.size()<<" size2 "<<cluster2.image_ids.size();
      // float cross_similarity_score = float(sum_weight)/sqrt((float)(cluster1.image_ids.size()*cluster2.image_ids.size()));
      // float cross_similarity_score = float(sum_weight)/(float)(cluster1.image_ids.size()*cluster2.image_ids.size());
      float cross_similarity_score = float(max_weight)/(float)mean_max_matches; //only use the best match to evaluate edges
      // float cross_similarity_score = float(max_weight)/(float)smaller_max_matches; //only use the best match to evaluate edges
      
      LOG(INFO)<<"max_weight: "<<max_weight
              <<" between image pair "<<src<<","<< dst
              <<" of subcluster pair "<<cluster1.cluster_id<<","<<cluster2.cluster_id;
      cross_similarity_scores.insert(std::make_pair(cluster_pair,cross_similarity_score));
    }
  }

  //show scores
  for(int i=0;i<intra_clusters_.size();i++)
  {
    ImageCluster cluster1 = intra_clusters_[i];    
    for(int j=i+1 ; j<intra_clusters_.size(); j++)
    {
      ImageCluster cluster2 = intra_clusters_[j];
      ImagePair cluster_pair = cluster1.cluster_id < cluster2.cluster_id 
                                  ? ImagePair(cluster1.cluster_id, cluster2.cluster_id)
                                  : ImagePair(cluster2.cluster_id, cluster1.cluster_id);
      if(cross_similarity_scores.count(cluster_pair)!=0)
      {
        LOG(INFO)<<"cluster pair: ("<<cluster1.cluster_id<<", "<<cluster2.cluster_id
              <<"), cluster size: ("<<cluster1.image_ids.size()<<", "<<cluster2.image_ids.size()
              <<"), self score: ("<<self_similarity_scores[cluster1.cluster_id]<<", "<<self_similarity_scores[cluster2.cluster_id]
              <<"), max matches: ("<<self_max_matches[cluster1.cluster_id]<<", "<<self_max_matches[cluster2.cluster_id]
              <<"), cross score: "<<cross_similarity_scores[cluster_pair];
      }
    }
  }

  cross_similarity_ratios = cross_similarity_scores;

//   //filter lost edges to strong edges
//   std::unordered_map<ImagePair, std::vector<graph::Edge>> clusters_strong_edges;
//   for(int i=0;i<intra_clusters_.size();i++)
//   {
//     ImageCluster cluster1 = intra_clusters_[i];    
//     for(int j=i+1 ; j<intra_clusters_.size(); j++)
//     {
//       ImageCluster cluster2 = intra_clusters_[j];
//       ImagePair cluster_pair = cluster1.cluster_id < cluster2.cluster_id 
//                                   ? ImagePair(cluster1.cluster_id, cluster2.cluster_id)
//                                   : ImagePair(cluster2.cluster_id, cluster1.cluster_id);
//       if(cross_similarity_scores[cluster_pair]==0)
//         continue;
//       float mean_self_score = (float)(self_similarity_scores[cluster1.cluster_id] + self_similarity_scores[cluster2.cluster_id]) / 2.0f;
//       float ratio = (float)cross_similarity_scores[cluster_pair] / mean_self_score;
//       cross_similarity_ratios.insert(std::make_pair(cluster_pair,ratio));
//       LOG(INFO)<<cluster_pair.first<<","<<cluster_pair.second<<","<<ratio;
// // #define test
// // #ifdef test
//       if(ratio>=0.02)//strong edge
//       {
//         clusters_strong_edges.insert(std::make_pair(cluster_pair,subclusters_all_edges_[cluster_pair]));
//       }
// // #endif
//     }
//   }


//   subclusters_strong_edges_ = clusters_strong_edges;//save strong edges

//   LOG(INFO)<<"subclusters_strong_edges_ size: "<<subclusters_strong_edges_.size()<<std::endl;
  LOG(INFO)<<"subclusters_all_edges_ size: "<<subclusters_all_edges_.size()<<std::endl;




}



float ImageCluster::ComputeSelfSimilarityScore(int& max_matches)
{
  size_t img_size = image_ids.size();
  size_t potential_pairs_size = img_size * (img_size-1) /2; //排列组合 C2,n


  if(img_size == 1)
  {
    return 0.0f;//subcluster内仅有一张影像
  }
  else
  {
    int max_weight = 0;
    int sum_weight = 0;
    for(auto edge:edges)
    {
      int weight = edge.second;
      sum_weight += weight;
      if(weight>max_weight)
        max_weight = weight;
    }
    max_matches = max_weight; //记录类内匹配最大值
    return (float)sum_weight/(float)potential_pairs_size; //记录类内匹配平均数
    // return (float)sum_weight/sqrt((float)potential_pairs_size);
  }
}

void ImageClustering::GroupingImageEdges(std::vector<std::vector<image_t>> subclusters_imgs, std::unordered_map<int, int> labels)
{
  int cluster_num = subclusters_imgs.size();
  // timer_.Start();
  LOG(INFO) << "Grouping images...";
  // Collect nodes according to the partition result
  LOG(INFO)<<"cluster num: "<<cluster_num;
  // LOG(INFO)<<"labels size: "<<labels.size();
  intra_clusters_.resize(cluster_num);

  for (int i=0; i<cluster_num; i++)
  {
    intra_clusters_[i].image_ids = subclusters_imgs[i];
    intra_clusters_[i].cluster_id = i;
  }




  // for (const auto label : labels) {
  //   // LOG(INFO)<<"label.second: "<<label.second<<" label.first"<<label.first;
  //   intra_clusters_[label.second].image_ids.push_back(label.first);
  // }
  // for (uint i = 0; i < intra_clusters_.size(); i++) {
  //   intra_clusters_[i].cluster_id = i;
  // }

  // Grouping edges
  LOG(INFO) << "Grouping edges...";
  std::vector<std::pair<int, int>> image_pairs;
  std::vector<int> weights;
  image_pairs.reserve(root_cluster_.edges.size());
  weights.reserve(root_cluster_.edges.size());

  for (auto edge : root_cluster_.edges) {
    image_pairs.push_back(std::make_pair(edge.first.first, edge.first.second));
    weights.push_back(edge.second);
  }

  for (size_t k = 0; k < image_pairs.size(); k++) {
    const image_t i = image_pairs[k].first;
    const image_t j = image_pairs[k].second;
    const int cluster_id1 = labels[i];
    const int cluster_id2 = labels[j];

    if (cluster_id1 == cluster_id2) {
      intra_clusters_[cluster_id1].edges.insert(
          std::make_pair(ImagePair(i, j), weights[k]));
    } else {
      const ImagePair view_pair = cluster_id1 < cluster_id2
                                      ? ImagePair(cluster_id1, cluster_id2)
                                      : ImagePair(cluster_id2, cluster_id1);
      // if(view_pair.first==0 && view_pair.second==633)
      // {
      //   LOG(INFO)<<"i: "<<i<<" j: "<<j<<" labels[i]: "<<labels[i]<<" labels[j]: "<<labels[j];
      // }
      
      subclusters_all_edges_[view_pair].push_back(graph::Edge(i, j, weights[k]));
    }
  }

  // ImagePair pair_test = std::make_pair(0,633);
  // std::vector<graph::Edge> edges_test = subclusters_all_edges_[pair_test];
  // LOG(INFO)<<"cluster pair: 0,633";
  // for (int i=0;i<edges_test.size(); i++)
  // {
  //   image_t img1 = edges_test[i].src;
  //   image_t img2 = edges_test[i].dst;
  //   int weight = edges_test[i].weight;
  //   LOG(INFO)<<"("<<img1<<","<<img2<<") "<<weight; 
  // }

  LOG(INFO) << "Grouping finished...";
}


std::unordered_map<ImagePair, std::vector<graph::Edge>> ImageClustering::GetAllInterClusterEdges(){
  return subclusters_all_edges_;
}


std::unordered_map<int, Edges> ImageClustering::GetBestEdges(std::vector<ImageCluster>& subclusters,  
            std::unordered_map<ImagePair, std::vector<graph::Edge>>& inter_subcluster_edges, 
            const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs, 
            const double max_relative_rotation_difference_degrees)
{
  std::unordered_map<int, Edges> all_best_edges;

  // // for(int i=0; i<inter_clusters_.size(); i++)
  // // {
  // //   LOG(INFO)<<i<<","<<inter_clusters_[i].cluster_id;
  // // }

  // for(int i=0; i<inter_clusters_.size(); i++)
  // {
  //   ImageCluster big_cluster = inter_clusters_[i];
  //   std::vector<std::pair<ImagePair, int>> sorted_edges; //将subcluster间的edges按大小进行排序
  //   for(auto edge:big_cluster.edges)
  //   {
  //     sorted_edges.push_back(std::make_pair(edge.first, edge.second));
  //   }
  //   auto const edges_cmp = [](std::pair<ImagePair, int> const edge1, std::pair<ImagePair, int> const edge2){
  //     return edge1.second > edge2.second;
  //   };
  //   sort(sorted_edges.begin(), sorted_edges.end(), edges_cmp);

  //   LOG(INFO)<< "Performing RA inside subclusters...";
  //   std::unordered_map<image_t, std::unordered_map<image_t, Eigen::Vector3d>> subclusters_rotations;//cluster内有几个subcluster就有几个map
  //   std::unordered_map<image_t, std::unordered_map<ImagePair, TwoViewInfo>> subclusters_view_pairs;
  //   for(int j=0; j<big_cluster.image_ids.size(); j++) //遍历每个subcluster，做global rotations
  //   {
  //     image_t subcluster_id = big_cluster.image_ids[j];
  //     LOG(INFO)<<"RA in subcluster "<<subcluster_id;
  //     ImageCluster subcluster = subclusters[subcluster_id]; //当前subcluster
  //     const auto image_ids = subcluster.image_ids;
  //     const auto edges = subcluster.edges;
  //     std::unordered_map<image_t, Eigen::Vector3d> rotations;

  //     for (auto image_id : image_ids) {
  //       rotations[image_id] = Eigen::Vector3d::Zero();
  //     }
  //     std::unordered_map<ImagePair, TwoViewInfo> subcluster_view_pairs;
  //     for(auto edge : edges)
  //     {
  //       subcluster_view_pairs.insert(std::make_pair(edge.first,view_pairs.at(edge.first)));
  //     }

  //     // Choose the global rotation estimation type.
  //     std::unique_ptr<RotationEstimator> rotation_estimator;
  //     RobustRotationEstimator::Options robust_rotation_estimator_options;
  //     rotation_estimator.reset(
  //         new RobustRotationEstimator(robust_rotation_estimator_options));

  //     bool success = rotation_estimator->EstimateRotations(subcluster_view_pairs, &rotations);
  //     subclusters_rotations.insert(std::make_pair(subcluster_id, rotations)); //保存每个subcluster内部的rotation
  //     subclusters_view_pairs.insert(std::make_pair(subcluster_id, subcluster_view_pairs)); //保存每个subcluster内部的edges

  //     if (!success) {
  //       LOG(INFO)<<"rotation averaging failed in subcluster :"<<subcluster_id;
  //       exit(EXIT_FAILURE);
  //     }

  //     // for(auto it: rotations)
  //     // {
  //     //   LOG(INFO)<<"img id: "<<it.first<<" r:"<<it.second[0]<<","<<it.second[1]<<","<<it.second[2];
  //     // }
  //   }

  //   LOG(INFO)<<"Performing RA in combined clusters...";
  //   for(auto sorted_edge : sorted_edges)
  //   {
  //     ImagePair subcluster_pair = sorted_edge.first;        
  //     LOG(INFO)<<"RA in merged subcluster pair: "<<subcluster_pair.first<<", "<<subcluster_pair.second;
  //     image_t subcluster_id1 = subcluster_pair.first;
  //     image_t subcluster_id2 = subcluster_pair.second;
  //     std::unordered_map<image_t, Eigen::Vector3d> rotations;
  //     for (auto image_id : subclusters[subcluster_id1].image_ids) {
  //       rotations[image_id] = Eigen::Vector3d::Zero();
  //     }
  //     for (auto image_id : subclusters[subcluster_id2].image_ids) {
  //       rotations[image_id] = Eigen::Vector3d::Zero();
  //     }
  //     std::unordered_map<ImagePair, TwoViewInfo> merged_view_pairs;
  //     //合并两个subcluster内部的edges
  //     merged_view_pairs.insert(subclusters_view_pairs[subcluster_id1].begin(),subclusters_view_pairs[subcluster_id1].end());
  //     merged_view_pairs.insert(subclusters_view_pairs[subcluster_id2].begin(),subclusters_view_pairs[subcluster_id2].end());
  //     //以及两个subcluster之间的edges

  //     for(auto itr : inter_subcluster_edges[subcluster_pair]) //遍历subcluster之间的匹配
  //     {          
  //       // LOG(INFO)<<"subcluster pair: "<<subcluster_pair.first<<", "<<subcluster_pair.second;
  //       // std::vector<graph::Edge> edges = itr;
  //       ImagePair pair = std::make_pair(itr.src,itr.dst);
  //       merged_view_pairs.insert(std::make_pair(pair, view_pairs.at(pair)));
  //     }
  //     // Choose the global rotation estimation type.
  //     std::unique_ptr<RotationEstimator> rotation_estimator;
  //     RobustRotationEstimator::Options robust_rotation_estimator_options;
  //     rotation_estimator.reset(
  //         new RobustRotationEstimator(robust_rotation_estimator_options));
      
  //     bool success = rotation_estimator->EstimateRotations(merged_view_pairs, &rotations);

  //     if (!success) {
  //       LOG(INFO)<<"rotation averaging failed in merged subcluster "<<subcluster_id1<<" + "<<subcluster_id2;
  //       exit(EXIT_FAILURE);
  //     }

  //     for(auto it: rotations)
  //     {
  //       LOG(INFO)<<"img id: "<<it.first<<" r:"<<it.second[0]<<","<<it.second[1]<<","<<it.second[2];
  //     }

  //     //将两个subcluster合并前后的rotation进行过滤，去除有问题的edge
  //     FilterSubclusterEdgesFromOrientation(subclusters_rotations[subcluster_id1], subclusters_rotations[subcluster_id2], rotations,
  //                                           subclusters_view_pairs[subcluster_id1], subclusters_view_pairs[subcluster_id2],
  //                                           max_relative_rotation_difference_degrees);
  //   }

  // }
  // exit(EXIT_SUCCESS);



  for(int i=0; i<inter_clusters_.size(); i++)
  {
    if(inter_clusters_[i].image_ids.size()==1) //有可能存在独立的一个subcluster，与周围其他subcluster关联非常小
    {
      all_best_edges.insert(std::make_pair(inter_clusters_[i].cluster_id, inter_clusters_[i].edges));
      continue;
    }

    //step1: 构建每个cluster内部以subcluster为节点，cross ratio为edge的mst
    //////////////////////////////////////////////////////////////////////

    ///////////////////1.1 构建完整树/////////////////////
    LOG(INFO)<<"inter_clusters_[i].image_ids.size(): "<<inter_clusters_[i].image_ids.size();
    graph::Graph<graph::Node, graph::Edge> inter_cluster_graph;
    //1.Add nodes
    for(int j=0; j<inter_clusters_[i].image_ids.size(); j++)
    {
      image_t img_id = inter_clusters_[i].image_ids[j];
      graph::Node node(img_id);
      inter_cluster_graph.AddNode(node);
    }
    //2.Add edges
    LOG(INFO)<<"inter_clusters_["<<i<<"].edges size: "<<inter_clusters_[i].edges.size();
    for(auto edge:inter_clusters_[i].edges)
    {
      ImagePair pair = edge.first;
      int weight = edge.second;
      inter_cluster_graph.AddEdge(graph::Edge(pair.first,pair.second,(float)weight));
      LOG(INFO)<<"first: "<<pair.first<<" second: "<<pair.second<<" weight: "<<weight;
    }

    //////////////////////// Output full connection to igraph /////////////////////
    std::vector<std::pair<int, int>> image_pairs_full;
    std::vector<int> weights_full;
    for(auto full_edge : inter_clusters_[i].edges) {
      image_pairs_full.push_back(std::make_pair(full_edge.first.first,full_edge.first.second));
      weights_full.push_back(full_edge.second); 
    }
    const std::unique_ptr<Cluster> full_cluster = this->CreateCluster();
    CHECK_NOTNULL(full_cluster.get());
    std::string full_dst_path = options_.graph_dir + "/igraph_intercluster/";
    CreateDirIfNotExists(full_dst_path);
    full_dst_path += std::to_string(i)+"/";
    LOG(INFO) << "Output igraph to " << full_dst_path;
    CreateDirIfNotExists(full_dst_path);

    full_cluster->OutputIGraphInfo(image_pairs_full, weights_full, full_dst_path);
    full_cluster->OutputGraphJson(image_pairs_full, weights_full, full_dst_path);
    // cluster->OutputIGraph(dst_path);


    ///////////////////1.2 subcluster间的最大连通域/////////////////////
    const graph::Graph<graph::Node, graph::Edge> largest_cc = inter_cluster_graph.ExtractLargestCC();
    LOG(INFO)<<"nodes after lcc: "<<largest_cc.GetNodesNum();
    //////////////////////// Output lcc connection to igraph /////////////////////
    std::vector<std::pair<int, int>> image_pairs_cc;
    std::vector<int> weights_cc;
    std::unordered_map<size_t, std::unordered_map<size_t, graph::Edge>> cc_edges = largest_cc.GetEdges();
    
    for(auto cc_edge : cc_edges) {
      for(auto edge : cc_edge.second) {
        image_pairs_cc.push_back(std::make_pair(edge.second.src,edge.second.dst));
        weights_cc.push_back(edge.second.weight);        
      }
    }
    const std::unique_ptr<Cluster> cc_cluster = this->CreateCluster();
    CHECK_NOTNULL(cc_cluster.get());
    // cluster->InitIGraphWithID(image_pairs_mst, weights_mst);
    // std::unordered_map<int, int> labels =
    //     cluster->ComputeCluster(image_pairs_mst, weights_mst, 2);
        
    std::string cc_dst_path = options_.graph_dir + "/igraph_cc/";
    CreateDirIfNotExists(cc_dst_path);
    cc_dst_path += std::to_string(i)+"/";
    LOG(INFO) << "Output cc igraph to " << cc_dst_path;
    CreateDirIfNotExists(cc_dst_path);

    cc_cluster->OutputIGraphInfo(image_pairs_cc, weights_cc, cc_dst_path);
    cc_cluster->OutputGraphJson(image_pairs_cc, weights_cc, cc_dst_path);
    // cluster->OutputIGraph(dst_path);


    std::vector<size_t> vec_largest_cc_nodes;
    vec_largest_cc_nodes.reserve(largest_cc.GetNodes().size());
    for (auto node_it : largest_cc.GetNodes()) {
      vec_largest_cc_nodes.push_back(node_it.first);
    }
    LOG(INFO)<<"left "<<inter_clusters_[i].image_ids.size() - vec_largest_cc_nodes.size()<<" unconnected subclusters";
    // CHECK_EQ(inter_clusters_[i].image_ids.size(),vec_largest_cc_nodes.size());//cluster内部的subclusters应该是连通的才对

    ///////////////////1.3 lcc到mst/////////////////////
    LOG(INFO) << "Finding Maximum Spanning Tree...";
    std::vector<graph::Edge> mst_edges = largest_cc.MaxstKruskal();
    if (mst_edges.size() < largest_cc.GetNodesNum() - 1) {
      LOG(WARNING) << "Invalid MST";
      return all_best_edges;
    }


    //change std::vector<graph::Edge> to std::set<ImagePair>
    std::unordered_set<ImagePair> maxst_subclusters;//用于保存认为可靠的edges
    for(auto mst_edge : mst_edges)
    {
      ImagePair pair = std::make_pair(mst_edge.src,mst_edge.dst);
      LOG(INFO)<<"mst_edge src: "<<mst_edge.src<<", mst_edge dst: "<<mst_edge.dst
                <<" with weight: "<<inter_clusters_[i].edges[pair];
      CHECK_LT(mst_edge.src,mst_edge.dst);
      maxst_subclusters.insert(pair); //添加mst中的edges
    }

    //////////////////////// Output mst connection to igraph /////////////////////
    std::vector<std::pair<int, int>> image_pairs_mst;
    std::vector<int> weights_mst;
    for(auto mst_edge : mst_edges) {
      image_pairs_mst.push_back(std::make_pair(mst_edge.src,mst_edge.dst));
      weights_mst.push_back(inter_clusters_[i].edges[std::make_pair(mst_edge.src,mst_edge.dst)]);
    }

    const std::unique_ptr<Cluster> cluster = this->CreateCluster();
    CHECK_NOTNULL(cluster.get());
    // cluster->InitIGraphWithID(image_pairs_mst, weights_mst);
    // std::unordered_map<int, int> labels =
    //     cluster->ComputeCluster(image_pairs_mst, weights_mst, 2);
        
    std::string mst_dst_path = options_.graph_dir + "/igraph_mst/";
    CreateDirIfNotExists(mst_dst_path);
    mst_dst_path += std::to_string(i)+"/";
    LOG(INFO) << "Output igraph to " << mst_dst_path;
    CreateDirIfNotExists(mst_dst_path);

    cluster->OutputIGraphInfo(image_pairs_mst, weights_mst, mst_dst_path);
    cluster->OutputGraphJson(image_pairs_mst, weights_mst, mst_dst_path);
    // cluster->OutputIGraph(dst_path);




   
   
    // std::vector<std::pair<ImagePair, int>> sorted_edges; //将subcluster间的edges按大小进行排序
    // for(auto edge:inter_clusters_[i].edges)
    // {
    //   sorted_edges.push_back(std::make_pair(edge.first, edge.second));
    // }
    // auto const edges_cmp = [](std::pair<ImagePair, int> const edge1, std::pair<ImagePair, int> const edge2){
    //   return edge1.second > edge2.second;
    // };
    // sort(sorted_edges.begin(), sorted_edges.end(), edges_cmp);

    // LOG(INFO)<< "Performing RA inside mst...";
    // // std::unordered_map<image_t, Eigen::Vector3d> mst_rotations;//cluster内有几个subcluster就有几个map
    // // std::unordered_map<ImagePair, TwoViewInfo> mst_view_pairs;
    // // for(auto itr:maxst_subclusters)

    // image_t subcluster_id = inter_clusters_[i].image_ids[j];
    // LOG(INFO)<<"RA in subcluster "<<subcluster_id;
    // ImageCluster subcluster = subclusters[subcluster_id]; //当前subcluster
    // const auto image_ids = subcluster.image_ids;
    // const auto edges = subcluster.edges;
    // std::unordered_map<image_t, Eigen::Vector3d> rotations;

    // for (auto image_id : image_ids) {
    //   rotations[image_id] = Eigen::Vector3d::Zero();
    // }
    // std::unordered_map<ImagePair, TwoViewInfo> subcluster_view_pairs;
    // for(auto edge : edges)
    // {
    //   subcluster_view_pairs.insert(std::make_pair(edge.first,view_pairs.at(edge.first)));
    // }

    // // Choose the global rotation estimation type.
    // std::unique_ptr<RotationEstimator> rotation_estimator;
    // RobustRotationEstimator::Options robust_rotation_estimator_options;
    // rotation_estimator.reset(
    //     new RobustRotationEstimator(robust_rotation_estimator_options));

    // bool success = rotation_estimator->EstimateRotations(subcluster_view_pairs, &rotations);
    // subclusters_rotations.insert(std::make_pair(subcluster_id, rotations)); //保存每个subcluster内部的rotation
    // subclusters_view_pairs.insert(std::make_pair(subcluster_id, subcluster_view_pairs)); //保存每个subcluster内部的edges



    //step2: 基于获得的mst筛选每个cluster内的best edges
    //////////////////////////////////////////////////////
    Edges best_edges; //保存当前cluster内部的可靠edges
    std::unordered_map<image_t, Eigen::Vector3d> mst_rotations;//保存所有mst中涉及到的影像rotations
    std::unordered_map<ImagePair, TwoViewInfo> mst_view_pairs;//mst中的view_pairs
    //1.先push subcluster内部的影像edge
    for(int j=0; j<inter_clusters_[i].image_ids.size(); j++)
    {
      image_t subcluster_id = inter_clusters_[i].image_ids[j];
      CHECK_GT(subclusters.size(), subcluster_id);
      ImageCluster subcluster = subclusters[subcluster_id]; 
      CHECK_EQ(subcluster.cluster_id, subcluster_id);
      best_edges.insert(subcluster.edges.begin(),subcluster.edges.end()); //subcluster内部的edges一定可靠
      
      //旋转初始化为0
      for(auto img_id : subcluster.image_ids)
      {
        mst_rotations[img_id] = Eigen::Vector3d::Zero();
      }

      for(auto edge : subcluster.edges)
      {
        // CHECK_EQ(view_pairs.count(edge.first), 1);
        if(mst_rotations.count(edge.first.first)!=0 && mst_rotations.count(edge.first.second)!=0)
          mst_view_pairs.insert(std::make_pair(edge.first, view_pairs.at(edge.first))); //把subcluster内部的view_pairs全部加进来
      }

      // if(i==0&&j==0)
      // {
      //   for(auto itr:subcluster.edges)
      //     LOG(INFO)<<"subcluster "<<subcluster_id<<" src: "<<itr.first.first<<" dst: "<<itr.first.second<<" weight: "<<itr.second;
      // }

      CHECK_EQ(subcluster_id,subcluster.cluster_id);
    }

#define EXPAND_MST
#ifdef EXPAND_MST

    //2.再push mst中的view_pairs
    for(auto itr : maxst_subclusters)
    {
      ImagePair subcluster_pair = itr;
      CHECK_EQ(inter_subcluster_edges.count(subcluster_pair), 1);
      for(auto img_edge : inter_subcluster_edges[subcluster_pair])
      {        
        ImagePair img_pair = std::make_pair(img_edge.src, img_edge.dst);
        if(mst_rotations.count(img_pair.first)!=0 && mst_rotations.count(img_pair.second)!=0)
          mst_view_pairs.insert(std::make_pair(img_pair, view_pairs.at(img_pair))); //把subcluster之间mst的view_pairs全部加进来
      }
    }


    std::unordered_map<image_t, Eigen::Vector3d> empty_rotations = mst_rotations;
    // Choose the global rotation estimation type.
    std::unique_ptr<RotationEstimator> rotation_estimator;
    RobustRotationEstimator::Options robust_rotation_estimator_options;
    rotation_estimator.reset(
        new RobustRotationEstimator(robust_rotation_estimator_options));
    


    //3.在mst内部执行一次RA

    LOG(INFO)<<"mst rotations size: "<<mst_rotations.size();
    LOG(INFO)<<"mst_view_pairs size: "<<mst_view_pairs.size();
    bool success = rotation_estimator->EstimateRotations(mst_view_pairs, &mst_rotations);
    
    // for(auto itr: mst_view_pairs)
    // {
    //   LOG(INFO)<<"two view info: "<<itr.second.rotation_2;
    // }
    // for(auto itr: mst_rotations)
    // {
    //   LOG(INFO)<<"mst rotations img id: "<<itr.first<<" rotation: "<<itr.second[0];
    // }
    LOG(INFO)<<"mst RA finished...";

    ImageCluster big_cluster = inter_clusters_[i]; //当前cluster

    //4. 记录当前cluster内每个subcluster已有的rotations，用于后面对比
    LOG(INFO)<<"recording subclusters rotations...";
    std::unordered_map<image_t, std::unordered_map<image_t, Eigen::Vector3d>> subclusters_rotations;//cluster内有几个subcluster就有几个map
    std::unordered_map<image_t, std::unordered_map<ImagePair, TwoViewInfo>> subclusters_view_pairs;
    for(auto subcluster_id : big_cluster.image_ids)
    {
      ImageCluster subcluster = subclusters[subcluster_id];
      std::unordered_map<image_t, Eigen::Vector3d> subcluster_rotations;
      for(auto imgid : subcluster.image_ids)
      {
        // LOG(INFO)<<"img id: "<<imgid;
        subcluster_rotations.insert(std::make_pair(imgid, mst_rotations.at(imgid)));
      }
      subclusters_rotations.insert(std::make_pair(subcluster.cluster_id, subcluster_rotations));

      std::unordered_map<ImagePair, TwoViewInfo> subcluster_view_pairs;
      for(auto edge : subcluster.edges)
        subcluster_view_pairs.insert(std::make_pair(edge.first, view_pairs.at(edge.first))) ;
      subclusters_view_pairs.insert(std::make_pair(subcluster.cluster_id, subcluster_view_pairs));
    }



    
    //5. 将subcluster间edges进行排序
    LOG(INFO)<<"sorting edges...";
    std::vector<std::pair<ImagePair, int>> sorted_edges; //将subcluster间的edges按大小进行排序
    for(auto edge:big_cluster.edges)
    {
      sorted_edges.push_back(std::make_pair(edge.first, edge.second));
    }
    auto const edges_cmp = [](std::pair<ImagePair, int> const edge1, std::pair<ImagePair, int> const edge2){
      return edge1.second > edge2.second;
    };
    sort(sorted_edges.begin(), sorted_edges.end(), edges_cmp);
    
    LOG(INFO)<<"Performing RA in merging clusters...";
    std::unordered_set<ImagePair> robust_subclusters = maxst_subclusters;
    // std::unordered_map<image_t, Eigen::Vector3d> merged_rotations = mst_rotations;//cluster内有几个subcluster就有几个map
    std::unordered_map<ImagePair, TwoViewInfo> merged_view_pairs = mst_view_pairs;
    // std::unordered_map<image_t, Eigen::Vector3d> robust_rotations = mst_rotations;
    // std::unordered_map<ImagePair, TwoViewInfo> robust_view_pairs = merged_view_pairs;

    //此处应该先基于获得的mst扩展triplets，以获得回环间的数据关联！！！
    //先对mst影像RA，再验证新加进来的edge
    //step3: 按其他edges的大小顺序逐一check每一条边的可靠性，若可靠则加入best_edges中
    //存在的问题: 1）此处edges数量非常大的时候，效率非常低，文库数据2600张影像几十万条edges，太花时间了
    //           2) 在小数据上测试时发现，所有edges都通过了测试，但是最终structure仍然被拉走了
    //改进：      1）只添加比较关键的circle edges， 而非全部edges？
    //            2）利用hierarchical思想，先划分较多大组，这样大组内的edges数量会减少很多？
    ////////////////////////////////////////////////////////////////////////////
    LOG(INFO)<<"size of sorted edges: "<<sorted_edges.size();
    int edge_count = 0;

    std::vector<std::pair<ImagePair, double>> match_errors;
    for(auto sorted_edge : sorted_edges)
    {
      edge_count++;
      ImagePair subcluster_pair = sorted_edge.first;  
      if(robust_subclusters.count(subcluster_pair)!=0) //此edge本来就在mst中
        continue;
      LOG(INFO)<<100.0f*(float)edge_count/(float)sorted_edges.size()<<"% has done";      
      LOG(INFO)<<"RA when merging subcluster pair: "<<subcluster_pair.first<<", "<<subcluster_pair.second;
      image_t subcluster_id1 = subcluster_pair.first;
      image_t subcluster_id2 = subcluster_pair.second;

      // //************* Version 0: use all mst images
      // std::unordered_map<image_t, Eigen::Vector3d> merged_rotations = empty_rotations; //初始值设置为0

      //************* Version 1: use only images in two subclusters connected by this edge
      std::unordered_map<image_t, Eigen::Vector3d> merged_rotations;
      for (auto image_id : subclusters[subcluster_id1].image_ids) {
        merged_rotations[image_id] = Eigen::Vector3d::Zero();
      }
      for (auto image_id : subclusters[subcluster_id2].image_ids) {
        merged_rotations[image_id] = Eigen::Vector3d::Zero();
      }
      merged_view_pairs.clear(); //之前将这个设置为mst中的全部edges了
      //合并两个subcluster内部的edges
      merged_view_pairs.insert(subclusters_view_pairs[subcluster_id1].begin(),subclusters_view_pairs[subcluster_id1].end());
      merged_view_pairs.insert(subclusters_view_pairs[subcluster_id2].begin(),subclusters_view_pairs[subcluster_id2].end());



      //合并上两个subcluster之间的edges
      for(auto itr : inter_subcluster_edges[subcluster_pair]) //遍历subcluster之间的匹配
      {
        ImagePair pair = std::make_pair(itr.src,itr.dst);
        merged_view_pairs.insert(std::make_pair(pair, view_pairs.at(pair)));
      }
      // Choose the global rotation estimation type.
      std::unique_ptr<RotationEstimator> rotation_estimator;
      RobustRotationEstimator::Options robust_rotation_estimator_options;
      rotation_estimator.reset(
          new RobustRotationEstimator(robust_rotation_estimator_options));
      
      bool success = rotation_estimator->EstimateRotations(merged_view_pairs, &merged_rotations);

      if (!success) {
        LOG(INFO)<<"rotation averaging failed in merged subcluster "<<subcluster_id1<<" + "<<subcluster_id2;
        exit(EXIT_FAILURE);
      }

      // for(auto it: rotations)
      // {
      //   LOG(INFO)<<"img id: "<<it.first<<" r:"<<it.second[0]<<","<<it.second[1]<<","<<it.second[2];
      // }

      //将两个subcluster合并前后的rotation进行过滤，去除有问题的edge
      // FilterSubclusterEdgesFromOrientation(subclusters_rotations[subcluster_id1], subclusters_rotations[subcluster_id2], rotations,
      //                                       subclusters_view_pairs[subcluster_id1], subclusters_view_pairs[subcluster_id2],
      //                                       max_relative_rotation_difference_degrees);
      // if(FilterSubclusterEdgesFromOrientation(robust_rotations, merged_rotations, 
      //                                         robust_view_pairs, max_relative_rotation_difference_degrees))

      ////************* Version 0: use all mst rotations to check (too slow ?)
      // if(FilterSubclusterEdgesFromOrientation(mst_rotations, merged_rotations, 
      //                                         mst_view_pairs, max_relative_rotation_difference_degrees))
      
      
      //************* Version 1: use only this two subclusters to check
      //两个subcluster内的images组成的rotations
      std::unordered_map<image_t, Eigen::Vector3d> ratations_0 = 
                                          subclusters_rotations.at(subcluster_id1);
      std::unordered_map<image_t, Eigen::Vector3d> ratations_1 =
                                          subclusters_rotations.at(subcluster_id2);
      ratations_0.insert(ratations_1.begin(), ratations_1.end());
      // if(FilterSubclusterEdgesFromOrientation(ratations_0, merged_rotations, 
      //                                         merged_view_pairs, max_relative_rotation_difference_degrees))
      
      bool EdgeAccepted = FilterSubclusterEdgesFromOrientation(ratations_0, merged_rotations, 
                                              merged_view_pairs, max_relative_rotation_difference_degrees,
                                              match_errors);
      
      if(EdgeAccepted)
      {
        // robust_rotations = merged_rotations;
        // robust_view_pairs = merged_view_pairs;
        maxst_subclusters.insert(subcluster_pair); //增加到robust edges中
      }  
    }

    std::string edge_errpr_path = options_.graph_dir + "/edge_RA_error.txt";
    LOG(INFO) << "Output edge RA error to " << edge_errpr_path;
    auto const error_cmp = [](std::pair<ImagePair, double> const edge1, 
                              std::pair<ImagePair, double> const edge2){
      return edge1.second > edge2.second;
    };
    std::sort(match_errors.begin(), match_errors.end(), error_cmp);
    std::ofstream error_f(edge_errpr_path, std::ios::out);
    if(!error_f)
    {
      LOG(INFO)<<"can't open this file...";
      exit(EXIT_FAILURE);
    }
    for(int i=0; i<match_errors.size()/10; i++)
    {
      error_f << match_errors[i].first.first << "\t" 
              << match_errors[i].first.second<< "\t" 
              << match_errors[i].second<<std::endl;
    }
    error_f.close();

    // exit(EXIT_SUCCESS);


    //////////////////////// Output verified connections to igraph /////////////////////
    std::vector<std::pair<int, int>> image_pairs_verified;
    std::vector<int> weights_verified;
    for(auto subcluster_pair : maxst_subclusters) {
      image_pairs_verified.push_back(std::make_pair(subcluster_pair.first,subcluster_pair.second));
      weights_verified.push_back(inter_clusters_[i].edges[subcluster_pair]);
    }

    const std::unique_ptr<Cluster> verified_cluster = this->CreateCluster();
    CHECK_NOTNULL(verified_cluster.get());
    // cluster->InitIGraphWithID(image_pairs_mst, weights_mst);
    // std::unordered_map<int, int> labels =
    //     cluster->ComputeCluster(image_pairs_mst, weights_mst, 2);
        
    std::string verified_dst_path = options_.graph_dir + "/igraph_verified/";
    CreateDirIfNotExists(verified_dst_path);
    verified_dst_path += std::to_string(i)+"/";
    LOG(INFO) << "Output igraph to " << verified_dst_path;
    CreateDirIfNotExists(verified_dst_path);

    verified_cluster->OutputIGraphInfo(image_pairs_verified, weights_verified, verified_dst_path);
    verified_cluster->OutputGraphJson(image_pairs_verified, weights_verified, verified_dst_path);
    // cluster->OutputIGraph(dst_path);

#endif

    auto const cmp = [](graph::Edge const edge1, graph::Edge const edge2){
      return edge1.weight > edge2.weight;
    };

    //step4: 再push maxst_subclusters覆盖到的edges
    ///////////////////////////////////////////////////////////
    for(auto itr : inter_subcluster_edges)
    {
      ImagePair subcluster_pair = itr.first;
      if(maxst_subclusters.count(subcluster_pair)==0) //如果不在mst中的edge则不保留
      {
        // LOG(INFO)<<"left subcluster pair: "<<subcluster_pair.first<<", "<<subcluster_pair.second;
        continue;
      }
        
      LOG(INFO)<<"subcluster pair: "<<subcluster_pair.first<<", "<<subcluster_pair.second;
      std::vector<graph::Edge> edges = itr.second;
      sort(edges.begin(),edges.end(),cmp);
      int max_weight = edges[0].weight;
      // LOG(INFO)<<"best matches "<<max_weight;
      int weight_thresh = max_weight * 0.0;//0.1;//0.5; //可靠edge的阈值70%
      for(int j=0; j<edges.size(); j++)
      {
        if(edges[j].weight>weight_thresh)
        {
          ImagePair pair = std::make_pair(edges[j].src,edges[j].dst);
          best_edges.insert(std::make_pair(pair,edges[j].weight));
          // LOG(INFO)<<"image pair: "<<pair.first<<", "<<pair.second<<" weight:"<<edges[j].weight;
        }
      }
    }
    CHECK_EQ(inter_clusters_[i].cluster_id, i);
    all_best_edges.insert(std::make_pair(inter_clusters_[i].cluster_id, best_edges));
  }

  LOG(INFO) << "GetBestEdges finished...";

  LOG(INFO)<<all_best_edges.size();
  for(auto all_best_edge : all_best_edges)
  {
    LOG(INFO)<<"cluster "<<all_best_edge.first<<" has "<<all_best_edge.second.size()<<" best edges";
  }
  return all_best_edges;
}

void ImageClustering::CutUnconnectedClusters( const std::vector<image_t>& image_ids,
                                        const std::vector<std::pair<int, int>>& edges,
                                        std::unordered_map<size_t, std::vector<image_t>>& components) {
  // Union Find to split components
  graph::UnionFind uf(image_ids.size());
  std::vector<size_t> tmp_nodes(image_ids.begin(), image_ids.end());
  uf.InitWithNodes(tmp_nodes);

  for (auto image_pair : edges) {
      uf.Union(image_pair.first, image_pair.second);
  }

  // std::unordered_map<size_t, std::vector<image_t>> components;
  components.clear();
  for (auto image_id : image_ids) {
      const size_t parent_id = uf.FindRoot(image_id);
      components[parent_id].push_back(image_id);
  }

  LOG(INFO) << "There are " << components.size() << " connected components.";
  int cluster_num = components.size();
  size_t cluster_id = 0;
  for(auto component : components)
  {
      LOG(INFO) << "Component #" << component.first << "# has "
          << component.second.size() << " images.";
  }
}


void ImageClustering::CutSubClusters(std::unordered_map<int, int>& labels, int& cluster_num) {
  timer_.Start();
  const uint num_clusters =
      root_cluster_.image_ids.size() / options_.num_images_ub;
  if(num_clusters <= 1)//如果不需要cut，则直接返回
  {
    LOG(INFO)<<"only one cluster, return...";
    cluster_num = 1;
    return;
  }
  
  // const uint num_clusters = 2;
  CHECK_GE(num_clusters, 1);
  // cluster_num = num_clusters;
  std::vector<std::pair<int, int>> image_pairs;
  std::vector<int> weights;
  image_pairs.reserve(root_cluster_.edges.size());
  weights.reserve(root_cluster_.edges.size());

  for (auto edge : root_cluster_.edges) {
    // //notice!!! just for test here
    // if(edge.second<5)
    //   continue;
    // //end

    image_pairs.push_back(std::make_pair(edge.first.first, edge.first.second));
    weights.push_back(edge.second);

    // if(edge.first.first==1045 || edge.first.second==1045)
    // {
    //   LOG(INFO)<<"image pairs: "<<edge.first.first<<" "<<edge.first.second<<" "<<edge.second;
    // }
  }

  // LOG(INFO) <<"root_cluster_.edges.size: "<<root_cluster_.edges.size();
  LOG(INFO) << "Subclusters Clustering Started ";

  const std::unique_ptr<Cluster> cluster = this->CreateCluster();
  CHECK_NOTNULL(cluster.get());
  LOG(INFO) << "wanted cluster num: " << num_clusters;
  cluster->InitIGraph(image_pairs, weights);


  std::ofstream edges_out(options_.graph_dir+"/subcluster_edges.txt");
  LOG(INFO) << "Output edges to " << options_.graph_dir+"/subcluster_edges.txt";
  edges_out<<image_pairs.size()<<std::endl;
  for(int i=0; i<image_pairs.size(); i++)
  {
    edges_out<<image_pairs[i].first<<"\t"<<image_pairs[i].second<<"\t"<<weights[i]<<std::endl;
  }
  edges_out.close();

    // cluster->OutputIGraph(dst_path);
  // std::unordered_map<int, int> labels =
  //     cluster->ComputeCluster(image_pairs, weights, num_clusters);
  labels = cluster->ComputeCluster(image_pairs, weights, num_clusters);

  LOG(INFO)<<"ComputeCluster";

  cluster_num = cluster->ClusterNum();
  LOG(INFO) << "got cluster num: " << cluster_num;

  ////////////////计算分类之后的最大最小cluster
	std::unordered_map<int,int> cluster_id_img_nums;

	for(auto label : labels)
	{
		int cluster_id = label.second;
		if(cluster_id_img_nums.count(cluster_id)==0)
			cluster_id_img_nums.insert(std::make_pair(cluster_id,1));
		else
			cluster_id_img_nums[cluster_id]++;
	}
  // std::ofstream labels_out("/16t/gy/cluster_stat.txt");
  // for(auto itr : cluster_id_img_nums)  
  //   labels_out<<itr.first<<"\t"<<itr.second<<std::endl;
  // labels_out.close();

	int max_cluster_size = 0;
  int min_cluster_size = 1000;
  int max_cluster_id = 0;
  int min_cluster_id = 0;

	for(auto itr:cluster_id_img_nums)
	{
		if(itr.second>max_cluster_size)
		{
			max_cluster_size = itr.second;
			max_cluster_id = itr.first;
		}
		if(itr.second<min_cluster_size)
		{
			min_cluster_size = itr.second;
			min_cluster_id = itr.first;
      

      LOG(INFO)<<itr.first<<" "<<itr.second;
		}
	}
	LOG(INFO)<<"the cluster "<<max_cluster_id<<" has max "<<max_cluster_size<< " subclusters"<<std::endl;
  LOG(INFO)<<"the cluster "<<min_cluster_id<<" has min "<<min_cluster_size<< " subclusters"<<std::endl;

  LOG(INFO)<<"cluter size before split: "<<num_clusters;
  // exit(EXIT_SUCCESS);



  //////////////////////// Output full subclusters connection to igraph /////////////////////      
  std::string subclusters_dst_path = options_.graph_dir + "/igraph_subclusters/";
  CreateDirIfNotExists(subclusters_dst_path);
  LOG(INFO) << "Output igraph to " << subclusters_dst_path;
  // cluster->OutputIGraphInfo(image_pairs, weights, subclusters_dst_path);
  cluster->OutputGraphJson(image_pairs, weights, labels, cluster_num, subclusters_dst_path);
  // if (options_.is_output_igraph) {
  //   LOG(INFO) << "Output igraph to " << options_.graph_dir;
  //   CreateDirIfNotExists(options_.graph_dir);
  //   cluster->OutputIGraph(options_.graph_dir);
    // cluster->MyOutputClusterBeforeExpand(options_.graph_dir,labels); // mycode
  // }
  


  /////////注意 Ncut分类结果存在一个问题：同一类内有可能存在多个互不相连的区域，
  /////////因此需要在分类之后split这些独立区域成为单独的cluster
  /////////再将这些碎片cluster merge到最相关的cluster内
  //////////////////  split cluster内部不连通的独立区域，成为新的cluster (通过修改labels)   //////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  LOG(INFO) << "Cutting Complete, split unconnected parts...";
  std::vector<std::vector<image_t>> cluster_imgs;
  std::vector<std::vector<std::pair<int, int>>> cluster_edges; //大cluster -> cluster内部的edges
  cluster_imgs.resize(cluster->ClusterNum());
  cluster_edges.resize(cluster->ClusterNum());
  for (const auto label : labels) {
    cluster_imgs[label.second].push_back(label.first);
  }
  LOG(INFO) << "Grouping subcluster edges for split...";
  for (size_t k = 0; k < image_pairs.size(); k++) {
    const image_t i = image_pairs[k].first;
    const image_t j = image_pairs[k].second;
    const int cluster_id1 = labels[i];
    const int cluster_id2 = labels[j];

    if (cluster_id1 == cluster_id2) {
      cluster_edges[cluster_id1].push_back(std::make_pair(i, j));
    }
  }

  int current_cluster_size = cluster_num;
  for(int i=0; i<current_cluster_size; i++)
  {
    std::vector<image_t> subcluster_ids = cluster_imgs[i];
    std::vector<std::pair<int, int>> subcluster_edges =  cluster_edges[i];
    std::unordered_map<size_t, std::vector<image_t>> components;
    CutUnconnectedClusters(subcluster_ids, subcluster_edges, components);
    if(components.size()>1)//有超过一个独立区域
    {
      // LOG(INFO)<<"it should not be more than one connected part !!!";
      // exit(EXIT_FAILURE);
      // cluster_num += components.size()-1;
      int component_id = 0;
      for(auto subcluster_itr : components)
      {
        component_id++;
        if(component_id == 1)//第一个类不需要修改labels
          continue;
        cluster_num++;
        for(int j=0; j<subcluster_itr.second.size(); j++) //遍历其他区域的每个subcluster_id, 修改其在labels中的标签
        {
          image_t subcluster_id = subcluster_itr.second[j];
          labels[subcluster_id] = cluster_num-1;//由于cluster_id从0开始计数，所以此处要-1 
          LOG(INFO)<<"change label of subcluster "<<subcluster_id<<" to "<<cluster_num-1;
        }
      }
    }    
  }

  LOG(INFO)<<"cluter size after split: "<<cluster_num;
  LOG(INFO) << "Split Complete, merge small parts...";

  int subcluster_num_min_thresh = 10;//10;
  while(true)
  {    
    LOG(INFO)<<"NEW ITERATION...";
    std::unordered_map<int, std::vector<image_t>> cluster_imgs_m;
    for (const auto label : labels) {
      cluster_imgs_m[label.second].push_back(label.first);
    }

    //test code :
    for(auto cluster : cluster_imgs_m) //判断当前循环内是否需要merge
    {
      if(cluster.second.size()<subcluster_num_min_thresh)
      {
        LOG(INFO)<<"TEST: CLUSTER "<<cluster.first<<" HAS "<<cluster.second.size()<<" SUBCLUSTERS";
      }
    }

    bool need_merge = false;
    for(auto cluster : cluster_imgs_m) //判断当前循环内是否需要merge
    {
      if(cluster.second.size()<subcluster_num_min_thresh)
      {
        need_merge = true;
        break;
      }
    }
    if(!need_merge)//如果所有cluster size都大于一定阈值，则跳出循环
      break;

    LOG(INFO) << "Finding best matched cluster for small cluster...";
    // LOG(INFO)<<"labels.size(): "<<labels.size();
    std::vector<int> best_intersubcluster_connected_id(labels.size(),-1); //保存每一个subcluster在其cluster之外的最佳关联subcluster
    std::vector<int> best_intersubcluster_connected_weight(labels.size(),-1);
    for (size_t k = 0; k < image_pairs.size(); k++) {
      const image_t i = image_pairs[k].first;
      const image_t j = image_pairs[k].second;
      const int cluster_id1 = labels[i];
      const int cluster_id2 = labels[j];
      int weight = weights[k];
      if (cluster_id1 != cluster_id2) {
        if( weight >= best_intersubcluster_connected_weight[i] )
        {        
          best_intersubcluster_connected_id[i] = j;
          best_intersubcluster_connected_weight[i] = weight;
        }
        if( weight >= best_intersubcluster_connected_weight[j] )
        {
          best_intersubcluster_connected_id[j] = i;
          best_intersubcluster_connected_weight[j] = weight;
        }      
      }
    }

    for(auto cluster : cluster_imgs_m)
    {
      if(cluster.second.size() < subcluster_num_min_thresh) //cluster内部subcluster数量小于10则merge
      {
        LOG(INFO)<<"current cluster: "<<cluster.first<<" size: "<<cluster.second.size();
        cluster_num--;
        int target_cluster_id = -1;
        int best_weight = -1;
        for(auto itr_subcluster: cluster.second)
        {
          // LOG(INFO)<<"has subcluster id: "<<itr_subcluster;
          // // LOG(INFO)<<"best_intersubcluster_connected_weight[itr_subcluster]: "<<best_intersubcluster_connected_weight[itr_subcluster];
          // LOG(INFO)<<"best_intersubcluster_connected_id[itr_subcluster]: "<<best_intersubcluster_connected_id[itr_subcluster];
          //////////注意此处不应直接把labels[best_intersubcluster_connected_id[itr_subcluster]]输出，会导致出现 labels[-1]=0
          // LOG(INFO)<<"the best connected cluster: "<<labels[best_intersubcluster_connected_id[itr_subcluster]];
          if(best_intersubcluster_connected_weight[itr_subcluster] > best_weight)
          {          
            target_cluster_id = labels[best_intersubcluster_connected_id[itr_subcluster]]; //由当前最佳连接subcluster找到他的最佳连接cluster
            best_weight = best_intersubcluster_connected_weight[itr_subcluster];
          }
        }

        if(target_cluster_id==-1)
        {
          LOG(INFO)<<"this cluster has no connection to other clusters";
          continue;
        }
        for(auto itr_subcluster: cluster.second)
        {
          labels[itr_subcluster] = target_cluster_id;
          LOG(INFO)<<"merge subcluster "<<itr_subcluster<<" to cluster "<<target_cluster_id<< " which has "<<cluster_imgs_m[target_cluster_id].size()<<" subclusters";
        }

        //如果对应的最佳cluster小于阈值，则开启下一轮循环merge，因为当前merge改变了targe cluster的size
        if(cluster_imgs_m[target_cluster_id].size() < subcluster_num_min_thresh) 
          break;
      }
    }
  }

  LOG(INFO)<<"cluster_num after merge: "<<cluster_num;
  //merge之后整理类id为[0,cluster_num-1]
  std::unordered_map<int, std::vector<image_t>> cluster_imgs_m;
  std::set<int> current_cluster_ids;
  std::vector<int> change_cluster_ids;
  for (const auto label : labels) {
    cluster_imgs_m[label.second].push_back(label.first);
    current_cluster_ids.insert(label.second);
    if(label.second>=cluster_num)
    {
      change_cluster_ids.push_back(label.second);
    }
  }
  std::sort(change_cluster_ids.begin(), change_cluster_ids.end());
  change_cluster_ids.erase(std::unique(change_cluster_ids.begin(), change_cluster_ids.end()), change_cluster_ids.end());
  LOG(INFO)<<"sum of cluster ids need to change: "<<change_cluster_ids.size();
  LOG(INFO)<<"current cluster ids size: "<<current_cluster_ids.size();
  std::unordered_map<int,int>old_new_cluster_id;
  for(int i=0; i<cluster_num; i++)
  {
    if(current_cluster_ids.count(i)==0) //当前类id不存在
    {
      old_new_cluster_id.insert(std::make_pair(change_cluster_ids.back(),i));
      change_cluster_ids.pop_back();
    }
  } 
  for(auto itr:old_new_cluster_id)
  {
    LOG(INFO)<<"change cluster_id "<<itr.first<<" to "<<itr.second;
    for(int i = 0; i < cluster_imgs_m[itr.first].size(); i++)
    {      
      labels[cluster_imgs_m[itr.first][i]] = itr.second;
    }
  }





  // Collect nodes according to the partition result
  // intra_clusters_.resize(cluster->ClusterNum());

  LOG(INFO)<<"cluter size after merge: "<<cluster_num;
  intra_clusters_.resize(cluster_num);
  for (const auto label : labels) {
    // LOG(INFO)<<"label.second: "<<label.second<<", label.first: "<<label.first;
    intra_clusters_[label.second].image_ids.push_back(label.first);
  }
  // LOG(INFO)<<"seting intra_clusters id...";
  for (uint i = 0; i < intra_clusters_.size(); i++) {
    intra_clusters_[i].cluster_id = i;
  }
  LOG(INFO) <<"image_pairs.size: "<<image_pairs.size();
  // Grouping edges
  LOG(INFO) << "Grouping subcluster edges...";
  for (size_t k = 0; k < image_pairs.size(); k++) {
    const image_t i = image_pairs[k].first;
    const image_t j = image_pairs[k].second;
    const int cluster_id1 = labels[i];
    const int cluster_id2 = labels[j];


    //     if(i==1045 || j==1045)
    // {
    //   LOG(INFO)<<"image pairs: "<<i<<" "<<j;
    //   LOG(INFO)<<"cluster ids: "<<cluster_id1<<", "<<cluster_id2;
    // }

    if (cluster_id1 == cluster_id2) {
      intra_clusters_[cluster_id1].edges.insert(
          std::make_pair(ImagePair(i, j), weights[k]));
    } else {
      const ImagePair view_pair = cluster_id1 < cluster_id2
                                      ? ImagePair(cluster_id1, cluster_id2)
                                      : ImagePair(cluster_id2, cluster_id1);
      clusters_lost_edges_[view_pair].push_back(graph::Edge(i, j, weights[k]));
    }
  }

  // for(auto it : clusters_lost_edges_)
  // {
  //   std::vector<graph::Edge> lost_edges = it.second;
  //   if(it.first.first == 73 || it.first.second ==73)
  //   {
  //     LOG(INFO)<<"cluster pair is: "<<it.first.first<<","<<it.first.second;
  //     for(int i=0; i<lost_edges.size(); i++)
  //     {
  //       LOG(INFO)<<lost_edges[i].src<<"-"<<lost_edges[i].dst<<"-"<<lost_edges[i].weight;
  //     }
  //   }
  // }



  timer_.Pause();
  summary_.total_cutting_num = 1;
  summary_.total_cutting_time = timer_.ElapsedSeconds();


  //////////////////////// Output intracluster connection to igraph /////////////////////
  for(int i=0; i<cluster_num; i++)
  {  
    std::vector<std::pair<int, int>> image_pairs_intra;
    std::vector<int> weights_intra;
    for(auto intra_edge : intra_clusters_[i].edges) {
      image_pairs_intra.push_back(std::make_pair(intra_edge.first.first,intra_edge.first.second));
      weights_intra.push_back(intra_edge.second); 
    }
    const std::unique_ptr<Cluster> intra_cluster = this->CreateCluster();
    CHECK_NOTNULL(intra_cluster.get());
    std::string intra_dst_path = options_.graph_dir + "/igraph_intracluster/";
    CreateDirIfNotExists(intra_dst_path);
    intra_dst_path += std::to_string(i)+"/";
    LOG(INFO) << "Output igraph to " << intra_dst_path;
    CreateDirIfNotExists(intra_dst_path);

    // intra_cluster->OutputIGraphInfo(image_pairs_intra, weights_intra, intra_dst_path);
    intra_cluster->OutputGraphJson(image_pairs_intra, weights_intra, intra_dst_path);
    // cluster->OutputIGraph(dst_path);
  }
}

void ImageClustering::Cut() {
  timer_.Start();
  const uint num_clusters =
      root_cluster_.image_ids.size() / options_.num_images_ub;
  // const uint num_clusters = 2;
  CHECK_GE(num_clusters, 1);

  std::vector<std::pair<int, int>> image_pairs;
  std::vector<int> weights;
  image_pairs.reserve(root_cluster_.edges.size());
  weights.reserve(root_cluster_.edges.size());

  for (auto edge : root_cluster_.edges) {
    image_pairs.push_back(std::make_pair(edge.first.first, edge.first.second));
    weights.push_back(edge.second);
  }

  LOG(INFO) << "Images Clustering Started ";

  const std::unique_ptr<Cluster> cluster = this->CreateCluster();
  CHECK_NOTNULL(cluster.get());
  LOG(INFO) << "cluster num: " << num_clusters;
  cluster->InitIGraph(image_pairs, weights);
  std::unordered_map<int, int> labels =
      cluster->ComputeCluster(image_pairs, weights, num_clusters);
LOG(INFO) << "ComputeCluster finished: " << num_clusters;
        for(const auto label:labels)
    {
      LOG(INFO)<<label.first<<" label:"<<label.second;
    }
  if (options_.is_output_igraph) {
    LOG(INFO) << "Output igraph to " << options_.graph_dir;
    CreateDirIfNotExists(options_.graph_dir);
    cluster->OutputIGraph(options_.graph_dir);
    cluster->MyOutputClusterBeforeExpand(options_.graph_dir,labels); // mycode
  }
  LOG(INFO) << "Cutting Complete, grouping images...";

  // Collect nodes according to the partition result
  intra_clusters_.resize(cluster->ClusterNum());
  for (const auto label : labels) {
    intra_clusters_[label.second].image_ids.push_back(label.first);
  }
  for (uint i = 0; i < intra_clusters_.size(); i++) {
    intra_clusters_[i].cluster_id = i;
  }

  // Grouping edges
  LOG(INFO) << "Grouping edges...";
  for (size_t k = 0; k < image_pairs.size(); k++) {
    const image_t i = image_pairs[k].first;
    const image_t j = image_pairs[k].second;
    const int cluster_id1 = labels[i];
    const int cluster_id2 = labels[j];

    if (cluster_id1 == cluster_id2) {
      intra_clusters_[cluster_id1].edges.insert(
          std::make_pair(ImagePair(i, j), weights[k]));
    } else {
      const ImagePair view_pair = cluster_id1 < cluster_id2
                                      ? ImagePair(cluster_id1, cluster_id2)
                                      : ImagePair(cluster_id2, cluster_id1);
      clusters_lost_edges_[view_pair].push_back(graph::Edge(i, j, weights[k]));
    }
  }

  timer_.Pause();
  summary_.total_cutting_num = 1;
  summary_.total_cutting_time = timer_.ElapsedSeconds();
}

void ImageClustering::ExpandByStrongEdges() {

  LOG(INFO) << "Expanding Images...";
  //   for(auto it : clusters_lost_edges_)
  // {
  //   std::vector<graph::Edge> lost_edges = it.second;
  //   if(it.first.first == 73 || it.first.second ==73)
  //   {
  //     LOG(INFO)<<"cluster pair is: "<<it.first.first<<","<<it.first.second;
  //     for(int i=0; i<lost_edges.size(); i++)
  //     {
  //       LOG(INFO)<<lost_edges[i].src<<"-"<<lost_edges[i].dst<<"-"<<lost_edges[i].weight;
  //     }
  //   }
  // }



  const uint num_clusters = intra_clusters_.size();
  inter_clusters_.reserve(num_clusters);
  for (auto cluster : intra_clusters_) {
    inter_clusters_.emplace_back(cluster);
  }
  
  LOG(INFO)<<"clusters_lost_edges_.size(): "<<clusters_lost_edges_.size();
  timer_.Start();
  if (num_clusters > 1) 
  {
    for (auto it : clusters_lost_edges_) {
      const ImagePair cluster_pair = it.first;
      std::vector<graph::Edge> lost_edges = it.second;

      // LOG(INFO)<<"lost_edges.size(): "<<lost_edges.size();

    // LOG(INFO)<<"just for test";

    // // std::vector<graph::Edge> lost_edges = it.second;
    // if(it.first.first == 73 || it.first.second ==73)
    // {
    //   LOG(INFO)<<"cluster pair is: "<<it.first.first<<","<<it.first.second;
    //   for(int i=0; i<lost_edges.size(); i++)
    //   {
    //     LOG(INFO)<<lost_edges[i].src<<"-"<<lost_edges[i].dst<<"-"<<lost_edges[i].weight;
    //   }
    // }
      
      const auto cmp = [](const graph::Edge& edge1, const graph::Edge& edge2) {
        return edge1.weight > edge2.weight;
      };
      sort(lost_edges.begin(),lost_edges.end(),cmp);
      //只保留大于20的边（todo 应改为可选参数）
      if(lost_edges.size()>3)
      {
        for(int i = 0; i< lost_edges.size(); i++)
        {
          if(lost_edges[i].weight<20) //若第i个edge已经小于20
          {
            if(i<=3)
            {
              lost_edges.resize(3);
            }
            else if(i<=10)
            {
              lost_edges.resize(i);
            }
            else
              lost_edges.resize(10);
            break;
          }
        }
      }

      

      // if(lost_edges[0].weight<20)
      // {
      //   lost_edges.resize( lost_edges.size() > 3 ? 3 : lost_edges.size());//最多保留3条边
      // }
      // else
      // {
      //   for(int i = 0; i< lost_edges.size(); i++)
      //   {
      //     if(lost_edges[i].weight<20) //若第i个edge已经小于20
      //     {
      //       lost_edges.resize(i);
      //       break;
      //     }
      //   }
      // }  



      if(lost_edges.size()==0) //没有足够的strong edges，跳过
        continue;
      // else if(lost_edges.size()>7) //最多只保留10条最佳edges
      //   lost_edges.resize(7);

      LOG(INFO)<< "cluster pair: ("<<cluster_pair.first<<", "<<cluster_pair.second<<")";
      for(int i=0;i<lost_edges.size();i++)
      {
        LOG(INFO)<<"\t subcluster pair: ("<<lost_edges[i].src<<", "<<lost_edges[i].dst<<") with weight: "<<lost_edges[i].weight;
      }


      ImageCluster& cluster1 = inter_clusters_[cluster_pair.first];
      ImageCluster& cluster2 = inter_clusters_[cluster_pair.second];
      AddLostEdgesBetweenClusters(cluster1, cluster2, lost_edges);
    }
  }

  for (uint i = 0; i < inter_clusters_.size(); i++) {
    uint repeated_node_num = 0;    
    for (uint j = 0; j < inter_clusters_.size(); j++) {
      if (i == j) continue;
      const uint common_images_num =
          CommonImagesNum(inter_clusters_[i], inter_clusters_[j]);
      if(common_images_num!=0)
        LOG(INFO)<<"common images between inter_cluster ["<<i<<","<<j<<"]: "<<common_images_num;
      repeated_node_num += common_images_num;
    }
    const float repeated_ratio =
      (float)repeated_node_num / (float)(inter_clusters_[i].image_ids.size());
    LOG(INFO)<<"shared images in cluster " << i<<": "<<repeated_node_num<<" with ratio: "<<repeated_ratio;
  }

  timer_.Pause();
  summary_.total_expansion_time = timer_.ElapsedSeconds();
  summary_.total_expansion_num = 1;
  summary_.total_time =
      summary_.total_cutting_time + summary_.total_expansion_time;
  AnalyzeStatistic();
  MyOutputClustersExpanded(options_.graph_dir);//my code
}

void ImageClustering::Expand() {
  LOG(INFO) << "Expanding Images...";

  const uint num_clusters = intra_clusters_.size();
  inter_clusters_.reserve(num_clusters);
  for (auto cluster : intra_clusters_) {
    inter_clusters_.emplace_back(cluster);
  }

  timer_.Start();
  if (num_clusters > 1) {
    for (auto it : clusters_lost_edges_) {
      const ImagePair cluster_pair = it.first;
      std::vector<graph::Edge> lost_edges = it.second;
      ImageCluster& cluster1 = inter_clusters_[cluster_pair.first];
      ImageCluster& cluster2 = inter_clusters_[cluster_pair.second];
      AddLostEdgesBetweenClusters(cluster1, cluster2, lost_edges);
    }
  }

  timer_.Pause();
  summary_.total_expansion_time = timer_.ElapsedSeconds();
  summary_.total_expansion_num = 1;
  summary_.total_time =
      summary_.total_cutting_time + summary_.total_expansion_time;
  AnalyzeStatistic();
  MyOutputClustersExpanded(options_.graph_dir);//my code
}



void ImageClustering::ExpandAllEdges() {
  LOG(INFO) << "Expanding All Lost Edges...";

  const uint num_clusters = intra_clusters_.size();
  inter_clusters_.reserve(num_clusters);
  for (auto cluster : intra_clusters_) {
    inter_clusters_.emplace_back(cluster);
  }

  timer_.Start();
  if (num_clusters > 1) {
    for (auto it : clusters_lost_edges_) {
      const ImagePair cluster_pair = it.first;
      std::vector<graph::Edge> lost_edges = it.second;

      ImageCluster& cluster1 = inter_clusters_[cluster_pair.first];
      ImageCluster& cluster2 = inter_clusters_[cluster_pair.second];

      for (const graph::Edge& edge : lost_edges) {
        const ImagePair image_pair = edge.src < edge.dst
                                         ? ImagePair(edge.src, edge.dst)
                                         : ImagePair(edge.dst, edge.src);

        ImageCluster& selected_cluster =
            cluster1.edges.size() > cluster2.edges.size() ? cluster2 : cluster1;
        const std::unordered_set<image_t> images(
            selected_cluster.image_ids.begin(),
            selected_cluster.image_ids.end());
        if (images.count(edge.src) == 0) {
          selected_cluster.image_ids.push_back(edge.src);
        }
        if (images.count(edge.dst) == 0) {
          selected_cluster.image_ids.push_back(edge.dst);
        }

        selected_cluster.edges[image_pair] = edge.weight;
      }
    }
  }

  timer_.Pause();
  summary_.total_expansion_time = timer_.ElapsedSeconds();
  summary_.total_expansion_num = 1;
  summary_.total_time =
      summary_.total_cutting_time + summary_.total_expansion_time;
  AnalyzeStatistic();
}

std::vector<ImageCluster> ImageClustering::BiCut(
    const ImageCluster& image_cluster) {
  timer_.Start();
  std::vector<ImageCluster> image_clusters(options_.branching_factor);
  std::vector<std::pair<int, int>> image_pairs;
  std::vector<int> weights;

  for (auto edge : image_cluster.edges) {
    image_pairs.push_back(std::make_pair(edge.first.first, edge.first.second));
    weights.push_back(edge.second);
  }

  const std::unique_ptr<Cluster> cluster = this->CreateCluster();
  std::unordered_map<int, int> labels =
      cluster->ComputeCluster(image_pairs, weights, options_.branching_factor);

  // collect nodes according to the partition result
  for (const auto label : labels) {
    image_clusters[label.second].image_ids.push_back(label.first);
  }

  for (size_t k = 0; k < image_pairs.size(); k++) {
    const image_t i = image_pairs[k].first;
    const image_t j = image_pairs[k].second;
    const int cluster_id1 = labels[i];
    const int cluster_id2 = labels[j];

    if (cluster_id1 == cluster_id2) {
      image_clusters[cluster_id1].edges.insert(
          std::make_pair(ImagePair(i, j), weights[k]));
    } else {
      discarded_edges_.push(graph::Edge(i, j, weights[k]));
    }
  }

  timer_.Pause();
  summary_.total_cutting_time += timer_.ElapsedSeconds();
  summary_.total_cutting_num++;

  return image_clusters;
}

void ImageClustering::CutAndExpand() {
  Timer timer;
  timer.Start();

  std::vector<ImageCluster> init_clusters = BiCut(root_cluster_);
  std::queue<ImageCluster> candidate_clusters;
  for (auto cluster : init_clusters) {
    candidate_clusters.push(cluster);
  }

  while (!candidate_clusters.empty()) {
    LOG(INFO) << summary_.total_iters_num++ << "-th iterations";

    while (!candidate_clusters.empty()) {
      ImageCluster cluster = candidate_clusters.front();
      candidate_clusters.pop();
      if (cluster.image_ids.size() <= options_.num_images_ub) {
        inter_clusters_.push_back(cluster);
      } else {
        std::vector<ImageCluster> clusters = BiCut(cluster);
        for (int k = 0; k < options_.branching_factor; k++) {
          candidate_clusters.push(clusters[k]);
        }
      }
    }

    // Graph Expansion
    timer_.Start();
    while (!discarded_edges_.empty()) {
      // If there is no cluster satisfies completeness ratio constraint
      // we can jump out of the loop
      if (!IsRemainingClusters()) {
        graph::LargerEdgePriorityQueue<graph::Edge> empty_edges;
        std::swap(empty_edges, discarded_edges_);
        break;
      }
      graph::Edge edge = discarded_edges_.top();
      discarded_edges_.pop();

      const int cluster_id = ClusterSatisfyCompletenessRatio(edge);
      if (cluster_id == -1) {
        LOG(INFO) << "Not find suitable clusters for edge: " << edge.src << ", "
                  << edge.dst;
        continue;
      }

      ImageCluster& image_cluster = inter_clusters_[cluster_id];
      std::unordered_set<image_t> image_sets(image_cluster.image_ids.begin(),
                                             image_cluster.image_ids.end());
      image_t added_image =
          image_sets.find(edge.src) == image_sets.end() ? edge.src : edge.dst;
      VLOG(2) << "discarded edge: " << edge.src << ", " << edge.dst;
      VLOG(2) << "added image:    " << added_image;
      if (image_sets.find(added_image) == image_sets.end()) {
        image_cluster.image_ids.push_back(added_image);
      }
      image_cluster.edges.insert(
          std::make_pair(ImagePair(edge.src, edge.dst), edge.weight));
    }
    timer_.Pause();
    summary_.total_expansion_time += timer_.ElapsedSeconds();
    summary_.total_expansion_num++;

    // After graph expansion, there may be some image graphs that don't
    // satisfy the size constraint, thus we need to re-partition it
    LOG(INFO) << "Re-grouping clusters...";
    for (auto iter = inter_clusters_.begin(); iter != inter_clusters_.end();) {
      if (iter->image_ids.size() >
          options_.relax_ratio * options_.num_images_ub) {
        candidate_clusters.push(*iter);
        iter = inter_clusters_.erase(iter);
      } else {
        iter++;
      }
    }
  }

  timer.Pause();
  summary_.total_time += timer.ElapsedSeconds();
  LOG(INFO) << "Image Clustering Complete!";
  AnalyzeStatistic();
}

const std::vector<ImageCluster>& ImageClustering::GetIntraClusters() const {
  return intra_clusters_;
}

const std::vector<ImageCluster>& ImageClustering::GetInterClusters() const {
  return inter_clusters_;
}

const ImageCluster& ImageClustering::GetRootCluster() const {
  return root_cluster_;
}

double ImageClustering::AnalyzeDegree(
    const std::vector<std::pair<int, int>>& image_pairs,
    const std::vector<int>& weights) const {
  using namespace DAGSfM::graph;
  Graph<Node, Edge> graph;
  for (uint i = 0; i < image_pairs.size(); i++) {
    graph.AddEdge(
        Edge(image_pairs[i].first, image_pairs[i].second, weights[i]));
    graph.AddEdge(
        Edge(image_pairs[i].second, image_pairs[i].first, weights[i]));
  }

  graph.CountInDegrees();
  graph.CountOutDegrees();
  graph.CountDegrees();

  // TODO: (chenyu) degrees are depends on graph weights.
  const std::unordered_map<size_t, size_t> degrees = graph.GetDegrees();

  // Analyze the degrees of view graph.
  double ave_degree = 0.0;
  for (auto degree : degrees) {
    ave_degree += degree.second;
  }
  ave_degree /= degrees.size();

  double covariance_degree = 0.0;
  for (auto degree : degrees) {
    covariance_degree +=
        (degree.second - ave_degree) * (degree.second - ave_degree);
  }
  covariance_degree /= degrees.size();

  return covariance_degree / ave_degree;
}

std::unique_ptr<Cluster> ImageClustering::CreateCluster() const {
  if (options_.cluster_type == "NCUT") {
    return std::unique_ptr<Cluster>(new NCutCluster());
  } else if (options_.cluster_type == "KMEANS") {
    return std::unique_ptr<Cluster>(new KMeansCluster());
  } else if (options_.cluster_type == "SPECTRAL") {
    return std::unique_ptr<Cluster>(new SpectralCluster());
  } else if (options_.cluster_type == "HYBRID") {
    return std::unique_ptr<Cluster>(new HybridCluster());
  } else if (options_.cluster_type == "COMMUNITY_DETECTION") {
    return std::unique_ptr<Cluster>(new CommunityDetectionCluster());
  } else if (options_.cluster_type == "METRIC") {
    return std::unique_ptr<Cluster>(new MetricCluster());
  } else {
    return std::unique_ptr<Cluster>(new NCutCluster());
  }
}

std::unique_ptr<Cluster> ImageClustering::CreateCluster(
    const std::vector<std::pair<int, int>>& image_pairs,
    const std::vector<int>& weights)  // const
{
  const double sigma = 4.0;
  const double degree_distribution = AnalyzeDegree(image_pairs, weights);

  if (degree_distribution >= sigma) {
    options_.cluster_type = "HYBRID";
    return std::unique_ptr<Cluster>(new HybridCluster());
  } else {
    options_.cluster_type = "NCUT";
    return std::unique_ptr<Cluster>(new NCutCluster());
  }
}

bool ImageClustering::IsSatisfyCompletenessRatio(const ImageCluster& cluster) {
  // if (cluster.is_condition_satisfy) return true;

  // const uint i = cluster.cluster_id;
  // std::unordered_set<image_t> image_sets(cluster.image_ids.begin(),
  //                                        cluster.image_ids.end());

  // uint repeated_node_num = 0;
  // for (uint j = 0; j < inter_clusters_.size(); j++) {
  //   if (i == j) continue;
  //   const uint common_images_num =
  //       CommonImagesNum(inter_clusters_[i], inter_clusters_[j]);
  //   repeated_node_num += common_images_num;
  // }

  // // check if satisfy completeness ratio to avoid adding too many edges
  // const float repeated_ratio =
  //     (float)repeated_node_num / (float)(inter_clusters_[i].image_ids.size());
  // if (repeated_ratio <= options_.completeness_ratio) {
  //   VLOG(4) << "repeated ratio: " << repeated_ratio;
  //   return false;
  // } else {
  //   inter_clusters_[i].is_condition_satisfy = true;
  //   return true;
  // }
  return false;
}

int ImageClustering::ClusterSatisfyCompletenessRatio(const graph::Edge& edge) {
  int cluster_id = -1;

  for (uint i = 0; i < inter_clusters_.size() - 1; i++) {
    std::unordered_set<image_t> image_sets(inter_clusters_[i].image_ids.begin(),
                                           inter_clusters_[i].image_ids.end());
    if (image_sets.find(edge.src) == image_sets.end() &&
        image_sets.find(edge.dst) == image_sets.end()) {
      continue;
    }

    uint repeated_node_num = 0;
    for (uint j = 0; j < inter_clusters_.size(); j++) {
      if (i == j) continue;
      const uint common_images_num =
          CommonImagesNum(inter_clusters_[i], inter_clusters_[j]);
      // LOG(INFO) << "common images num: " << common_images_num;
      repeated_node_num += common_images_num;
    }

    // check if satisfy completeness ratio to avoid adding too many edges
    const float repeated_ratio =
        (float)repeated_node_num / (float)(inter_clusters_[i].image_ids.size());
    if (repeated_ratio <= options_.completeness_ratio) {
      VLOG(4) << "repeated ratio: " << repeated_ratio;
      cluster_id = i;
      break;
    } else {
      // LOG(INFO) << "cluster " << i << " repeated ratio: " << repeated_ratio;
      // LOG(INFO) << "total clusters: " << inter_clusters_.size();
      inter_clusters_[i].is_condition_satisfy = true;
    }
  }

  return cluster_id;
}

uint ImageClustering::CommonImagesNum(const ImageCluster& cluster1,
                                      const ImageCluster& cluster2) const {
  uint common_images_num = 0;
  const std::unordered_set<image_t> images1(cluster1.image_ids.begin(),
                                            cluster1.image_ids.end());
  const std::unordered_set<image_t> images2(cluster2.image_ids.begin(),
                                            cluster2.image_ids.end());

  for (auto it = images1.begin(); it != images1.end(); ++it) {
    if (images2.find(*it) != images2.end()) {
      common_images_num++;
    }
  }
  // VLOG(2) << "common images num: " << common_images_num;
  return common_images_num;
}

void ImageClustering::OutputClusteringSummary() const {
  LOG(INFO)
      << "#Images Clustering Config:#\n"
      << "\t - image upperbound: " << options_.num_images_ub << "\n"
      << "\t - completeness ratio: " << options_.completeness_ratio << "\n"
      << "\t - cluster type: " << options_.cluster_type << "\n"
      << "#Images Clustering Summary:#\n"
      << "\t - Clusters number: " << inter_clusters_.size() << "\n"
      << "\t - Total graph cutting time: " << summary_.total_cutting_time
      << " seconds\n"
      << "\t - Total graph cutting number: " << summary_.total_cutting_num << "\n"
      << "\t - Total graph expansion time: " << summary_.total_expansion_time
      << " seconds\n"
      << "\t - Total graph expansion number: " << summary_.total_expansion_num
      << "\n"
      << "\t - Total time took: " << summary_.total_time << " seconds\n"
      << "\t - Total iteration number: " << summary_.total_iters_num << "\n"
      << "\t - Images number expanded from " << summary_.original_images_num
      << " to " << summary_.clustered_images_num << "\n"
      << "\t - Repeated Ratio: "
      << (float)(summary_.clustered_images_num - summary_.original_images_num) /
             (float)summary_.original_images_num
      << "\n"
      << "\t - Edges number reduced from " << summary_.original_edges_num << " to "
      << summary_.clustered_edges_num << "\n"
      << "\t - Lost ratio: "
      << (float)(summary_.original_edges_num - summary_.clustered_edges_num) /
             (float)summary_.original_edges_num;
}

bool ImageClustering::IsRemainingClusters() const {
  for (auto cluster : inter_clusters_) {
    if (!cluster.is_condition_satisfy) return true;
  }
  return false;
}

void ImageClustering::AddLostEdgesBetweenClusters(
    ImageCluster& cluster1, ImageCluster& cluster2,
    std::vector<graph::Edge>& lost_edges) {
  // The intersection of cluster1 and cluster2 can't surpass the
  // maximum image overlapping.

  // if(cluster1.cluster_id == 16 && cluster2.cluster_id == 27)
  // {
  //   LOG(INFO)<<"current CommonImagesNum: "<<CommonImagesNum(cluster1, cluster2);
  //   LOG(INFO)<<"is_condition_satisfy?"<<cluster1.is_condition_satisfy<<", "<<cluster2.is_condition_satisfy;
  // }


  if (CommonImagesNum(cluster1, cluster2) > options_.image_overlap) {
    return;
  }

  // Either cluster1 or cluster2 should not achieve the completeness ratio.
  if (IsSatisfyCompletenessRatio(cluster1) &&
      IsSatisfyCompletenessRatio(cluster2)) {
    return;
  }

  const auto cmp = [](const graph::Edge& edge1, const graph::Edge& edge2) {
    return edge1.weight > edge2.weight;
  };
  std::sort(lost_edges.begin(), lost_edges.end(), cmp);


  // if(cluster1.cluster_id == 16 && cluster2.cluster_id == 27)
  // {
  //   LOG(INFO)<<"lost_edges: "<<lost_edges.size();
  // }

  // RandomNumberGenerator rng;
  for (uint k = 0; /*k < options_.image_overlap &&*/ k < lost_edges.size();
       k++) {
    const ImagePair view_pair =
        lost_edges[k].src < lost_edges[k].dst
            ? ImagePair(lost_edges[k].src, lost_edges[k].dst)
            : ImagePair(lost_edges[k].dst, lost_edges[k].src);

    const std::unordered_set<image_t> images1(cluster1.image_ids.begin(),
                                              cluster1.image_ids.end());
    const std::unordered_set<image_t> images2(cluster2.image_ids.begin(),
                                              cluster2.image_ids.end());

    const image_t added_image1 =
        images1.find(lost_edges[k].src) == images1.end() ? lost_edges[k].src
                                                         : lost_edges[k].dst;
    const image_t added_image2 =
        images2.find(lost_edges[k].src) == images2.end() ? lost_edges[k].src
                                                         : lost_edges[k].dst;

    // Select a cluster that has a smaller size, such that
    // larger clusters can avoid become too large.
    int selected_image =
        cluster1.image_ids.size() > cluster2.image_ids.size() ? 2 : 1;
  // if(cluster1.cluster_id == 16 && cluster2.cluster_id == 27)
  // {
  //   LOG(INFO)<<"selected_image: "<<selected_image;
  //   LOG(INFO)<<"src,dst: "<<lost_edges[k].src<<", "<<lost_edges[k].dst;
  //   LOG(INFO)<<"ratio satisfy? "<<IsSatisfyCompletenessRatio(cluster1)<<", " <<IsSatisfyCompletenessRatio(cluster2);
  // }
    if (selected_image == 1) {
      if (!IsSatisfyCompletenessRatio(cluster1) &&
          images1.find(added_image1) == images1.end()) {
        cluster1.image_ids.push_back(added_image1);
        cluster1.edges[view_pair] = lost_edges[k].weight;
      }
      else if(!IsSatisfyCompletenessRatio(cluster2) &&
          images2.find(added_image2) == images2.end())
      {
        cluster2.image_ids.push_back(added_image2);
        cluster2.edges[view_pair] = lost_edges[k].weight;
      }
    } else {
      if (!IsSatisfyCompletenessRatio(cluster2) &&
          images2.find(added_image2) == images2.end()) {
        cluster2.image_ids.push_back(added_image2);
        cluster2.edges[view_pair] = lost_edges[k].weight;
      }
      else if (!IsSatisfyCompletenessRatio(cluster1) &&
          images1.find(added_image1) == images1.end()) {
        cluster1.image_ids.push_back(added_image1);
        cluster1.edges[view_pair] = lost_edges[k].weight;
      }
    }
  }
}

void ImageClustering::AnalyzeStatistic() {
  LOG(INFO) << "Analysing Statistics...";
  for (auto& cluster : inter_clusters_) {
    std::sort(cluster.image_ids.begin(), cluster.image_ids.end());    
    summary_.clustered_images_num += cluster.image_ids.size();
    summary_.clustered_edges_num += cluster.edges.size();
  }
}

void ImageClustering::MyOutputClustersExpanded(std::string graph_dir)
{
    // FILE* file = nullptr;
  std::ofstream out_file(graph_dir + "/clusters_expanded.txt");
  if (!out_file.is_open()) {
    LOG(WARNING) << graph_dir << "/clusters_expanded.txt can't be opened!";
    return;
  }
  for (auto& cluster : inter_clusters_) 
  {
    out_file<<"cluster id: "<<cluster.cluster_id<<std::endl;
    for (auto image_id : cluster.image_ids) 
    {
      out_file<<image_id<<std::endl;
    }
  }
  out_file.close();
}

}  // namespace DAGSfM