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

#include "clustering/kmeans_cluster.h"

namespace DAGSfM {

std::unordered_map<int, int> KMeansCluster::ComputeCluster(
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<int>& weights, const int num_partitions) {

  LOG(INFO)<<"KMeans Clustering...(todo)";
  LOG(INFO)<<"Here is ncut from METIS";

  //get xadj, adjncy, adjwgt;
  std::unordered_map<int, std::vector<std::pair<int, int>>> adjacency_list;
  for (size_t i = 0; i < edges.size(); ++i) {
    const auto& edge = edges[i];
    const auto weight = weights[i];
    const int vertex_idx1 = GetVertexIdx(edge.first);
    const int vertex_idx2 = GetVertexIdx(edge.second);
    adjacency_list[vertex_idx1].emplace_back(vertex_idx2, weight);
    adjacency_list[vertex_idx2].emplace_back(vertex_idx1, weight);
  }

  LOG(INFO)<<"vertex_id_to_idx_ size: "<<vertex_id_to_idx_.size();

  xadj_.reserve(vertex_id_to_idx_.size() + 1);
  adjncy_.reserve(2 * edges.size());
  adjwgt_.reserve(2 * edges.size());

  idx_t edge_idx = 0;
  for (size_t i = 0; i < vertex_id_to_idx_.size(); ++i) {
    xadj_.push_back(edge_idx);

    if (adjacency_list.count(i) == 0) {
      continue;
    }

    for (const auto& edge : adjacency_list[i]) {
      edge_idx += 1;
      adjncy_.push_back(edge.first);
      adjwgt_.push_back(edge.second);
    }
  }

  xadj_.push_back(edge_idx);

  CHECK_EQ(edge_idx, 2 * edges.size());
  CHECK_EQ(xadj_.size(), vertex_id_to_idx_.size() + 1);
  CHECK_EQ(adjncy_.size(), 2 * edges.size());
  CHECK_EQ(adjwgt_.size(), 2 * edges.size());

  // LOG(INFO)<<"CHECK finished";

  idx_t nVertices = xadj_.size() - 1; // 节点数
	idx_t nEdges = adjncy_.size() / 2;    // 边数
	idx_t nWeights = 1;
	idx_t nParts = num_partitions; //2;   // 子图个数
	idx_t objval;
	std::vector<idx_t> part(nVertices, 0);

  LOG(INFO)<<"nVertices: "<<nVertices;

	int ret = METIS_PartGraphKwayReplace(&nVertices, &nWeights, xadj_.data(), adjncy_.data(),
		adjwgt_.data(), NULL, NULL, &nParts, NULL,
		NULL, NULL, &objval, part.data());

  CHECK_EQ(nParts, num_partitions);


  

	// LOG(INFO) << ret;

	// for (unsigned part_i = 0; part_i < part.size(); part_i++) {
	// 	LOG(INFO) << part_i << " " << part[part_i];
	// }



  std::unordered_map<int,int> part_id_to_cluster_id;
  int cluster_id_count = 0;
  std::unordered_map<int, int> labels;
  for (unsigned part_i = 0; part_i < part.size(); part_i++) {
    int part_id = part[part_i];
    if(part_id_to_cluster_id.count(part_id)==0) //尚未创建对应的cluster_id
    {
      part_id_to_cluster_id.insert(std::make_pair(part_id, cluster_id_count));
      labels.insert(std::make_pair(vertex_idx_to_id_[part_i],cluster_id_count));
      cluster_id_count++;
    }
    else //之前已创建过该part对应的cluster_id
    {
      labels.insert(std::make_pair(vertex_idx_to_id_[part_i],part_id_to_cluster_id[part_id]));
    }
    // labels.insert(std::make_pair(vertex_idx_to_id_[part_i],part[part_i]));
	}

  cluster_num_ = cluster_id_count;
  
  // std::ofstream labels_out("/16t/gy/labels_inside1.txt");
  // for(auto label : labels)  
  //   labels_out<<label.first<<"\t"<<label.second<<std::endl;
  // labels_out.close();




// std::unordered_map<int,int> cluster_id_img_nums;

// 	for(auto label : labels)
// 	{
// 		int cluster_id = label.second;
// 		if(cluster_id_img_nums.count(cluster_id)==0)
// 			cluster_id_img_nums.insert(std::make_pair(cluster_id,1));
// 		else
// 			cluster_id_img_nums[cluster_id]++;
// 	}

// 	int max_cluster_size = 0;
//     int min_cluster_size = 1000;
//     int max_cluster_id = 0;
//     int min_cluster_id = 0;

// 	for(auto itr:cluster_id_img_nums)
// 	{
// 		if(itr.second>max_cluster_size)
// 		{
// 			max_cluster_size = itr.second;
// 			max_cluster_id = itr.first;
// 		}
// 		if(itr.second<min_cluster_size)
// 		{
// 			min_cluster_size = itr.second;
// 			min_cluster_id = itr.first;
// 		}
// 	}
//   LOG(INFO)<<"inside kmeans";
// 	LOG(INFO)<<"the cluster "<<max_cluster_id<<" has max "<<max_cluster_size<< " subclusters";
//   LOG(INFO)<<"the cluster "<<min_cluster_id<<" has min "<<min_cluster_size<< " subclusters";



  return labels;
}

  std::unordered_map<int, int> KMeansCluster::ComputeCluster(
          const std::vector<image_t>& image_ids,
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<int>& weights) 
    {
      return {};
    }//my code



  int KMeansCluster::GetVertexIdx(const int id) {
    const auto it = vertex_id_to_idx_.find(id);
    if (it == vertex_id_to_idx_.end()) {
      const int idx = vertex_id_to_idx_.size();
      vertex_id_to_idx_.emplace(id, idx);
      vertex_idx_to_id_.emplace(idx, id);
      return idx;
    } else {
      return it->second;
    }
  }

}  // namespace DAGSfM