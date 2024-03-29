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

#include <glog/logging.h>

#include <fstream>

#include "graph/graph.h"
#include "graph/graph_cut.h"
#include "graph/svg_drawer.h"
#include "graph/union_find.h"

namespace DAGSfM {
namespace graph {

template <typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph() {
  size_ = 0;
}

template <typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph(size_t n) {
  size_ = n;
  for (size_t i = 0; i < n; i++)
    degrees_[i] = 0;
}

template <typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph(const Graph<NodeType, EdgeType> &graph) {
  std::unordered_map<size_t, NodeType> nodes = graph.GetNodes();
  std::unordered_map<size_t, EdgeMap> edges = graph.GetEdges();

  for (auto it = nodes.begin(); it != nodes.end(); ++it) {
    this->AddNode(it->second);
  }
  for (auto it = edges.begin(); it != edges.end(); ++it) {
    EdgeMap em = it->second;
    for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
      this->AddEdge(em_it->second);
    }
  }
}

template <typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType> Graph<NodeType, EdgeType>::Clone() const {
  Graph<NodeType, EdgeType> graph(this->size_);

  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    graph.AddNode(it->second);
  }
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    EdgeMap em = it->second;
    for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
      graph.AddEdge(em_it->second);
    }
  }
  return graph;
}

template <typename NodeType, typename EdgeType>
NodeType Graph<NodeType, EdgeType>::GetNode(size_t idx) const {
  if (!HasNode(idx))
    return NodeType();
  return nodes_.at(idx);
}

template <typename NodeType, typename EdgeType>
const std::unordered_map<size_t, NodeType> &
Graph<NodeType, EdgeType>::GetNodes() const {
  return nodes_;
}

template <typename NodeType, typename EdgeType>
size_t Graph<NodeType, EdgeType>::GetNodesNum() const {
  return nodes_.size();
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::HasNode(const size_t &idx) const {
  if (nodes_.find(idx) == nodes_.end())
    return false;
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::AddNode(const NodeType &node) {
  if (HasNode(node.id))
    return false;

  size_++;
  // if (node.idx == -1) node.idx = size_;
  nodes_[node.id] = node;
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::DeleteNode(const size_t &idx) {
  if (!HasNode(idx))
    return false;
  size_--;
  nodes_.erase(idx);
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::RemoveSingletonNodes() {
  // this->CountDegrees();
  for (auto it = degrees_.begin(); it != degrees_.end(); ++it) {
    if (it->second == 0) {
      nodes_.erase(it->first);
      degrees_.erase(it->first);
    }
  }
  return true;
}

template <typename NodeType, typename EdgeType>
std::vector<NodeType> Graph<NodeType, EdgeType>::FindSingletonNodes() {
  std::vector<NodeType> singletonnodes_;
  this->CountOutDegrees();
  this->CountInDegrees();
  this->CountDegrees();
  for (auto it = degrees_.begin(); it != degrees_.end(); ++it) {
    if (it->second == 0) {
      singletonnodes_.push_back(this->GetNode(it->first));
    }
  }
  return singletonnodes_;
}

template <typename NodeType, typename EdgeType>
const std::unordered_map<size_t, std::unordered_map<size_t, EdgeType>> &
Graph<NodeType, EdgeType>::GetEdges() const {
  return edges_;
}

template <typename NodeType, typename EdgeType>
EdgeType Graph<NodeType, EdgeType>::GetEdge(size_t src, size_t dst) const {
  if (!HasEdge(src, dst))
    return EdgeType();
  return edges_.at(src).at(dst);
}

template <typename NodeType, typename EdgeType>
size_t Graph<NodeType, EdgeType>::GetEdgesNum() const {
  size_t sum = 0;
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    EdgeMap em = it->second;
    sum += em.size();
  }
  // std::vector<std::unordered_map<size_t, EdgeType>> edges =
  // this->SerializeEdges(); for (auto em : edges) { sum += em.size(); }
  return sum;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::HasEdge(const size_t &src,
                                        const size_t &dst) const {
  const auto em_ite = edges_.find(src);
  if (em_ite == edges_.end())
    return false;
  EdgeMap em = em_ite->second;
  if (em.find(dst) == em.end())
    return false;
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::AddEdge(const EdgeType &edge) {
  if (HasEdge(edge.src, edge.dst))
    return false;
  if (!HasNode(edge.src))
    this->AddNode(edge.src);
  if (!HasNode(edge.dst))
    this->AddNode(edge.dst);

  auto em_ite = edges_.find(edge.src);
  if (em_ite == edges_.end()) {
    EdgeMap em;
    em.insert(std::make_pair(edge.dst, edge));
    edges_.insert(std::make_pair(edge.src, em));
  } else
    em_ite->second.insert(std::make_pair(edge.dst, edge));
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::AlterEdge(const EdgeType &edge) {
  if (!HasEdge(edge.src, edge.dst))
    return false;
  if (!HasNode(edge.src) || !HasNode(edge.dst))
    return false;

  edges_.at(edge.src).at(edge.dst) = edge;
  return true;
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::AddUEdge(const EdgeType &edge,
                                         const EdgeType &rev_edge) {
  return this->AddEdge(edge) && this->AddEdge(rev_edge);
}

template <typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::DeleteEdge(const size_t &src,
                                           const size_t &dst) {
  if (!HasEdge(src, dst))
    return false;
  auto em_ite = edges_.find(src);
  em_ite->second.erase(em_ite->second.find(dst));
  if (edges_[src].empty())
    edges_.erase(em_ite);
  return true;
}

template <typename NodeType, typename EdgeType>
EdgeType Graph<NodeType, EdgeType>::FindConnectedEdge(const int &idx) const {
  EdgeType edge;
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    auto em = it->second;
    bool find = false;
    for (auto em_it = em.begin(); em_it != em.end(); em_it++) {
      if (static_cast<int>(em_it->second.src) == idx ||
          static_cast<int>(em_it->first) == idx) {
        edge = em_it->second;
        find = true;
        break;
      }
    }
    if (find) {
      break;
    }
  }
  return edge;
}


template <typename NodeType, typename EdgeType>
std::vector<size_t> Graph<NodeType, EdgeType>::ReturnAllConnectedNodes(const int &idx) {
  std::vector<std::pair<size_t,float>>connected_nodes_weights;
  std::vector<size_t>connected_nodes;

  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    auto em = it->second;
    if(static_cast<int>(it->first) == idx)
    {
      for(auto em_it = em.begin(); em_it != em.end(); em_it++){
        connected_nodes_weights.push_back(std::make_pair(em_it->first,em_it->second.weight));
      }
    }
    else
    {
      for (auto em_it = em.begin(); em_it != em.end(); em_it++) {
        if (static_cast<int>(em_it->first) == idx) {
          connected_nodes_weights.push_back(std::make_pair(it->first,em_it->second.weight));
        }
      }
    }
  }
  const auto cmp = [](const std::pair<size_t,float>pair1,const std::pair<size_t,float>pair2){
    return pair1.second<pair2.second;
  };
  sort(connected_nodes_weights.begin(),connected_nodes_weights.end(),cmp);
  for(int i=0; i<connected_nodes_weights.size(); i++)
  {
    connected_nodes.push_back(connected_nodes_weights[i].first);
  }
  return connected_nodes;
}


template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::FindBestEdgeToMerge(const std::set<size_t> connected_node_ids, int& src, int& dst)
{
  double max_metric = 0.0;
  size_t max_src;
  size_t max_dst;
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    auto em = it->second;
    if(connected_node_ids.count(it->first)==0)
      continue;
    for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
      if(connected_node_ids.count(em_it->first)==0)
        continue;
      double metric = em_it->second.weight;
      if(metric>max_metric)
      {
        max_src = em_it->second.src;
        max_dst = em_it->second.dst;
        max_metric = metric;
      }
      // uf.Union(em_it->second.src, em_it->second.dst);
    }
  }
  LOG(INFO)<<"best edge: ("<<max_src<<", "<<max_dst<<")  weight: "<<max_metric;
  src = max_src;
  dst = max_dst;
}



template <typename NodeType, typename EdgeType>
size_t Graph<NodeType, EdgeType>::GetSize() const {
  return nodes_.size();
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::CountDegrees() {
  // this->CountInDegrees();
  // this->CountOutDegrees();
  degrees_.clear();
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    size_t id = it->second.id;
    degrees_[it->second.id] = in_degrees_[id] + out_degrees_[id];
  }
}

template <typename NodeType, typename EdgeType>
const std::unordered_map<size_t, size_t> &
Graph<NodeType, EdgeType>::GetDegrees() const {
  return degrees_;
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::CountOutDegrees() {
  out_degrees_.clear();
  for (auto ite = edges_.begin(); ite != edges_.end(); ++ite) {
    EdgeMap em = ite->second;
    out_degrees_[ite->first] = em.size();
  }
}

template <typename NodeType, typename EdgeType>
const std::unordered_map<size_t, size_t> &
Graph<NodeType, EdgeType>::GetOutDegrees() const {
  return out_degrees_;
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::CountInDegrees() {
  in_degrees_.clear();
  // initializing degree before using
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    in_degrees_[it->second.id] = 0;
  }

  for (auto ite = edges_.begin(); ite != edges_.end(); ++ite) {
    EdgeMap em = ite->second;
    for (auto edge_ite = em.begin(); edge_ite != em.end(); ++edge_ite) {
      in_degrees_[edge_ite->first]++;
    }
  }
}

template <typename NodeType, typename EdgeType>
const std::unordered_map<size_t, size_t> &
Graph<NodeType, EdgeType>::GetInDegrees() const {
  return out_degrees_;
}

template <typename NodeType, typename EdgeType>
int Graph<NodeType, EdgeType>::FindLeafNode(
    const std::unordered_map<size_t, size_t> &degrees) const {
  // Finding nodes with degree equals 1
  int idx = -1;
  for (auto ite = degrees.begin(); ite != degrees.end(); ++ite) {
    if (ite->second == 1) {
      idx = ite->first;
      break;
    }
  }
  return idx;
}

template <typename NodeType, typename EdgeType>
std::vector<EdgeType> Graph<NodeType, EdgeType>::Kruskal() const {
  std::vector<EdgeType> mstedges_;
  SmallerEdgePriorityQueue<EdgeType> edges = this->CollectEdges();

  std::vector<size_t> nodes;
  nodes.reserve(nodes_.size());
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    nodes.push_back(it->second.id);
  }
  std::sort(nodes.begin(), nodes.end());

  UnionFind union_find(nodes_.size());
  union_find.InitWithNodes(nodes);

  while (!edges.empty()) {
    EdgeType edge = edges.top();
    edges.pop();
    size_t src = edge.src, dst = edge.dst;
    if (union_find.FindRoot(src) != union_find.FindRoot(dst)) {
      mstedges_.push_back(edge);
      union_find.Union(src, dst);
    }
  }
  return mstedges_;
}


template <typename NodeType, typename EdgeType>
std::vector<EdgeType> Graph<NodeType, EdgeType>::MaxstKruskal() const {
  std::vector<EdgeType> mstedges_;
  LargerEdgePriorityQueue<EdgeType> edges = this->MaxstCollectEdges();

  std::vector<size_t> nodes;
  nodes.reserve(nodes_.size());
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    nodes.push_back(it->second.id);
  }
  std::sort(nodes.begin(), nodes.end());

  UnionFind union_find(nodes_.size());
  union_find.InitWithNodes(nodes);

  while (!edges.empty()) {
    EdgeType edge = edges.top();
    edges.pop();
    size_t src = edge.src, dst = edge.dst;
    if (union_find.FindRoot(src) != union_find.FindRoot(dst)) {
      mstedges_.push_back(edge);
      union_find.Union(src, dst);
    }
  }
  return mstedges_;
}

template <typename NodeType, typename EdgeType>
std::vector<EdgeType>
Graph<NodeType, EdgeType>::ShortestPath(const size_t &src,
                                        const size_t &dst) const {
  std::vector<EdgeType> paths;
  std::queue<size_t> qu;
  std::unordered_map<size_t, int> parents;
  std::unordered_map<size_t, bool> visited;

  std::unordered_map<size_t, EdgeMap> edges;
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    EdgeMap em = it->second;
    for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
      edges[it->first].insert(std::make_pair(em_it->first, em_it->second));
      EdgeType edge(em_it->second.dst, em_it->second.src, em_it->second.weight);
      edges[edge.src].insert(std::make_pair(edge.dst, edge));
    }
  }

  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    visited.insert(std::make_pair(it->second.id, false));
  }
  qu.push(src);
  parents[src] = -1;
  visited[src] = true;

  while (!qu.empty()) {
    size_t cur_id = qu.front();
    qu.pop();
    if (cur_id == dst) { // arrive destination
      int id = cur_id;
      std::cout << "Shortest Path(BFS): " << id << "->";
      while (parents[id] != -1) {
        // #ifdef __DEBUG__
        std::cout << parents[id] << "->";
        // #endif
        EdgeType edge = edges.at(parents[id]).at(id);
        paths.push_back(edge);
        id = parents[id];
      }
      std::cout << dst << std::endl;
      break;
    }

    EdgeMap em = edges.at(cur_id);
    for (auto it = em.begin(); it != em.end(); ++it) {
      if (!visited[it->first]) {
        visited[it->first] = true;
        qu.push(it->first);
        parents[it->first] = cur_id;
      }
    }
  }
  std::reverse(paths.begin(), paths.end());
  return paths;
}

template <typename NodeType, typename EdgeType>
std::unordered_map<int, int>
Graph<NodeType, EdgeType>::NormalizedCut(const size_t cluster_num) const {
  std::vector<std::pair<int, int>> edges;
  std::vector<int> weights;

  for (auto em_it = edges_.begin(); em_it != edges_.end(); ++em_it) {
    EdgeMap em = em_it->second;
    for (auto it = em.begin(); it != em.end(); ++it) {
      edges.emplace_back(it->second.src, it->second.dst);
      weights.push_back(it->second.weight);
    }
  }

  return ComputeNormalizedMinGraphCut(edges, weights, cluster_num);
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::ShowInfo() const {
  std::vector<NodeType> nodes = this->SerializeNodes();
  // std::vector<EdgeMap> edges = this->SerializeEdges();

  LOG(INFO) << "[Graph Info]\n";
  LOG(INFO) << "Total nodes: " << std::to_string(GetNodesNum());
  LOG(INFO) << "\nTotal edges: " << std::to_string(GetEdgesNum());
  LOG(INFO) << "\n[Node]: \n";
  for (uint i = 0; i < nodes.size(); i++) {
    std::cout << nodes[i].id << " ";
  }
  LOG(INFO) << "\n[Edge]: \n";
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    auto em = it->second;
    if (em.size() == 0)
      continue;
    for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
      std::cout << "(" << em_it->second.src << ", " << em_it->second.dst
                << ") ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  // for (int i = 0; i < edges.size(); i++) {
  // if (edges[i].empty()) continue;
  // for (auto it = edges[i].begin(); it != edges[i].end(); ++it) {
  // std::cout << "(" << it->second.src << ", " << it->second.dst << ") ";
  // }
  // std::cout << std::endl;
  // }
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::ShowInfo(const std::string &filename) const {
  std::ofstream out(filename);
  if (!out.is_open()) {
    std::cout << filename << " cannot be opened!\n";
    return;
  }

  std::vector<NodeType> nodes = this->SerializeNodes();
  // std::vector<EdgeMap> edges = this->SerializeEdges();
  // auto edges = this->GetEdges();

  out << "[Graph Info]\n";
  out << "Total nodes: " + std::to_string(GetNodesNum());
  out << "\nTotal edges: " + std::to_string(GetEdgesNum());
  out << "\n[Node]: \n";
  for (int i = 0; i < nodes.size(); i++) {
    out << nodes[i].id << " ";
  }

  out << "\n[Edge]: \n";
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    auto em = it->second;
    if (em.size() == 0)
      continue;
    for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
      out << "(" << em_it->second.src << ", " << em_it->second.dst << ") ";
    }
    out << std::endl;
  }
  // for (int i = 0; i < edges.size(); i++) {
  // if (edges[i].empty()) continue;
  // for (auto it = edges[i].begin(); it != edges[i].end(); ++it) {
  // out << "(" << it->second.src << ", " << it->second.dst << ") ";
  // }
  // out << std::endl;
  // }
}

template <typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType> Graph<NodeType, EdgeType>::ExtractLargestCC() const {
  graph::UnionFind uf(nodes_.size());

  std::vector<size_t> node_ids;
  node_ids.reserve(nodes_.size());
  for (auto node_it : nodes_) {
    node_ids.push_back(node_it.first);
  }
  uf.InitWithNodes(node_ids);

  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    auto em = it->second;
    for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
      uf.Union(em_it->second.src, em_it->second.dst);
    }
  }

  std::unordered_map<size_t, std::unordered_set<size_t>> components;
  for (auto node_id : node_ids) {
    const size_t parent_id = uf.FindRoot(node_id);
    components[parent_id].insert(node_id);
  }

  size_t num_largest_component = 0;
  size_t largest_component_id;
  for (const auto &it : components) {
    if (num_largest_component < it.second.size()) {
      num_largest_component = it.second.size();
      largest_component_id = it.first;
    }
  }

  Graph<NodeType, EdgeType> largest_cc;
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    auto em = it->second;
    for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
      if (components[largest_component_id].count(em_it->second.src) == 0 ||
          components[largest_component_id].count(em_it->second.dst) == 0) {
        continue;
      }
      largest_cc.AddEdge(em_it->second);
    }
  }

  return largest_cc;
}

template <typename NodeType, typename EdgeType>
std::unordered_map<size_t, std::unordered_set<size_t>>
Graph<NodeType, EdgeType>::FindConnectedComponents() const {
  graph::UnionFind uf(nodes_.size());

  std::vector<size_t> node_ids;
  node_ids.reserve(nodes_.size());
  for (auto node_it : nodes_) {
    node_ids.push_back(node_it.first);
  }
  uf.InitWithNodes(node_ids);

  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    auto em = it->second;
    for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
      uf.Union(em_it->second.src, em_it->second.dst);
    }
  }

  std::unordered_map<size_t, std::unordered_set<size_t>> components;
  for (auto node_id : node_ids) {
    const size_t parent_id = uf.FindRoot(node_id);
    components[parent_id].insert(node_id);
  }

  return components;
}

template <typename NodeType, typename EdgeType>
SmallerEdgePriorityQueue<EdgeType>
Graph<NodeType, EdgeType>::CollectEdges() const {
  SmallerEdgePriorityQueue<EdgeType> edges;
  for (auto ite = edges_.begin(); ite != edges_.end(); ++ite) {
    EdgeMap em = ite->second;
    for (auto edge_ite = em.begin(); edge_ite != em.end(); ++edge_ite) {
      edges.push(edge_ite->second);
    }
  }
  return edges;
}


template <typename NodeType, typename EdgeType>
LargerEdgePriorityQueue<EdgeType>
Graph<NodeType, EdgeType>::MaxstCollectEdges() const {
  LargerEdgePriorityQueue<EdgeType> edges;
  for (auto ite = edges_.begin(); ite != edges_.end(); ++ite) {
    EdgeMap em = ite->second;
    for (auto edge_ite = em.begin(); edge_ite != em.end(); ++edge_ite) {
      edges.push(edge_ite->second);
    }
  }
  return edges;
}

template <typename NodeType, typename EdgeType>
std::vector<NodeType> Graph<NodeType, EdgeType>::SerializeNodes() const {
  std::vector<NodeType> nodes;
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    nodes.push_back(it->second);
  }
  std::sort(nodes.begin(), nodes.end(), Node::CompareById);
  return nodes;
}

template <typename NodeType, typename EdgeType>
std::vector<std::unordered_map<size_t, EdgeType>>
Graph<NodeType, EdgeType>::SerializeEdges() const {
  std::vector<std::unordered_map<size_t, EdgeType>> edges;
  edges.resize(nodes_.size());
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    EdgeMap em = it->second;
    for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
      edges[it->first].insert(std::make_pair(em_it->first, em_it->second));
      EdgeType edge(em_it->second.dst, em_it->second.src, em_it->second.weight);
      edges[edge.src].insert(std::make_pair(edge.dst, edge));
    }
  }
  return edges;
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::UpdateGraph() {
  std::vector<NodeType> serialnodes_ = this->SerializeNodes();
  std::vector<EdgeMap> serialedges_ = this->SerializeEdges();

  // adjust the id of node
  for (size_t i = 0; i < serialnodes_.size(); i++) {
    size_t ori_id = serialnodes_[i].id;
    nodes_[ori_id].id = i;
  }

  // adjust the id and index of edges
  for (auto &it = edges_.begin(); it != edges_.end(); ++it) {
    size_t src = it->first;
    EdgeMap &em = it->second;
    for (auto &em_it = em.begin(); em_it != em.end(); ++em_it) {
      size_t dst = em_it->first;
      EdgeType &edge = em_it->second;
      edge.src = nodes_[src].id;
      edge.dst = nodes_[dst].id;
      em_it->first = nodes_[dst].id;
    }
    it->first = nodes_[src].id;
  }

  // re-adjust the index of nodes
  nodes_.clear();
  for (size_t i = 0; i < serialnodes_.size(); i++) {
    size_t ori_id = serialnodes_[i].id;
    nodes_[ori_id].id = i;
  }
}

template <typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::OutputSVG(const std::string &filename) const {
  const float scale_factor = 5.0f;
  const size_t image_num = nodes_.size();

  SvgDrawer svg_drawer((image_num + 3) * 5, (image_num + 3) * 5);

  // Draw rectangles for all image pairs.
  for (const auto &it : edges_) {
    const size_t i = it.first;
    const EdgeMap &em = it.second;

    for (auto &em_it : em) {
      const size_t j = em_it.first;
      const float score = em_it.second.weight;

      std::ostringstream os_color;
      os_color << "rgb(" << 0 << "," << 0 << "," << 255 << ")";

      std::ostringstream os_tooltip;
      os_tooltip << "(" << j << "," << i << " " << score << ")";
      svg_drawer.DrawSquare(
          j * scale_factor, i * scale_factor, scale_factor / 2.0f,
          SvgStyle().Fill(os_color.str()).NoStroke().ToolTip(os_tooltip.str()));

      os_tooltip.clear();
      os_tooltip << "(" << i << "," << j << " " << score << ")";
      svg_drawer.DrawSquare(
          i * scale_factor, j * scale_factor, scale_factor / 2.0f,
          SvgStyle().Fill(os_color.str()).NoStroke().ToolTip(os_tooltip.str()));
    }
  }

  // Display axes with 0-> image_num annotation
  std::ostringstream os_num_images;
  os_num_images << image_num;
  svg_drawer.DrawText((image_num + 1) * scale_factor, scale_factor,
                      scale_factor, "0", "black");
  svg_drawer.DrawText((image_num + 1) * scale_factor,
                      (image_num - 1) * scale_factor, scale_factor,
                      os_num_images.str(), "black");
  svg_drawer.DrawLine((image_num + 1) * scale_factor, 2 * scale_factor,
                      (image_num + 1) * scale_factor,
                      (image_num - 2) * scale_factor,
                      SvgStyle().Stroke("black", 1.0));
  svg_drawer.DrawText(scale_factor, (image_num + 1) * scale_factor,
                      scale_factor, "0", "black");
  svg_drawer.DrawText((image_num - 1) * scale_factor,
                      (image_num + 1) * scale_factor, scale_factor,
                      os_num_images.str(), "black");
  svg_drawer.DrawLine(2 * scale_factor, (image_num + 1) * scale_factor,
                      (image_num - 2) * scale_factor,
                      (image_num + 1) * scale_factor,
                      SvgStyle().Stroke("black", 1.0));

  std::ofstream svg_ofs(filename);
  svg_ofs << svg_drawer.CloseSvgFile().str();
}




} // namespace graph
} // namespace DAGSfM