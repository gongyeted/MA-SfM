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

#include "clustering/cluster.h"

#include <fstream>

namespace DAGSfM {

Cluster::Cluster() {
  igraph_vector_init(&i_edges_, 0);
  igraph_vector_init(&i_weights_, 0);
  cluster_num_ = 1;
}

Cluster::~Cluster() {
  igraph_destroy(&igraph_);
  igraph_vector_destroy(&i_edges_);
  igraph_vector_destroy(&i_weights_);
}

int Cluster::ClusterNum() const { return cluster_num_; }

bool Cluster::InitIGraph(const std::vector<std::pair<int, int>>& edges,
                         const std::vector<int>& weights) {
  // std::vector<int> nodes;
  std::unordered_map<int, int> node_mapper;
  for (uint i = 0; i < edges.size(); i++) {
    auto edge = edges[i];
    nodes_.push_back(edge.first);
    nodes_.push_back(edge.second);
  }

  std::sort(nodes_.begin(), nodes_.end());
  nodes_.erase(std::unique(nodes_.begin(), nodes_.end()), nodes_.end());

  const int n = nodes_.size();
  for (uint i = 0; i < n; i++) {
    node_mapper[nodes_[i]] = i;
  }

  // Filling edges in igraph data format.
  for (uint i = 0; i < edges.size(); i++) {
    auto edge = edges[i];
    igraph_vector_push_back(&i_edges_, node_mapper[edge.first]);
    igraph_vector_push_back(&i_edges_, node_mapper[edge.second]);
    igraph_vector_push_back(&i_weights_, weights[i]);
  }

  // Filling graph in igraph data format.
  igraph_create(&igraph_, &i_edges_, (igraph_integer_t)n, IGRAPH_UNDIRECTED);

  return true;
}



bool Cluster::InitIGraphWithID(const std::vector<std::pair<int, int>>& edges,
                         const std::vector<int>& weights) {
  // std::vector<int> nodes;
  std::unordered_map<int, int> node_mapper;
  for (uint i = 0; i < edges.size(); i++) {
    auto edge = edges[i];
    nodes_.push_back(edge.first);
    nodes_.push_back(edge.second);
  }

  std::sort(nodes_.begin(), nodes_.end());
  nodes_.erase(std::unique(nodes_.begin(), nodes_.end()), nodes_.end());


  const int n = nodes_.size();
  for (uint i = 0; i < n; i++) {
    node_mapper[nodes_[i]] = i;
    // node_mapper[nodes_[i]] = nodes_[i];
  }

  // Filling edges in igraph data format.
  for (uint i = 0; i < edges.size(); i++) {
    auto edge = edges[i];
    // igraph_vector_push_back(&i_edges_, node_mapper[edge.first]);
    // igraph_vector_push_back(&i_edges_, node_mapper[edge.second]);
      igraph_vector_push_back(&i_edges_, edge.first);
    igraph_vector_push_back(&i_edges_, edge.second);
    igraph_vector_push_back(&i_weights_, weights[i]);
  }

  // Filling graph in igraph data format.
  igraph_create(&igraph_, &i_edges_, (igraph_integer_t)n, IGRAPH_UNDIRECTED);

  return true;
}

bool Cluster::OutputIGraph(const std::string graph_dir,
                           const std::string image_path) const {
  // Write graph in gml format.
  FILE* file = nullptr;
  file = fopen((graph_dir + "/gml.txt").c_str(), "w");
  // igraph_write_graph_dot(&g, stdout);
  igraph_write_graph_gml(&igraph_, file, 0, "graph in cluster");
  fclose(file);

  std::ofstream out_file(graph_dir + "/labels.txt");
  if (!out_file.is_open()) {
    LOG(WARNING) << graph_dir << "/labels.txt can't be opened!";
    return false;
  }
  for (auto node : nodes_) {
    out_file << labels_.at(node) << " ";
  }
  out_file.close();

  // // Invoke python module.
  // Py_Initialize();

  // if (!Py_IsInitialized()) {
  //     LOG(ERROR) << "initialized error";
  //     return false;
  // }

  // PyObject* draw_graph_module(0);
  // draw_graph_module = PyImport_ImportModule("read_igraph.py");

  // if (!draw_graph_module) {
  //     PyErr_Print();
  //     LOG(WARNING) << "can not find read_igraph.py";
  //     return false;
  // } else {
  //     LOG(INFO) << "open Module";
  // }

  // PyObject *pDict = PyModule_GetDict(draw_graph_module);
  // PyObject *pFunc = PyDict_GetItemString(pDict, "drawIGraph");

  // PyObject* args = PyTuple_New(3);
  // PyObject* arg1 = Py_BuildValue("s", file_path.c_str());
  // PyObject* arg2 = Py_BuildValue("s", image_path.c_str());
  // PyTuple_SetItem(args, 0, arg1);
  // PyTuple_SetItem(args, 1, arg2);

  // PyObject_CallObject(pFunc, args);
  // PyErr_Print();

  // Py_DECREF(pFunc);
  // Py_DECREF(pDict);
  // Py_DECREF(draw_graph_module);
  // Py_Finalize();

  return true;
}


bool Cluster::OutputIGraphInfo(const std::vector<std::pair<int, int>> edges,
                    const std::vector<int> weights,
                    const std::string graph_dir) const {
  std::vector<int> nodes;
  for (uint i = 0; i < edges.size(); i++) {
    auto edge = edges[i];
    nodes.push_back(edge.first);
    nodes.push_back(edge.second);
  }
  
  std::sort(nodes.begin(), nodes.end());
  nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
  
  
  // Write graph in gml format.
  std::ofstream gml_out(graph_dir + "/gml.txt");
  gml_out<<"Creator Gongye"<<std::endl;
  gml_out<<"Version Gongye"<<std::endl;
  gml_out<<"graph"<<std::endl;
  gml_out<<"["<<std::endl;
  gml_out<<"\t"<<"directed 0"<<std::endl;
  //write nodes
  for(int i=0; i<nodes.size(); i++)
  {
    gml_out<<"\t"<<"node"<<std::endl;
    gml_out<<"\t"<<"["<<std::endl;
    gml_out<<"\t\t"<<"id "<<nodes[i]<<std::endl;
    gml_out<<"\t"<<"]"<<std::endl;
  }
  //write edges
  for(int i=0; i<edges.size(); i++)
  {
    gml_out<<"\t"<<"edge"<<std::endl;
    gml_out<<"\t"<<"["<<std::endl;
    gml_out<<"\t\t"<<"source "<<edges[i].first<<std::endl;
    gml_out<<"\t\t"<<"target "<<edges[i].second<<std::endl;
    gml_out<<"\t\t"<<"weight "<<weights[i]<<std::endl;
    gml_out<<"\t"<<"]"<<std::endl;
  }
  gml_out<<"]"<<std::endl;
  gml_out.close();

  // FILE* file = nullptr;
  // file = fopen((graph_dir + "/gml.txt").c_str(), "w");
  // // igraph_write_graph_dot(&g, stdout);
  // igraph_write_graph_gml(&igraph_, file, 0, "graph in cluster");
  // fclose(file);

  std::ofstream label_out(graph_dir + "/labels.txt");
  if (!label_out.is_open()) {
    LOG(WARNING) << graph_dir << "/labels.txt can't be opened!";
    return false;
  }
  for (auto node : nodes) {
    label_out << 0 << " ";
  }
  label_out.close();

  return true;
}



bool Cluster::OutputGraphJson(const std::vector<std::pair<int, int>> edges,
                    const std::vector<int> weights,
                    const std::string graph_dir) const {
  std::vector<int> nodes;
  for (uint i = 0; i < edges.size(); i++) {
    auto edge = edges[i];
    nodes.push_back(edge.first);
    nodes.push_back(edge.second);
  }

  std::sort(nodes.begin(), nodes.end());
  nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
  LOG(INFO)<<"nodes size when OUTPUT: "<<nodes.size();
  
  // Write graph in gml format.
  std::ofstream gml_out(graph_dir + "/gml.json");
  gml_out<<"{"<<std::endl;
  gml_out<<"\t"<<"\"nodes\": ["<<std::endl;
  //write nodes
  for(int i=0; i<nodes.size(); i++)
  {
    gml_out<<"\t  {"<<std::endl;
    gml_out<<"\t\t"<<"\"id\": "<<"\""<<nodes[i]<<"\","<<std::endl;
    gml_out<<"\t\t"<<"\"name\": "<<"\""<<nodes[i]<<"\""<<std::endl;
    if(i!=nodes.size()-1)
      gml_out<<"\t  },"<<std::endl;
    else
      gml_out<<"\t  }"<<std::endl;
  }
  gml_out<<"\t],"<<std::endl;
  //write edges
  gml_out<<"\t"<<"\"links\": ["<<std::endl;
  for(int i=0; i<edges.size(); i++)
  {
    gml_out<<"\t  {"<<std::endl;
    gml_out<<"\t\t"<<"\"source\": "<<"\""<<edges[i].first<<"\","<<std::endl;
    gml_out<<"\t\t"<<"\"target\": "<<"\""<<edges[i].second<<"\""<<std::endl;
    // gml_out<<"\t\t"<<"\"weight\": "<<"\""<<weights[i]<<"\","<<std::endl;
    if(i!=edges.size()-1)
      gml_out<<"\t  },"<<std::endl;
    else
      gml_out<<"\t  }"<<std::endl;
  }
  gml_out<<"\t],"<<std::endl;
  //write categories
  gml_out<<"\t"<<"\"categories\": ["<<std::endl;
  // for(int i=0; i<edges.size(); i++)
  // {
    gml_out<<"\t  {"<<std::endl;
    gml_out<<"\t\t"<<"\"name\": "<<"\"A\""<<std::endl;
    // if(i!=nodes.size()-1)
    //   gml_out<<"\t  },"<<std::endl;
    // else
      gml_out<<"\t  }"<<std::endl;
  // }
  gml_out<<"\t]"<<std::endl;
  gml_out<<"}"<<std::endl;
  gml_out.close();

  return true;
}



bool Cluster::OutputGraphJson(const std::vector<std::pair<int, int>> edges,
                    const std::vector<int> weights,
                    const std::unordered_map<int, int> labels,
                    const int cluster_num,
                    const std::string graph_dir) const {
  std::vector<int> nodes;
  for (uint i = 0; i < edges.size(); i++) {
    auto edge = edges[i];
    nodes.push_back(edge.first);
    nodes.push_back(edge.second);
  }

  std::sort(nodes.begin(), nodes.end());
  nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
  
  
  // Write graph in gml format.
  std::ofstream gml_out(graph_dir + "/gml.json");
  gml_out<<"{"<<std::endl;
  gml_out<<"\t"<<"\"nodes\": ["<<std::endl;
  //write nodes
  for(int i=0; i<nodes.size(); i++)
  {
    // int cluster_id = nodes[i];
    // int label = labels[cluster_id];
    gml_out<<"\t  {"<<std::endl;
    gml_out<<"\t\t"<<"\"id\": "<<"\""<<nodes[i]<<"\","<<std::endl;
    gml_out<<"\t\t"<<"\"name\": "<<"\""<<nodes[i]<<"\","<<std::endl;
    gml_out<<"\t\t"<<"\"category\": "<< labels.at(nodes[i]) <<std::endl;
    if(i!=nodes.size()-1)
      gml_out<<"\t  },"<<std::endl;
    else
      gml_out<<"\t  }"<<std::endl;
  }
  gml_out<<"\t],"<<std::endl;
  //write edges
  gml_out<<"\t"<<"\"links\": ["<<std::endl;
  for(int i=0; i<edges.size(); i++)
  {
    gml_out<<"\t  {"<<std::endl;
    gml_out<<"\t\t"<<"\"source\": "<<"\""<<edges[i].first<<"\","<<std::endl;
    gml_out<<"\t\t"<<"\"target\": "<<"\""<<edges[i].second<<"\""<<std::endl;
    // gml_out<<"\t\t"<<"\"weight\": "<<"\""<<weights[i]<<"\","<<std::endl;
    if(i!=edges.size()-1)
      gml_out<<"\t  },"<<std::endl;
    else
      gml_out<<"\t  }"<<std::endl;
  }
  gml_out<<"\t],"<<std::endl;
  //write categories
  gml_out<<"\t"<<"\"categories\": ["<<std::endl;
  for(int i=0; i<cluster_num; i++)
  {
    gml_out<<"\t  {"<<std::endl;
    gml_out<<"\t\t"<<"\"name\": "<<"\""<<i<<"\""<<std::endl;
    if(i!=cluster_num-1)
      gml_out<<"\t  },"<<std::endl;
    else
      gml_out<<"\t  }"<<std::endl;
  }
  gml_out<<"\t]"<<std::endl;
  gml_out<<"}"<<std::endl;
  gml_out.close();

  return true;
}





bool Cluster::MyOutputClusterBeforeExpand(const std::string graph_dir) const {
  // Write graph in gml format.
  FILE* file = nullptr;
  // file = fopen((graph_dir + "/gml.txt").c_str(), "w");
  // // igraph_write_graph_dot(&g, stdout);
  // igraph_write_graph_gml(&igraph_, file, 0, "graph in cluster");
  // fclose(file);

  std::ofstream out_file(graph_dir + "/clusters.txt",std::ios::out);
  if (!out_file.is_open()) {
    LOG(WARNING) << graph_dir << "/clusters.txt can't be opened!";
    return false;
  }
  for (auto node : nodes_) {
    out_file << labels_.at(node) << std::endl;
  }
  out_file.close();


  return true;
}


bool Cluster::MyOutputClusterBeforeExpand(const std::string graph_dir,std::unordered_map<int, int>& labels) const {
  std::ofstream out_file(graph_dir + "/clusters.txt",std::ios::out);
  if (!out_file.is_open()) {
    LOG(WARNING) << graph_dir << "/clusters.txt can't be opened!";
    return false;
  }
  
  // int output_size = 0;
  // std::unordered_map<int,std::vector<int>> all_labels;
  // for (auto label : labels) 
  // {
  //   std::vector<int>& imgids = all_labels[label.second];
  //   imgids.push_back(label.first);
  //   // out_file << label.first <<"\t"<<label.second<< std::endl;
  // }
  // for (auto& label : all_labels)
  // {
  //   sort(label.second);
  // }
  // std::vector<std::vector<int>> all_labels_vec;
  // int cluster_id = 0;
  // while(true)
  // {
  //   if(all_labels.count(cluster_id)!=0)
  //     all_labels_vec.push_back(all_labels[cluster_id]);
  //   if(all_labels_vec.size>=all_labels.size())
  //     break;
  // }

  // for(int i = 0; i<all_labels_vec.size(); i++)
  // {
  //   std::vector<int> labels = all_labels_vec[i];
  //   out_file<<"cluster "<<i<<":"<<std::endl;
  //   for(auto label : labels)
  //   {
  //     out_file<<label
  //   }
  // }

  std::vector<std::pair<int,int>> labels_vec;
  for(auto label : labels)
  {
    labels_vec.push_back(std::make_pair(label.first,label.second));
  }
  const auto cmp = [](const std::pair<int,int>& label1, const std::pair<int,int>& label2) {
    return label1.first < label2.first;
  };
  sort(labels_vec.begin(),labels_vec.end(),cmp);
  for(int i=0; i<labels_vec.size(); i++)
  {
    out_file<<labels_vec[i].first<<"\t"<<labels_vec[i].second<<std::endl;
  }

  out_file.close();
  return true;
}


}  // namespace DAGSfM