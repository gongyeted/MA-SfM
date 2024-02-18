#include "clustering/metric_cluster.h"
#include "graph/image_graph.h"

#include <Spectra/SymEigsSolver.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>

#include "clustering/kmeans.h"
// #include <Spectra/GenEigsSolver.h>
#include <Spectra/MatOp/SparseSymMatProd.h>
// #include <Spectra/MatOp/SparseGenMatProd.h>
#include <glog/logging.h>

#include "util/timer.h"

namespace DAGSfM {

std::unordered_map<int, int> MetricCluster::ComputeCluster(
    const std::vector<image_t>& image_ids,
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<int>& weights) {


    graph::UnionFind uf(image_ids.size());
    std::vector<size_t> tmp_nodes(image_ids.begin(), image_ids.end());
    uf.InitWithNodes(tmp_nodes);

    for (auto image_pair : edges) {
        uf.Union(image_pair.first, image_pair.second);
    }

    std::unordered_map<size_t, std::vector<image_t>> components;
    for (auto image_id : image_ids) {
        const size_t parent_id = uf.FindRoot(image_id);
        components[parent_id].push_back(image_id);
    }

    LOG(INFO) << "There are " << components.size() << " connected components.";
    cluster_num_ = components.size();
    size_t cluster_id = 0;
    for(auto component : components)
    {
        // size_t cluster_id = component.first;
        std::vector<image_t> img_ids = component.second;
        for(int i = 0; i<img_ids.size(); i++)
        {
            image_t img_id = img_ids[i];
            labels_[img_id]=cluster_id;
        }
        LOG(INFO) << "Component #" << component.first << "# has "
            << component.second.size() << " images.";
        cluster_id++;
    }
    return labels_;
}

std::unordered_map<int, int> MetricCluster::ComputeCluster(
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<int>& weights, const int num_partitions) {
  return labels_;
}

// Eigen::SparseMatrix<double> MetricCluster::ComputeLaplacian(
//     const Eigen::SparseMatrix<double>& S,
//     const std::unordered_map<int, int>& degrees) const {
//   // Compute degree matrix.
//   const int N = degrees.size();
//   Eigen::SparseMatrix<double> D(N, N);
//   // Eigen::SparseMatrix<double> D_inv(N, N);
//   // Eigen::SparseMatrix<double> D_sqrt(N, N);
//   for (auto it = degrees.begin(); it != degrees.end(); ++it) {
//     int id = it->first;
//     D.insert(id, id) = it->second;
//     // D.insert(node_mapper_.at(id), node_mapper_.at(id)) = it->second;
//     // D_inv.insert(node_mapper_.at(id), node_mapper_.at(id)) = 1.0 /
//     // it->second; D_sqrt.insert(node_mapper_.at(id), node_mapper_.at(id)) =
//     // -1.0 / sqrt(it->second);
//   }
//   D.makeCompressed();
//   // D_inv.makeCompressed();
//   // D_sqrt.makeCompressed();

//   // Compute Laplacian matrix.
//   Eigen::SparseMatrix<double> L = D - S;
//   // Eigen::MatrixXd L_random = D_inv * L;
//   // Eigen::MatrixXd L_sym = D_sqrt * L * D_sqrt;

//   return L;
// }

}  // namespace DAGSfM