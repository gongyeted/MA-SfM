#include "controllers/distributed_mapper_controller.h"

#include <ceres/rotation.h>
#include <omp.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "base/database.h"
#include "base/database_info.h"
#include "base/track_selection.h"
#include "controllers/sfm_aligner.h"
#include "map_reduce/distributed_task_manager.h"
#include "math/util.h"
#include "rotation_estimation/rotation_estimator.h"
#include "sfm/filter_view_pairs_from_orientation.h"
#include "sfm/incremental_triangulator.h"
#include "util/hash.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/reconstruction_io.h"
#include "util/timer.h"

using namespace colmap;

#define __DEBUG__

namespace DAGSfM {
namespace {

void ExtractColors(const std::string& image_path,
                   Reconstruction* reconstruction) {
  const auto image_ids = reconstruction->RegImageIds();
  for (const auto image_id : image_ids) {
    if (!reconstruction->ExtractColorsForImage(image_id, image_path)) {
      std::cout << StringPrintf("WARNING: Could not read image %s at path %s.",
                                reconstruction->Image(image_id).Name().c_str(),
                                image_path.c_str())
                << std::endl;
    }
  }
}

bool VerifyCameraParams(const std::string& camera_model,
                        const std::string& params) {
  if (!ExistsCameraModelWithName(camera_model)) {
    std::cerr << "ERROR: Camera model does not exist" << std::endl;
    return false;
  }

  const std::vector<double> camera_params = CSVToVector<double>(params);
  const int camera_model_id = CameraModelNameToId(camera_model);

  if (camera_params.size() > 0 &&
      !CameraModelVerifyParams(camera_model_id, camera_params)) {
    std::cerr << "ERROR: Invalid camera parameters" << std::endl;
    return false;
  }
  return true;
}
}  // namespace

bool DistributedMapperController::Options::Check() const {
  CHECK_OPTION_GT(num_workers, -1);
  return true;
}

DistributedMapperController::DistributedMapperController(
    const Options& options,
    const VocabSimilaritySearchOptions& similarity_search_options,
    const SiftExtractionOptions& extraction_options,
    const ImageClustering::Options& clustering_options,
    const IncrementalMapperOptions& mapper_options,
    ReconstructionManager* reconstruction_manager)
    : options_(options),
      clustering_options_(clustering_options),
      mapper_options_(mapper_options),
      database_(options_.database_path),
      reconstruction_manager_(reconstruction_manager),
      similarity_graph_(similarity_search_options, database_) {
  CHECK(options.Check());
}

void DistributedMapperController::SetMapReduceConfig(
    const MapReduceConfig& map_reduce_config) {
  map_reduce_config_ = map_reduce_config;
}

void DistributedMapperController::Run() {

  std::string mylog_filepath = options_.output_path +"/mylog.txt";//用于保存log信息
  std::fstream mylogfile(mylog_filepath,std::ios::out);
  if(!mylogfile.is_open())
  {
    std::cout<<"my log file open failed !!!"<<std::endl;
  }


  Timer timer;
  double load_twoview_geometries_time = 0.0;
  double indexing_time = 0.0;
  double matching_time = 0.0;
  double extract_largest_cc_time = 0.0;
  double ra_time = 0.0;
  double cluster_time = 0.0;
  double sfm_time = 0.0;
  double triangulate_time = 0.0;
  double ba_time = 0.0;

  // Load existed database.
  timer.Start();
  bool load_exist_database = LoadTwoviewGeometries();
  timer.Pause();
  load_twoview_geometries_time = timer.ElapsedSeconds();

  //////////////////////////////////////////////////////////////////
  // //  1. Indexing images by similarity search ///////////////////
  //////////////////////////////////////////////////////////////////
  if (!load_exist_database) {
    timer.Start();
    similarity_graph_.Start();
    similarity_graph_.Wait();
    timer.Pause();

    indexing_time = timer.ElapsedSeconds();
  }

  // 2. Distributed/Sequential Feature Extraction and Matching.
  if (!load_exist_database) {
    timer.Start();
    if (options_.distributed) {
      DistributedFeatureExtractionAndMatching();
    } else {
      SequentialFeatureExtractionAndMatching();
    }
    timer.Pause();

    matching_time = timer.ElapsedSeconds();
  }

  //////////////////////////////////////////////////////////////////
  // // 3. Load Two view geometries from match inliers. ////////////
  //////////////////////////////////////////////////////////////////
  if (!load_exist_database) {
    LoadTwoviewGeometries();
  }





  //my change for replaceability metric
  if(clustering_options_.cluster_type == "METRIC")
  {
    if(options_.metric_path.empty())
    {
      LOG(INFO)<<"MUST NAME THE METRIC FILE WHEN USING METRIC CLUSTER";    
      exit(EXIT_FAILURE);
    }      
  }
  std::unordered_map<ImagePair,float> replacing_relationships;
  std::set<image_t> reserved_imgs;
  if(!options_.metric_path.empty())
  {
    LoadReplaceabilityMetric(replacing_relationships,reserved_imgs);
    view_graph_.FilterKeySubset(replacing_relationships,reserved_imgs); //filter two view geometries by reserved imgs
  }


  // LOG(INFO)<<replacing_relationships[ImagePair(5973,5872)];
  // LOG(INFO)<<replacing_relationships[ImagePair(5872,5973)];

  // std::unordered_map<ImagePair, TwoViewInfo> twoview_geometries = view_graph_.TwoViewGeometries();
  // LOG(INFO)<<"twoview_geometries size"<<twoview_geometries.size();
  // ImagePair pair1 = std::make_pair(5973,5872);
  // ImagePair pair2 = std::make_pair(5872,5973);
  // if(twoview_geometries.count(pair1)!=0)
  // {
  //   LOG(INFO)<<"("<<pair1.first<<","<<pair1.second<<")";
  // }
  //   if(twoview_geometries.count(pair2)!=0)
  // {
  //   LOG(INFO)<<"("<<pair2.first<<","<<pair2.second<<")";
  // }
  // for(auto geometry : twoview_geometries)
  // {
  //   if(geometry.first.first ==5973 ||geometry.first.second==5973)
  //   {
  //     LOG(INFO)<<"("<<geometry.first.first<<","<<geometry.first.second<<")";
  //   }
  // }
  // getchar();



  if (options_.reconstruct_largest_cc) {
    PrintHeading1("Extracting Largest Connected Component...");
    LOG(INFO) << "Total image number: " << view_graph_.ImageIds().size();
    mylogfile<<"Extracting Largest Connected Component..."<<std::endl;
    mylogfile<<"Total image number: " << view_graph_.ImageIds().size()<<std::endl;
    timer.Start();
    //this may filter some best edges?
    // view_graph_.FilterViewGraphCyclesByRotation(5.0);
    view_graph_.TwoviewGeometriesToImagePairs();
    view_graph_.ExtractLargestCC(); // get image_ids_ of lcc
    view_graph_.FilterEdgesLcc(); //filter two view geometries by reserved imgs
    timer.Pause();
    extract_largest_cc_time = timer.ElapsedSeconds();
    LOG(INFO) << "image number in largest cc: "
              << view_graph_.ImageIds().size();
    LOG(INFO) << "edges after largest cc (TwoViewGeometries): "
              << view_graph_.TwoViewGeometries().size();
    LOG(INFO) << "edges after largest cc (ImagePairs): "
              << view_graph_.ImagePairs().size();
    mylogfile<<"image number in largest cc: "
              << view_graph_.ImageIds().size()<<std::endl;
  }



  // std::unordered_map<ImagePair, TwoViewInfo> twoview_geometries = view_graph_.TwoViewGeometries();
  // LOG(INFO)<<"twoview_geometries size"<<twoview_geometries.size();
  // ImagePair pair1 = std::make_pair(5973,5872);
  // ImagePair pair2 = std::make_pair(5872,5973);
  // if(twoview_geometries.count(pair1)!=0)
  // {
  //   LOG(INFO)<<"("<<pair1.first<<","<<pair1.second<<")";
  // }
  //   if(twoview_geometries.count(pair2)!=0)
  // {
  //   LOG(INFO)<<"("<<pair2.first<<","<<pair2.second<<")";
  // }
  // for(auto geometry : twoview_geometries)
  // {
  //   if(geometry.first.first ==5973 ||geometry.first.second==5973)
  //   {
  //     LOG(INFO)<<"("<<geometry.first.first<<","<<geometry.first.second<<")";
  //   }
  // }
  // getchar();

  //////////////////////////////////////////////////////////////////
  // // 4. Global Rotation Averaging ///////////////////////////////
  //////////////////////////////////////////////////////////////////
  PrintHeading1("Global Rotation Averaging...");
  LOG(INFO)<<"Global Rotation Averaging...";
  mylogfile<<"Global Rotation Averaging..."<<std::endl;
  timer.Start();
  //this step will change the database.db file every time
  // GlobalRotationAveraging();
  // const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
  //     view_graph_.TwoViewGeometries();
  // for (auto view_pair_it : view_pairs) {
  //   LOG(INFO)<<view_pair_it.first.first<<", "<<view_pair_it.first.second
  //           <<": r-("<<view_pair_it.second.position_2[0]<<","<<view_pair_it.second.position_2[1]<<","<<view_pair_it.second.position_2[2]
  //           <<": t-("<<view_pair_it.second.rotation_2[0]<<","<<view_pair_it.second.rotation_2[1]<<","<<view_pair_it.second.rotation_2[2];
  // }
  // exit(EXIT_SUCCESS);

  timer.Pause();
  ra_time = timer.ElapsedSeconds();

    LOG(INFO) << "image number after global rotation averaging: "
              << view_graph_.ImageIds().size();
    LOG(INFO) << "edges after largest cc (TwoViewGeometries): "
              << view_graph_.TwoViewGeometries().size();
    LOG(INFO) << "edges after largest cc (ImagePairs): "
              << view_graph_.ImagePairs().size();

  //////////////////////////////////////////////////////////////////
  // // 4.5 Construct metric graph and cluster images //////////////
  //////////////////////////////////////////////////////////////////
  
  if(clustering_options_.cluster_type == "METRIC")
  {
    PrintHeading1("Partitioning the Scene by metric...");
    mylogfile<<"Partitioning the Scene by metric..."<<std::endl;
       timer.Start();
    //Merge_v2 version
    //Move images and vertices to metric graph


  // CHECK_EQ(replacing_relationships.count(std::make_pair(3035,3036)),1);
  // ImagePair testpair = std::make_pair(3035,3036);
  // LOG(INFO)<<"metric_graph edge: "<<replacing_relationships[testpair];
  //   ImagePair testpair1 = std::make_pair(3036,3035);
  // LOG(INFO)<<"metric_graph edge: "<<replacing_relationships[testpair1];
  // exit(EXIT_SUCCESS);

    GenerateMetricGraph(replacing_relationships);
    ClusteringScenesByMetric(); // cluster images by the metric_graph


    timer.Pause();
    cluster_time = timer.ElapsedSeconds();

    mylogfile<<"SubclusterSequentialSfM..."<<std::endl;
    PrintHeading1("SubclusterSequentialSfM...");
    timer.Start();
    SubclusterSequentialSfM();//poses
    timer.Pause();
    sfm_time = timer.ElapsedSeconds();
    
  }

  else
  {
    //////////////////////////////////////////////////////////////////
    // 5. Partitioning the images into the given number of clusters //
    //////////////////////////////////////////////////////////////////
    PrintHeading1("Partitioning the Scene...");
    mylogfile<<"Partitioning the Scene..."<<std::endl;
    timer.Start();
    ClusteringScenes();
    timer.Pause();
    cluster_time = timer.ElapsedSeconds();

    mylogfile<<"SequentialSfM..."<<std::endl;
    PrintHeading1("SequentialSfM...");
    timer.Start();
    SequentialSfM();
    timer.Pause();
    sfm_time = timer.ElapsedSeconds();
  }




  // //////////////////////////////////////////////////////////////////
  // // // 6. Distributed/Parallel Reconstruction /////////////////////
  // //////////////////////////////////////////////////////////////////
  // mylogfile<<"SequentialSfM..."<<std::endl;
  // PrintHeading1("SequentialSfM...");
  // timer.Start();
  // if (options_.distributed) {
  //   DistributedSfM();
  // } else {
  //   SequentialSfM();
  // }
  // timer.Pause();
  // sfm_time = timer.ElapsedSeconds();

  // LOG(INFO)<<"My seq merged reconstructions size: "<<my_seq_merged_reconstructions_.size();
  // for (size_t i = 0; i < my_seq_merged_reconstructions_.size(); ++i) {
  //   ReconstructionManager* recon = my_seq_merged_reconstructions_[i];
  //   for(size_t j = 0; j < recon->Size(); ++j)
  //   {
  //     LOG(INFO)<<"j: "<<j<<" recon Size: "<<recon->Size();
  //     Reconstruction rec = recon->Get(j);
  //     LOG(INFO)<<"cameras:"<<rec.NumCameras()<<" images:"<<rec.NumImages()<<" points:"<<rec.NumPoints3D();

  //     std::string reconstruction_path;
  //     if(recon->Size()>1)
  //     {
  //       reconstruction_path=JoinPaths(JoinPaths(options_.output_path+"/seq_results/", std::to_string(i))+"/",std::to_string(j));
  //     }
  //     else{
  //       reconstruction_path=JoinPaths(options_.output_path+"/seq_results/", std::to_string(i));
  //     }

      
  //     // JoinPaths("/seq_results/",std::to_string(i));

  //     // const std::string reconstruction_path =
  //     // JoinPaths(options_.output_path+"/seq_results/", std::to_string(j));
  //     LOG(INFO)<<"write to "<<reconstruction_path;
  //     CreateDirIfNotExists(reconstruction_path);
  //     recon->Get(j).WriteBinary(reconstruction_path);
  //   }
  // }
  // getchar();

  LOG(INFO)<<"My seq merged reconstructions size: "<<my_seq_merged_reconstructions2_.size();
  for (size_t i = 0; i < my_seq_merged_reconstructions2_.size(); ++i) {
    ReconstructionManager* recon = &my_seq_merged_reconstructions2_[i];
    for(size_t j = 0; j < recon->Size(); ++j)
    {
      LOG(INFO)<<"j: "<<j<<" recon Size: "<<recon->Size();
      Reconstruction rec = recon->Get(j);
      LOG(INFO)<<"cameras:"<<rec.NumCameras()<<" images:"<<rec.NumImages()<<" points:"<<rec.NumPoints3D();

      std::string reconstruction_path;
      if(recon->Size()>1)
      {
        reconstruction_path=JoinPaths(JoinPaths(options_.output_path+"/seq_results/", std::to_string(i))+"/",std::to_string(j));
      }
      else{
        reconstruction_path=JoinPaths(options_.output_path+"/seq_results/", std::to_string(i));
      }

      
      // JoinPaths("/seq_results/",std::to_string(i));

      // const std::string reconstruction_path =
      // JoinPaths(options_.output_path+"/seq_results/", std::to_string(j));
      LOG(INFO)<<"write to "<<reconstruction_path;
      CreateDirIfNotExists(reconstruction_path);
      recon->Get(j).WriteBinary(reconstruction_path);
    }

    // if (i == my_seq_merged_reconstructions2_.size()-1)
    //   reconstruction_manager_ = &my_seq_merged_reconstructions2_[i];
    if (i == my_seq_merged_reconstructions2_.size()-1)
      *reconstruction_manager_ = std::move(my_seq_merged_reconstructions2_[i]);
  }
  // exit(1);
  
  //test
    Reconstruction& global_recon = reconstruction_manager_->Get(0);
    LOG(INFO)<<"current global recon has: "<<global_recon.NumImages()<<" images, "
                                          <<global_recon.NumCameras()<<" cameras, "
                                          <<global_recon.NumPoints3D()<<" points3d";

  //////////////////////////////////////////////////////////////////
  // // 7. Re-triangulate scene structures /////////////////////////
  //////////////////////////////////////////////////////////////////
  if (options_.retriangulate) {
    LOG(INFO) << "Triangulating for separators...";
    mylogfile<<"Triangulating for separators..."<<std::endl;
    timer.Start();
    Triangulate();
    timer.Pause();
    triangulate_time = timer.ElapsedSeconds();
  }


    LOG(INFO)<<"current global recon has: "<<global_recon.NumImages()<<" images, "
                                          <<global_recon.NumCameras()<<" cameras, "
                                          <<global_recon.NumPoints3D()<<" points3d";

  //////////////////////////////////////////////////////////////////
  // // 8. Final global bundle adjustment //////////////////////////
  //////////////////////////////////////////////////////////////////



  // if (options_.final_ba && (view_clustering_->GetInterClusters().size() > 1)) {

  // if (options_.final_ba && (subcluster_clustering_->GetInterClusters().size() > 1)) {
    LOG(INFO) << "Final Global Bundle Adjustment";
    mylogfile<<"Final Global Bundle Adjustment"<<std::endl;

    timer.Start();
    this->AdjustGlobalBundle();
    timer.Pause();
    ba_time = timer.ElapsedSeconds();
  // }

      LOG(INFO)<<"current global recon has: "<<global_recon.NumImages()<<" images, "
                                          <<global_recon.NumCameras()<<" cameras, "
                                          <<global_recon.NumPoints3D()<<" points3d";

  LOG(INFO) << "Time elapsed: \n"
            << "  -[load two view geometries]: " << load_twoview_geometries_time << " seconds.\n"
            << "  -[image indexing]: " << indexing_time << " seconds.\n"
            << "  -[extraction + matching]: " << matching_time << " seconds.\n"
            << "  -[Largest connected components extraction]: " << extract_largest_cc_time << " seconds.\n"
            << "  -[Rotation Averaging]: " << ra_time << " seconds.\n"
            << "  -[Image Clustering]: " << cluster_time << " seconds.\n"
            << "  -[SfM]: " << sfm_time << " seconds.\n"
            << "  -[Triangulate]: " << triangulate_time << " seconds.\n"
            << "  -[Bundle Adjustment]: " << ba_time << " seconds.\n";
  LOG(INFO) << "Reconstruction size: " << reconstruction_manager_->Size();

  mylogfile <<"Time elapsed: \n"
            << "  -[load two view geometries]: " << load_twoview_geometries_time << " seconds.\n"
            << "  -[image indexing]: " << indexing_time << " seconds.\n"
            << "  -[extraction + matching]: " << matching_time << " seconds.\n"
            << "  -[Largest connected components extraction]: " << extract_largest_cc_time << " seconds.\n"
            << "  -[Rotation Averaging]: " << ra_time << " seconds.\n"
            << "  -[Image Clustering]: " << cluster_time << " seconds.\n"
            << "  -[SfM]: " << sfm_time << " seconds.\n"
            << "  -[Triangulate]: " << triangulate_time << " seconds.\n"
            << "  -[Bundle Adjustment]: " << ba_time << " seconds.\n";
  mylogfile << "Reconstruction size: " << reconstruction_manager_->Size()<<std::endl;
  reconstruction_manager_->Get(0).ShowReconInfo();
  mylogfile.close();
}

bool DistributedMapperController::SubclusterSequentialSfM() {
  // //1.并行计算每个subcluster的位姿
  // ///////////////////////////////////
  // PrintHeading1("Reconstructing Subclusters...");
  // // mylogfile<<"Reconstructing Clusters..."<<std::endl;
  // std::unordered_map<size_t, ReconstructionManager> subcluster_reconstruction_managers;
  // std::vector<Reconstruction*> subcluster_reconstructions;
  // // ReconstructPartitions(reconstruction_managers, reconstructions);
  // ReconstructSubclusters(subcluster_reconstruction_managers,subcluster_reconstructions);


  //将inter_clusters_换成最底层的影像id
  std::vector<ImageCluster> inter_clusters;
  inter_clusters.reserve(inter_clusters_.size());
  // CHECK_EQ(inter_clusters_.size(),intra_clusters_.size());
  for(int i=0; i<inter_clusters_.size(); i++)
  {
    inter_clusters_[i].edges.clear();
    ImageCluster imgcluster;
    imgcluster.cluster_id = i; 
    std::vector<image_t> subcluster_ids =  inter_clusters_[i].image_ids;
    for(int j=0; j<subcluster_ids.size(); j++)
    {
      image_t subcluster_id = subcluster_ids[j];
      std::vector<image_t> imgs = subcluster_img_map_[subcluster_id];
      imgcluster.image_ids.insert(imgcluster.image_ids.begin(),imgs.begin(),imgs.end());
    }
    inter_clusters.push_back(imgcluster);
  }
  std::vector<ImageCluster> inter_subclusters = inter_clusters_;
  inter_clusters_ = inter_clusters;
  for(int i=0; i<inter_clusters_.size(); i++)
  {
    LOG(INFO)<< "inter_clusters_ index "<<i<<" has "<<inter_clusters_[i].image_ids.size()<<" images"<<std::endl;
  }

  //compute repeated ratio
  int original_images_num = metric_graph_.ImageIds().size();
  std::vector<std::set<image_t>> cluster_img_map;
  std::set<image_t> allimages;
  int clustered_images_num = 0;
  int average_cluster_size = 0;
  for(int i=0;i<inter_clusters_.size(); i++)
  {
    clustered_images_num += inter_clusters_[i].image_ids.size();//expanded clusters 影像数量
    std::set<image_t> cluster_imgs(inter_clusters_[i].image_ids.begin(),inter_clusters_[i].image_ids.end());//当前cluster有哪些images
    cluster_img_map.push_back(cluster_imgs);
    allimages.insert(cluster_imgs.begin(),cluster_imgs.end());
  }
  average_cluster_size = clustered_images_num / inter_clusters_.size();

  int clustered_edges_num =0;
  // std::set<image_t> allimages(metric_graph_.ImageIds().begin(),metric_graph_.ImageIds().end());
  int original_edges_num = 0;


  const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
      view_graph_.TwoViewGeometries();

  LOG(INFO)<<"view_graph_ pairs: "<<view_pairs.size();
  for (const auto& view_pair_it : view_pairs) {
    const ImagePair view_pair = view_pair_it.first;   

    image_t img1 = view_pair.first;
    image_t img2 = view_pair.second;

    if(allimages.count(img1)!=0 && allimages.count(img2)!=0)
      original_edges_num++;

    for(int i=0; i<cluster_img_map.size(); i++)
    {
      std::set<image_t> imgs = cluster_img_map[i];
      if(imgs.count(img1)!=0 && imgs.count(img2)!=0)//expanded cluster内部的edge
      {
        inter_clusters[i].edges.insert(std::make_pair(view_pair,1)); 
        break;
      }       
    }
  }

  for(int i=0;i<inter_clusters.size(); i++)
  {
    clustered_edges_num += inter_clusters[i].edges.size();
    inter_clusters[i].edges.clear();
  }


  LOG(INFO)
      << "#Images Clustering Summary:#\n"
      << "\t - Clusters number: " << inter_clusters_.size() << "\n"
      << "\t - Images number expanded from " << original_images_num
      << " to " << clustered_images_num << "\n"
      << "\t - Repeated Ratio: "
      << (float)(clustered_images_num - original_images_num) /
             (float)original_images_num
             << "\n average cluster size: "<<average_cluster_size
      << "\n"
      << "\t - Edges number reduced from " << original_edges_num << " to "
      << clustered_edges_num << "\n"
      << "\t - Lost ratio: "
      << (float)(original_edges_num - clustered_edges_num) /
             (float)original_edges_num;


  ////////////////////////////////////////////////////////
  //// 1. Reconstruct all clusters in sequential manner //
  ////////////////////////////////////////////////////////
  PrintHeading1("Reconstructing Clusters...");
  // mylogfile<<"Reconstructing Clusters..."<<std::endl;
  std::unordered_map<size_t, ReconstructionManager> reconstruction_managers;
  std::vector<Reconstruction*> reconstructions;
  ReconstructPartitions(reconstruction_managers, reconstructions);

  
    // exit(EXIT_SUCCESS);

  //////////////////////////////////////////////////
  //// 2. Merge clusters ///////////////////////////
  //////////////////////////////////////////////////
  PrintHeading1("Merging Clusters...");
  // mylogfile<<"Merging Clusters..."<<std::endl;

  // Determine the number of workers and threads per worker
  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
  // MergeClusters(reconstructions, reconstruction_managers, num_eff_threads);

  IncrementalSequentialSfM(reconstructions, reconstruction_managers, num_eff_threads);//按连接关系先后重建


  // //2.重建每个cluster
  // ///////////////////////////////////
  // std::unordered_map<size_t, ReconstructionManager> reconstruction_managers;
  // std::vector<Reconstruction*> reconstructions;
  // PrintHeading1("Incremental reconstruct Clusters...");
  // ReconstructPartitions(reconstruction_managers, reconstructions);
  // // ReconstructClusters(reconstruction_managers, reconstructions);

  return true;

  // //2.merge相连的subcluster
  // //////////////////////////////////
  // PrintHeading1("Merging Subclusters...");
  // // mylogfile<<"Merging Clusters..."<<std::endl;
  // // Determine the number of workers and threads per worker
  // const int kMaxNumThreads = -1;
  // const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
  // // MergeSubclusters(reconstructions, reconstruction_managers, num_eff_threads);

  // //3.merge不同的clusters
  // //////////////////////////////////
  // PrintHeading1("Merging Clusters...");
  // // mylogfile<<"Merging Clusters..."<<std::endl;
  // // Determine the number of workers and threads per worker
  // MergeClusters(reconstructions, reconstruction_managers, num_eff_threads);

  // return true;
}




bool DistributedMapperController::SequentialSfM() {

  // std::string mylog_filepath = options_.output_path +"/mylog.txt";//用于保存log信息
  // std::fstream mylogfile(mylog_filepath,std::ios::out||std::ios::app);
  // std::fstream mylogfile(mylog_filepath,std::ios::out);
  // if(!mylogfile.is_open())
  // {
  //   std::cout<<"my log file open failed !!!"<<std::endl;
  // }


  ////////////////////////////////////////////////////////
  //// 1. Reconstruct all clusters in sequential manner //
  ////////////////////////////////////////////////////////
  PrintHeading1("Reconstructing Clusters...");
  // mylogfile<<"Reconstructing Clusters..."<<std::endl;
  std::unordered_map<size_t, ReconstructionManager> reconstruction_managers;
  std::vector<Reconstruction*> reconstructions;
  ReconstructPartitions(reconstruction_managers, reconstructions);

  //////////////////////////////////////////////////
  //// 2. Merge clusters ///////////////////////////
  //////////////////////////////////////////////////
  PrintHeading1("Merging Clusters...");
  // mylogfile<<"Merging Clusters..."<<std::endl;

  // Determine the number of workers and threads per worker
  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
  MergeClusters(reconstructions, reconstruction_managers, num_eff_threads);

  return true;
}

bool DistributedMapperController::DistributedSfM() {
  Timer timer;
  /////////////////////////////////////////////////////////
  //// 1. Data preparation for local clusters /////////////
  /////////////////////////////////////////////////////////
  PrintHeading1("Reconstructing Clusters...");
  std::unique_ptr<std::unordered_map<size_t, DatabaseCache>> 
                  cluster_database_caches(new std::unordered_map<size_t, DatabaseCache>());
  std::unordered_map<size_t, std::vector<std::string>> cluster_images;
  const std::unordered_map<image_t, std::string>& image_id_to_name =
      view_graph_.ImageIdToName();

  cluster_database_caches->reserve(inter_clusters_.size());
  cluster_images.reserve(inter_clusters_.size());

  IncrementalMapperOptions local_mapper_options;
  for (size_t k = 0; k < inter_clusters_.size(); k++) {
    std::vector<std::string> image_name_list;
    image_name_list.reserve(inter_clusters_[k].image_ids.size());

    for (const auto image_id : inter_clusters_[k].image_ids) {
      image_name_list.push_back(image_id_to_name.at(image_id));
    }
    std::sort(image_name_list.begin(), image_name_list.end());
    LOG(INFO) << "cluster " << k << " has " << image_name_list.size()
              << " images.";
    cluster_images.emplace(k, image_name_list);

    std::unordered_set<std::string> image_name_set(image_name_list.begin(),
                                                   image_name_list.end());
    (*cluster_database_caches)[k].Load(
        database_, static_cast<size_t>(local_mapper_options.min_num_matches),
        local_mapper_options.ignore_watermarks, image_name_set);
  }

  /////////////////////////////////////////////////////////
  //// 2. Reconstruct all clusters in distributed manner //
  /////////////////////////////////////////////////////////
  CHECK_GT(map_reduce_config_.server_ips.size(), 0);

  const size_t cluster_num = inter_clusters_.size();

  DistributedTaskManager<SfMDataContainer> distributed_task_manager;
  distributed_task_manager.SetMapReduceConfig(map_reduce_config_);

  SfMDataContainer& sfm_data_container =
      distributed_task_manager.DataContainer();

  sfm_data_container.cluster_num = cluster_num;
  sfm_data_container.master_image_path = options_.image_path;
  sfm_data_container.server_ips =
      distributed_task_manager.GetMapReduceConfig().server_ips;
  sfm_data_container.server_ports =
      distributed_task_manager.GetMapReduceConfig().server_ports;
  if (options_.transfer_images_to_server) {
    sfm_data_container.cluster_images = cluster_images;
  }
  sfm_data_container.cluster_database_caches.swap(cluster_database_caches);
  sfm_data_container.task_type = "mapping";

  for (size_t i = 0; i < cluster_num; i++) {
    distributed_task_manager.Push(i);
  }

  distributed_task_manager.RunDistributed();

  // Stop all workers.
  for (size_t i = 0; i < map_reduce_config_.server_ips.size(); i++) {
    rpc::client c(map_reduce_config_.server_ips[i],
                  map_reduce_config_.server_ports[i]);
    c.call("Exit");
    c.call("StopServer");
  }

  //////////////////////////////////////////////////
  //// 3. Merge clusters ///////////////////////////
  //////////////////////////////////////////////////
  std::vector<Reconstruction*> reconstructions;
  for (uint i = 0; i < sfm_data_container.reconstructions.size(); i++) {
    reconstructions.push_back(sfm_data_container.reconstructions[i].release());
  }

  for (uint i = 0; i < reconstructions.size(); i++) {
    reconstructions[i]->ShowReconInfo();
  }
  // Determine the number of workers and threads per worker
  LOG(INFO) << "Sub-reconstructions size: " << reconstructions.size();
#ifdef __DEBUG__
  LOG(INFO) << "Extracting colors for local maps.";
  for (auto reconstruction : reconstructions) {
    ExtractColors(options_.image_path, reconstruction);
  }
  ExportUntransformedLocalRecons(reconstructions);
  LOG(INFO) << "Local reconstructions are exported to " << options_.output_path;
#endif

  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);

  PrintHeading1("Merging Clusters...");
  Node anchor_node;
  MergeClusters(reconstructions, num_eff_threads, anchor_node);

  LOG(INFO) << "Saving anchor node reconstruction...";
  timer.Start();
  reconstructions[anchor_node.id]->ShowReconInfo();
  reconstruction_manager_->Add(reconstructions[anchor_node.id]);
  reconstructions[anchor_node.id] = nullptr;
  timer.Pause();
  LOG(INFO) << "Time elapsed (saving anchor node reconstruction): " << timer.ElapsedSeconds();
  for (int i = 0; i < reconstructions.size(); ++i) {
    if (i != anchor_node.id) delete(reconstructions[i]);
  }
  
  return true;
}

bool DistributedMapperController::SequentialFeatureExtractionAndMatching() {
  Timer timer;
  LOG(INFO) << "Extracting features";
  timer.Start();
  ExtractFeature();
  timer.Pause();
  LOG(INFO) << "Time elapsed (feature extraction): " << timer.ElapsedSeconds()
            << " seconds";

  LOG(INFO) << "Matching...";
  timer.Start();
  Match();
  timer.Pause();
  LOG(INFO) << "Time elapsed: " << timer.ElapsedSeconds() << " seconds";

  return true;
}

bool DistributedMapperController::DistributedFeatureExtractionAndMatching() {
  const std::vector<ImagePair>& all_image_pairs =
      similarity_graph_.ImagePairs();
  const std::vector<int>& num_inliers = similarity_graph_.Scores();

  // Clustering images
  ImageCluster image_cluster;
  image_cluster.image_ids = similarity_graph_.ImageIds();
  for (uint i = 0; i < all_image_pairs.size(); i++) {
    const ImagePair image_pair = all_image_pairs[i];
    image_cluster.edges[image_pair] = num_inliers[i];
  }

  std::unique_ptr<ImageClustering> image_clustering(
      new ImageClustering(clustering_options_, image_cluster));
  image_clustering->Cut();
  image_clustering->ExpandAllEdges();
  image_clustering->OutputClusteringSummary();

  std::vector<ImageCluster> inter_clusters =
      image_clustering->GetInterClusters();

  /////////////////////////////////////////////////////////
  //// 1. Data preparation for local clusters /////////////
  /////////////////////////////////////////////////////////
  // PrintHeading1("Reconstructing Clusters...");
  const auto& image_id_to_name = similarity_graph_.ImageIdToName();
  std::unordered_map<size_t, std::vector<std::string>> cluster_images;
  std::unordered_map<size_t, std::vector<ImageNamePair>> cluster_matching_pairs;

  const size_t cluster_num = inter_clusters.size();
  cluster_images.reserve(cluster_num);
  cluster_matching_pairs.reserve(cluster_num);

  for (size_t k = 0; k < cluster_num; k++) {
    cluster_images[k].reserve(inter_clusters[k].image_ids.size());
    for (const auto image_id : inter_clusters[k].image_ids) {
      cluster_images[k].push_back(image_id_to_name.at(image_id));
    }
    LOG(INFO) << "cluster #" << k << " has " << cluster_images[k].size()
              << " images.";

    std::vector<ImagePair> image_pairs;
    for (const auto& image_pair : inter_clusters[k].edges) {
      image_pairs.emplace_back(image_pair.first);
    }

    cluster_matching_pairs[k].reserve(image_pairs.size());
    for (auto image_pair : image_pairs) {
      cluster_matching_pairs[k].emplace_back(
          image_id_to_name.at(image_pair.first),
          image_id_to_name.at(image_pair.second));
    }
  }

  /////////////////////////////////////////////////////////
  //// 2. Matching all clusters in distributed manner //
  /////////////////////////////////////////////////////////
  PrintHeading1("Reconstructing Clusters...");
  CHECK_GT(map_reduce_config_.server_ips.size(), 0);

  DistributedTaskManager<MatchesDataContainer> distributed_task_manager;
  distributed_task_manager.SetMapReduceConfig(map_reduce_config_);

  MatchesDataContainer& matches_data_container =
      distributed_task_manager.DataContainer();

  matches_data_container.cluster_num = inter_clusters.size();
  matches_data_container.master_image_path = options_.image_path;
  matches_data_container.server_ips =
      distributed_task_manager.GetMapReduceConfig().server_ips;
  matches_data_container.server_ports =
      distributed_task_manager.GetMapReduceConfig().server_ports;
  matches_data_container.image_name_to_id = similarity_graph_.ImageNameToId();
  matches_data_container.cluster_images = cluster_images;
  matches_data_container.cluster_matching_pairs = cluster_matching_pairs;
  matches_data_container.task_type = "matching";

  for (size_t i = 0; i < cluster_num; i++) {
    distributed_task_manager.Push(i);
  }

  distributed_task_manager.RunDistributed();
  // merge databases;
  matches_data_container.MergeData();
  matches_data_container.database_infos[0].ExportToDatabase(database_);

  return true;
}

void DistributedMapperController::ExtractFeature() {
  ImageReaderOptions reader_options;
  reader_options.database_path = options_.database_path;
  reader_options.image_path = options_.image_path;

  // if (!image_list_.empty()) {
  //   reader_options.image_list = image_list_;
  // }

  if (!ExistsCameraModelWithName(reader_options.camera_model)) {
    std::cerr << "ERROR: Camera model does not exist" << std::endl;
  }

  if (!VerifyCameraParams(reader_options.camera_model,
                          reader_options.camera_params)) {
    return;
  }
  LOG(INFO) << "Image path: " << reader_options.image_path;
  LOG(INFO) << "Database path: " << reader_options.database_path;
  // SiftExtractionOptions sift_extraction;
  SiftFeatureExtractor feature_extractor(reader_options, extraction_options_);

  feature_extractor.Start();
  feature_extractor.Wait();
}

void DistributedMapperController::Match() {
  const auto& images = database_.ReadAllImages();

  SiftMatchingOptions options;
  // Database database(database_path_);
  FeatureMatcherCache cache(5 * similarity_graph_.ImageIds().size(),
                            &database_);

  const std::vector<ImagePair>& image_id_pairs = similarity_graph_.ImagePairs();

  SiftFeatureMatcher sift_feature_matcher(options, &database_, &cache);
  sift_feature_matcher.Setup();
  cache.Setup();
  sift_feature_matcher.Match(image_id_pairs);
}

BundleAdjustmentOptions DistributedMapperController::GlobalBundleAdjustment()
    const {
  BundleAdjustmentOptions options;
  options.solver_options.function_tolerance = 0.0;
  options.solver_options.gradient_tolerance = 1.0;
  options.solver_options.parameter_tolerance = 0.0;
  options.solver_options.max_num_iterations = 50;
  options.solver_options.max_linear_solver_iterations = 100;
  // options.solver_options.minimizer_progress_to_stdout = true;
  options.solver_options.minimizer_progress_to_stdout = false;
  options.solver_options.num_threads = -1;
#if CERES_VERSION_MAJOR < 2
  options.solver_options.num_linear_solver_threads = GetEffectiveNumThreads(-1);
#endif  // CERES_VERSION_MAJOR
  options.print_summary = true;
  options.refine_focal_length = true;
  options.refine_principal_point = false;
  options.refine_extra_params = true;
  options.loss_function_type =
      BundleAdjustmentOptions::LossFunctionType::TRIVIAL;
  return options;
}

bool DistributedMapperController::IsPartialReconsExist(
    std::vector<Reconstruction*>& recons) const {
  const std::vector<std::string> dirs =
      colmap::GetRecursiveDirList(options_.output_path);
  recons.reserve(dirs.size());
  for (auto path : dirs) {
    Reconstruction* recon = new Reconstruction();
    if (ExistsFile(JoinPaths(path, "cameras.bin")) &&
        ExistsFile(JoinPaths(path, "images.bin")) &&
        ExistsFile(JoinPaths(path, "points3D.bin"))) {
      recon->ReadBinary(path);
    } else if (ExistsFile(JoinPaths(path, "cameras.txt")) &&
               ExistsFile(JoinPaths(path, "images.txt")) &&
               ExistsFile(JoinPaths(path, "points3D.txt"))) {
      recon->ReadText(path);
    } else {
      LOG(WARNING) << "cameras, images, points3D files do not exist at "
                   << path;
      continue;
    }
    recons.push_back(recon);
  }

  if (recons.empty()) return false;
  return true;
}

void DistributedMapperController::LoadImages() {
  // Loading database.
  Timer timer;

  // Reading all images.
  LOG(INFO) << "Reading images...";
  timer.Start();
  const std::vector<Image> vec_images = database_.ReadAllImages();

  for (const auto& image : vec_images) {
    similarity_graph_.AddImage(image.ImageId(), image.Name());
  }
}


void DistributedMapperController::LoadReplaceabilityMetric(std::unordered_map<ImagePair,float>& replacing_relationships,
                                                            std::set<image_t>& reserved_imgs) {
    Timer timer;

  replacing_relationships.clear();
  reserved_imgs.clear();

  // Every image in key subset should have a best replacing vertex with its metric
  //col 1: image names; col 2:best replacing vertex names; col 3:replacing metrics
  LOG(INFO) << "Reading keysubset and metrics...";
  timer.Start(); 

  std::ifstream metric_file(options_.metric_path,std::ios::in);
  if(!metric_file.is_open())
  {
    LOG(INFO)<<"Can't open metric file";
    exit(EXIT_FAILURE);
  }

  image_t imgid = 0, best_replace_id = 0;
  float metric = 0.0f;

  std::string line;
  getline(metric_file,line);
  getline(metric_file,line);
	while (metric_file.peek()!=EOF) {
		metric_file >> imgid >> best_replace_id >> metric;
    ImagePair replacing_pair = std::make_pair(imgid, best_replace_id);
    replacing_relationships.insert(std::make_pair(replacing_pair, metric));
    if(reserved_imgs.count(imgid)==0)
      reserved_imgs.insert(imgid);
    if(reserved_imgs.count(best_replace_id)==0)
      reserved_imgs.insert(best_replace_id);
  }



  LOG(INFO) << "Reserved "<< reserved_imgs.size()<<" imgs "<<replacing_relationships.size()<<" replacing entries";
  LOG(INFO) << "Reading metric file finished...";
  timer.Pause();
  // LOG(INFO) << "Elapsed Time[database reading]: " << timer.ElapsedSeconds()
  //           << " seconds";
  // LOG(INFO) << "image pairs: " << view_graph_.TwoViewGeometries().size();
}

bool DistributedMapperController::GenerateMetricGraph(std::unordered_map<ImagePair,float>replacing_relashionships)
{
  Timer timer;
  for (const auto& image_id : view_graph_.ImageIds()) {
    metric_graph_.AddImage(image_id, view_graph_.ImageIdToName()[image_id]);
  }

  std::unordered_map<ImagePair, TwoViewInfo> twoview_geometries = view_graph_.TwoViewGeometries();


  // //// HERE MY CODE FOR TEST NCUT
  // //// Clustering images
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
  // clustering_options_.cluster_type = "NCUT";
  // image_clustering_ = std::unique_ptr<ImageClustering>(
  //     new ImageClustering(clustering_options_, image_cluster_t));
  // image_clustering_->Cut();
  // exit(EXIT_SUCCESS);




  // ImagePair pair1 = std::make_pair(1729,1730);
  // ImagePair pair2 = std::make_pair(1730,3407);
  // if(twoview_geometries.count(pair1)!=0)
  // {
  //   LOG(INFO)<<"("<<pair1.first<<","<<pair1.second<<")";
  // }
  //   if(twoview_geometries.count(pair2)!=0)
  // {
  //   LOG(INFO)<<"("<<pair2.first<<","<<pair2.second<<")";
  // }

  //   if(replacing_relashionships.count(pair1)!=0)
  // {
  //   LOG(INFO)<<"-("<<pair1.first<<","<<pair1.second<<")";
  // }
  //   if(replacing_relashionships.count(pair2)!=0)
  // {
  //   LOG(INFO)<<"-("<<pair2.first<<","<<pair2.second<<")";
  // }
  // getchar();

  
  for (const auto& edge : twoview_geometries)
  {
    ImagePair imagepair1 = edge.first;
    ImagePair imagepair2 = ImagePair(imagepair1.second,imagepair1.first);

    int score1 = 0, score2 = 0;
    if(replacing_relashionships.count(imagepair1)!=0)
      score1 = (int)10000*replacing_relashionships[imagepair1];//change metric to int for convenience
    if(replacing_relashionships.count(imagepair2)!=0)
      score2 = (int)10000*replacing_relashionships[imagepair2];
    
    if(score1!=0||score2!=0)//img1和img2存在最佳替代关系
    {
      TwoViewInfo two_view_info;
      two_view_info.visibility_score = score1+score2; //如果互相为最佳替代节点，那么可靠性非常高
      // if(imagepair1.first==3035 || imagepair1.second==3035 || imagepair1.first==3036 || imagepair1.second==3036)
      //   LOG(INFO)<<"metric_graph edge: "<<imagepair1.first<<", "<<imagepair1.second;

      metric_graph_.AddTwoViewGeometry(imagepair1, two_view_info);
    }
  }


  timer.Pause();
  LOG(INFO) << "Elapsed Time[Metric Graph Generating]: " << timer.ElapsedSeconds()
            << " seconds";
  LOG(INFO) << "image pairs: " << metric_graph_.TwoViewGeometries().size();

  return metric_graph_.TwoViewGeometries().size() > 0;
}




bool DistributedMapperController::LoadTwoviewGeometries() {
  // Loading database.
  Timer timer;

  // Reading all images.
  LOG(INFO) << "Reading images...";
  timer.Start();
  const std::vector<Image> vec_images = database_.ReadAllImages();

  for (const auto& image : vec_images) {
    view_graph_.AddImage(image.ImageId(), image.Name());
  }

  // Reading scene graph
  LOG(INFO) << "Reading two view geometries...";
  std::vector<TwoViewGeometry> two_view_geometries;
  std::vector<image_pair_t> image_pair_ids;
  std::vector<std::pair<image_t, image_t>> image_pairs;
  std::vector<int> num_inliers;
  database_.ReadTwoViewGeometryNumInliers(&image_pairs, &num_inliers);
  database_.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

  for(int i=0; i< two_view_geometries.size(); i++)
  {
    if(two_view_geometries[i].qvec.x()==0 || two_view_geometries[i].qvec.y()==0
      || two_view_geometries[i].qvec.z()==0 || two_view_geometries[i].qvec.w()==0)
      {
        LOG(INFO)<<"two_view_geometries id: "<<i<<"\t"
            <<two_view_geometries[i].qvec.x()<<"\t"
            <<two_view_geometries[i].qvec.y()<<"\t"
            <<two_view_geometries[i].qvec.z()<<"\t"
            <<two_view_geometries[i].qvec.w()<<"\t";
      }
  }

    // ImagePair pair1 = std::make_pair(6227,6166);
  // ImagePair pair2 = std::make_pair(6166,6227);
  // if(twoview_geometries.count(pair1)!=0)
  // {
  //   LOG(INFO)<<"("<<pair1.first<<","<<pair1.second<<")";
  // }
  //   if(twoview_geometries.count(pair2)!=0)
  // {
  //   LOG(INFO)<<"("<<pair2.first<<","<<pair2.second<<")";
  // }
  // LOG(INFO)<<"search 6227 in image_pairs";
  // for(auto image_pair : image_pairs)
  // {
  //   if(image_pair.first ==6227 ||image_pair.second==6227)
  //   {
  //     LOG(INFO)<<"("<<image_pair.first<<","<<image_pair.second<<")";
  //   }
  // }

  // LOG(INFO)<<"search 6227 in image_pair_ids";
  // for(auto image_pair_id : image_pair_ids)
  // {
  //   image_t img1, img2;
  //   database_.PairIdToImagePair(image_pair_id,&img1,&img2);
  //   if(img1 ==6227 ||img2==6227)
  //   {
  //     LOG(INFO)<<"("<<img1<<","<<img2<<")";
  //   }
  // }
  // getchar();


  CHECK_EQ(image_pairs.size(), image_pair_ids.size());

  LOG(INFO) << "Estimating two view geometries";
  // #pragma omp parallel for
  for (uint i = 0; i < image_pairs.size(); i++) {
    const image_t image_id1 = image_pairs[i].first,
                  image_id2 = image_pairs[i].second;
    const ImagePair view_id_pair = ImagePair(image_id1, image_id2);
    TwoViewGeometry two_view_geometry = two_view_geometries[i];

    TwoViewInfo two_view_info;
    ceres::QuaternionToAngleAxis(two_view_geometry.qvec.data(),
                                 two_view_info.rotation_2.data());
    two_view_info.position_2 = two_view_geometry.tvec;
    two_view_info.visibility_score = two_view_geometry.inlier_matches.size();
    //my code add filter

// if(image_id1>1510 && image_id1<1700 && image_id2 >1720 && image_id2< 1950)
//   LOG(INFO)<<"view graph edge "<<image_id1<<" ,"<<image_id2<<" matches: "<<two_view_info.visibility_score;
  // CHECK_EQ(replacing_relationships.count(std::make_pair(3035,3036)),1);

    // 注意：当前版本一般都会加这个过滤，还比较有效果
    // if(two_view_info.visibility_score<50)
    //   continue;
    view_graph_.AddTwoViewGeometry(view_id_pair, two_view_info);
  }

  // exit(EXIT_SUCCESS);


  timer.Pause();
  LOG(INFO) << "Elapsed Time[database reading]: " << timer.ElapsedSeconds()
            << " seconds";
  LOG(INFO) << "image pairs: " << view_graph_.TwoViewGeometries().size();

  return view_graph_.TwoViewGeometries().size() > 0;
}

void DistributedMapperController::ClusteringScenesByMetric()
{
  std::unordered_map<int, int> basic_labels; //save subcluster results
  int subclusters_num;
  SubclustersByMetricGraph(basic_labels,subclusters_num);

#define debug_test
#ifdef debug_test
  //Output subclusters information
  if (clustering_options_.is_output_igraph) {
    std::string cluster_results_path = clustering_options_.graph_dir + "/subclusters.txt";
    LOG(INFO) << "Output results to " << cluster_results_path;
    CreateDirIfNotExists(clustering_options_.graph_dir);
    std::ofstream subcluster_out_file(cluster_results_path,std::ios::out);
    if (!subcluster_out_file.is_open()) {
      LOG(WARNING) << cluster_results_path<<" can't be opened!";
    }

    std::vector<std::pair<int,int>> labels_vec;
    for(auto label : basic_labels)
    {
      labels_vec.push_back(std::make_pair(label.first,label.second));
    }
    const auto cmp = [](const std::pair<int,int>& label1, const std::pair<int,int>& label2) {
      return label1.first < label2.first;
    };
    sort(labels_vec.begin(),labels_vec.end(),cmp);
    for(int i=0; i<labels_vec.size(); i++)
    {
      subcluster_out_file<<labels_vec[i].first<<"\t"<<labels_vec[i].second<<std::endl;
    }
    subcluster_out_file.close();
  }
#endif

  LOG(INFO)<<"subclusters_num: "<<subclusters_num;

#ifdef debug_test
  //code for check the map
  for(int i=0; i<subcluster_img_map_.size(); i++)
  {
    for(int j=0; j<subcluster_img_map_[i].size(); j++)
    {
      image_t img_id = subcluster_img_map_[i][j];
      // LOG(INFO)<<basic_labels[img_id]<<" "<<i;
      CHECK_EQ(basic_labels[img_id],i);//验证两种数据结构的一致性
    }
  }
#endif

  std::vector<image_t> subcluster_ids;//保存下subcluster id列表，供后续使用
  subcluster_ids.reserve(subclusters_num);
  for(int i=0;i<subclusters_num;i++)
  {
    subcluster_ids.push_back(i);
  }


  // ImageClustering::Options metric_cluster_options = clustering_options_;
  // metric_cluster_options.num_images_ub = metric_cluster_options.metric_num_imgs_ub; //可选参数，子cluster的最大size
  // metric_cluster_options.cluster_type = "NCUT";

  //此处应:
  //1.先用metric_graph的连通性划分出独立的多个cluster
  ///////////////////////////////////////////////////////
//   graph::UnionFind uf(metric_graph_.ImageIds().size());
//   std::vector<size_t> tmp_nodes(metric_graph_.ImageIds().begin(), metric_graph_.ImageIds().end());
//   uf.InitWithNodes(tmp_nodes);

//   for (auto image_pair : metric_graph_.ImagePairs()) {
//       uf.Union(image_pair.first, image_pair.second);
//   }

//   std::unordered_map<size_t, std::vector<image_t>> components;
//   for (auto image_id : metric_graph_.ImageIds()) {
//       const size_t parent_id = uf.FindRoot(image_id);
//       components[parent_id].push_back(image_id);
//   }

//   LOG(INFO) << "There are " << components.size() << " connected components in metric graph.";
//   // cluster_num_ = components.size();
//   size_t subcluster_id = 0;
//   for(auto component : components)
//   {
//       std::vector<image_t> img_ids = component.second;
//       LOG(INFO) << "Component #" << subcluster_id << "# has "
//           << img_ids.size() << " images.";

//       //1.1 独立区域的影像数量本就小于最小cluster的要求，则直接成为小cluster，无需进一步cut
//       ///////////////////////////////////////////////////////////////////////////////////
//       if(img_ids.size() <= metric_cluster_options.metric_num_imgs_ub)
//       {
//         for(int i = 0; i<img_ids.size(); i++)
//         {
//             image_t img_id = img_ids[i];
//             basic_labels[img_id] = subcluster_id;
//         }
//         subcluster_id++;
//       }
//       //1.2 独立区域的影像数量大于最小cluster的要求，根据metric进一步cut
//       ///////////////////////////////////////////////////////////////////////////////////
//       else
//       {
//         //cut large connected metric graph
//         std::set<image_t> img_set; // image set of current connnected metric graph
//         for(int i=0; i < img_ids.size(); i++)
//         {
//           img_set.insert(img_ids[i]);
//         }

//         // Clustering images
//         ImageCluster metric_cluster;
//         metric_cluster.image_ids = img_ids;//current metric graph

//         const std::unordered_map<ImagePair, TwoViewInfo>& metric_pairs =
//             metric_graph_.TwoViewGeometries(); // all metric edges
//         for (const auto& view_pair_it : metric_pairs) {
//           const ImagePair view_pair = view_pair_it.first;
//           if(img_set.count(view_pair.first) != 0
//            || (img_set.count(view_pair.second) != 0))//当前连通区域内的边
//            {
//               metric_cluster.edges[view_pair] = view_pair_it.second.visibility_score;
//            }          
//         }

//         metric_clustering_ = std::unique_ptr<ImageClustering>(
//             new ImageClustering(metric_cluster_options, metric_cluster));
//         std::unordered_map<int, int> labels;
//         int subcluster_num = 0;
//         metric_clustering_->CutByMetric(labels,subcluster_num); //基于metric edge将连通的metric graph划分为较小的clusters
//         for(auto label : labels)
//         {
//           CHECK_EQ(basic_labels.count(label.first),0);//basic_labels中不应有当前connected graph中的影像id
//           CHECK_LT(label.second, subcluster_num); //cluster id 小于 cluster num
//           basic_labels[label.first] = subcluster_id + label.second; //每一个小cluster都累加上之前的cluster id，往后顺延
//         }
//         subcluster_id += subcluster_num; //cluster id 加上当前graph分出的子集数量
//       }
//   }
//   int subclusters_num = subcluster_id;
//   LOG(INFO)<<"Total subclusters num: "<<subclusters_num;

// #define debug_test
// #ifdef debug_test
//   //Output subclusters information
//   if (metric_cluster_options.is_output_igraph) {
//     std::string cluster_results_path = metric_cluster_options.graph_dir + "/subclusters.txt";
//     LOG(INFO) << "Output results to " << cluster_results_path;
//     CreateDirIfNotExists(metric_cluster_options.graph_dir);
//     std::ofstream subcluster_out_file(cluster_results_path,std::ios::out);
//     if (!subcluster_out_file.is_open()) {
//       LOG(WARNING) << cluster_results_path<<" can't be opened!";
//     }

//     std::vector<std::pair<int,int>> labels_vec;
//     for(auto label : basic_labels)
//     {
//       labels_vec.push_back(std::make_pair(label.first,label.second));
//     }
//     const auto cmp = [](const std::pair<int,int>& label1, const std::pair<int,int>& label2) {
//       return label1.first < label2.first;
//     };
//     sort(labels_vec.begin(),labels_vec.end(),cmp);
//     for(int i=0; i<labels_vec.size(); i++)
//     {
//       subcluster_out_file<<labels_vec[i].first<<"\t"<<labels_vec[i].second<<std::endl;
//     }
//     subcluster_out_file.close();
//   }
// #endif

//   std::vector<image_t> subcluster_ids;//保存下subcluster id列表，供后续使用
//   subcluster_ids.reserve(subclusters_num);
//   for(int i=0;i<subclusters_num;i++)
//   {
//     subcluster_ids.push_back(i);
//   }

//   std::vector<std::vector<image_t>> subcluster_map(subclusters_num); //subcluster id to img id list
//   for(auto label : basic_labels)
//   {
//     image_t img_id = label.first;
//     int subcluster_id = label.second;
//     std::vector<image_t>* img_list = &subcluster_map[subcluster_id];
//     img_list->push_back(img_id);
//   }

// // #define debug_test
// #ifdef debug_test
//   //code for check the map
//   for(int i=0; i<subcluster_map.size(); i++)
//   {
//     for(int j=0; j<subcluster_map[i].size(); j++)
//     {
//       image_t img_id = subcluster_map[i][j];
//       // LOG(INFO)<<basic_labels[img_id]<<" "<<i;
//       CHECK_EQ(basic_labels[img_id],i);//验证两种数据结构的一致性
//     }
//   }
// #endif

//   //sort the map
//   for(int i=0; i<subcluster_map.size(); i++)
//   {
//     std::vector<image_t>* img_list = &subcluster_map[i];
//     sort(img_list->begin(),img_list->end());
//   }

//   //LOG(INFO) the map
//   for(int i=0; i<subcluster_map.size(); i++)
//   {
//     LOG(INFO)<< "subcluster id: "<<i;
//     for(int j=0; j<subcluster_map[i].size(); j++)
//     {
//       LOG(INFO)<< subcluster_map[i][j];
//     }
//   }

//   subcluster_img_map_ = subcluster_map; // save map from subcluster id to img id





  //2.基于匹配关系计算subclusters类内聚合程度和类间相似度,相当于计算3中的edges
  ///////////////////////////////////////////////////////////////////
  ImageCluster img_cluster;
  img_cluster.image_ids = metric_graph_.ImageIds();

  const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
      view_graph_.TwoViewGeometries();
  for (const auto& view_pair_it : view_pairs) {
    const ImagePair view_pair = view_pair_it.first;
    img_cluster.edges[view_pair] = view_pair_it.second.visibility_score;
  }

  // LOG(INFO)<<"basic_labels: 1868 subcluster id: "<<basic_labels[1868];
  // LOG(INFO)<<"basic_labels: 4947 subcluster id: "<<basic_labels[4947];
  img_clustering_ = std::unique_ptr<ImageClustering>(
      new ImageClustering(clustering_options_, img_cluster));
  // img_clustering_->GroupingImageEdges(basic_labels,subclusters_num); // group edges(inside cluster and between cluster)
  img_clustering_->GroupingImageEdges(subcluster_img_map_, basic_labels); // group edges(inside cluster and between cluster)
  std::vector<ImageCluster> all_subclusters = img_clustering_->GetIntraClusters();
  std::unordered_map<ImagePair, std::vector<graph::Edge>> inter_subcluster_edges = img_clustering_->GetAllInterClusterEdges();


  std::unordered_map<size_t, float> self_similarity_scores;  
  std::unordered_map<ImagePair, float> cross_similarity_ratios;
  img_clustering_->FilterImageEdges(self_similarity_scores, cross_similarity_ratios); // Filter edges between images


  //3.基于subcluster之间的edges大小建立graph，并cut这个graph
  //////////////////////////////////////////////////////////////////
  ImageCluster subcluster_cluster;
  subcluster_cluster.image_ids = subcluster_ids;
  for(auto cross_similarity_ratio : cross_similarity_ratios)
  {
    subcluster_cluster.edges[cross_similarity_ratio.first] = (int)1000*cross_similarity_ratio.second;//将float强制转为int
    // if(cross_similarity_ratio.first.first==63 &&cross_similarity_ratio.first.second==65)
    //   LOG(INFO)<<"cross ratio of 63,65: (float)"<<cross_similarity_ratio.second<<", " <<subcluster_cluster.edges[cross_similarity_ratio.first];
  }



  ImageClustering::Options subcluster_cluster_options = clustering_options_;
  subcluster_cluster_options.num_images_ub = clustering_options_.cluster_subcluster_num_ub; //每个大的cluster包含20个subcluster，相当于一个cluster100张左右
  // subcluster_cluster_options.cluster_type = "NCUT";
  subcluster_cluster_options.cluster_type = "KMEANS"; //METIS NCUT
  // subcluster_cluster_options.cluster_type = "SPECTRAL";
  // subcluster_cluster_options.cluster_type = "HYBRID";
  // subcluster_cluster_options.cluster_type = "COMMUNITY_DETECTION";
  subcluster_clustering_ = std::unique_ptr<ImageClustering>(
      new ImageClustering(subcluster_cluster_options, subcluster_cluster));

  std::unordered_map<int, int> cluster_labels;
  int cluster_num = 0;
  subcluster_clustering_->CutSubClusters(cluster_labels, cluster_num); // group subclusters

  if(cluster_num == 1)//如果只分了1个cluster
  {    
    inter_clusters_.clear();
    LOG(INFO)<<"subcluster_cluster.image_ids.size(): "<<subcluster_cluster.image_ids.size();
    for(int i=0; i<subcluster_cluster.image_ids.size(); i++)
    {      
      cluster_labels.insert(std::make_pair(subcluster_cluster.image_ids[i],0)); //cluster id 都为0
    }
    inter_clusters_.push_back(subcluster_cluster);
    subcluster_clustering_->inter_clusters_ = inter_clusters_;
    
    // LOG(INFO)<<inter_clusters_.size();
    // LOG(INFO)<<inter_clusters_[0].image_ids.size();
    // LOG(INFO)<<inter_clusters_[0].edges.size();
    // LOG(INFO)<<inter_clusters_[0].cluster_id;
    // inter_clusters_[0].image_ids = subcluster_cluster.image_ids;
    // inter_clusters_[0].cluster_id = 0;
    // LOG(INFO)<<subcluster_cluster.edges.size();
    // inter_clusters_[0].edges = subcluster_cluster.edges;
    // LOG(INFO)<<inter_clusters_[0].edges.size();
    // // intra_clusters_ = inter_clusters_;
    // LOG(INFO)<<inter_clusters_.size();
  }


  // std::ofstream labels_out("/16t/gy/cluster_stat_1.txt");
  // for(auto itr : cluster_labels)  
  //   labels_out<<itr.first<<"\t"<<itr.second<<std::endl;
  // labels_out.close();

  //Output final cluster results
#ifdef debug_test
  //Output subclusters information
  if (subcluster_cluster_options.is_output_igraph) {
    std::string cluster_results_path = subcluster_cluster_options.graph_dir + "/final_clusters.txt";
    LOG(INFO) << "Output results to " << cluster_results_path;
    CreateDirIfNotExists(subcluster_cluster_options.graph_dir);
    std::ofstream cluster_out_file(cluster_results_path,std::ios::out);
    if (!cluster_out_file.is_open()) {
      LOG(WARNING) << cluster_results_path<<" can't be opened!";
    }

    LOG(INFO)<<"cluster_num: "<<cluster_num;
    std::vector<std::vector<int>> clusters_subcluster_map(cluster_num);//cluster id 及其对应的 subclusters ids
    for(auto cluster_label : cluster_labels)
    {
      int subcluster_id = cluster_label.first;
      int cluster_id = cluster_label.second;
      std::vector<int>& subclusters_ids = clusters_subcluster_map[cluster_id];
      subclusters_ids.push_back(subcluster_id);
    }


    LOG(INFO)<<"clusters_subcluster_map size: "<<clusters_subcluster_map.size();

    int max_cluster_size = 0;
    int min_cluster_size = 1000;
    int max_cluster_id = 0;
    int min_cluster_id = 0;
    for(int i=0; i<clusters_subcluster_map.size(); i++) //sort ids
    {
      std::vector<int>& subclusters = clusters_subcluster_map[i];
      sort(subclusters.begin(),subclusters.end());
      //确认cluster内subcluster数量的范围
      if(subclusters.size()>max_cluster_size)
      {
        max_cluster_size = subclusters.size();
        max_cluster_id = i;
      }
      if(subclusters.size()<min_cluster_size)
      {
        min_cluster_size = subclusters.size();
        min_cluster_id = i;
      }
    }
    LOG(INFO)<<"the cluster "<<max_cluster_id<<" has max "<<max_cluster_size<< " subclusters";
    LOG(INFO)<<"the cluster "<<min_cluster_id<<" has min "<<min_cluster_size<< " subclusters";




    for(int i=0; i<clusters_subcluster_map.size(); i++) //highest-level clusters
    {
      // LOG(INFO)<<"cluster id: "<<i<<std::endl;      
      std::vector<int> subclusters = clusters_subcluster_map[i];
      cluster_out_file<<"cluster id: "<<i<<" with "<<subclusters.size()<<" subclusters"<<std::endl;
      LOG(INFO)<<"cluster id: "<<i<<" with "<<subclusters.size()<<" subclusters";
      for(int j=0; j< subclusters.size(); j++)//subclusters ids
      {        
        int subcluster_id = subclusters[j];
        // LOG(INFO)<<"subcluster_id: "<<subcluster_id;
        // LOG(INFO)<<"subcluster_img_map_ size(): "<<subcluster_img_map_.size();
        std::vector<image_t> imgids = subcluster_img_map_[subcluster_id];
        cluster_out_file<<"\tsubcluster id: "<<subcluster_id<<" with "<<imgids.size()<<" images"<<std::endl;
        for(int k=0; k<imgids.size(); k++)
        {
          cluster_out_file<<"\t\t"<<imgids[k]<<std::endl;
        }
      }
    }
    cluster_out_file.close();
  }
#endif




  //3.利用subclusters间的strong edges构建clusters间的overlap
  /////////////////////////////////////////////////////////
  // subcluster_clustering_->Expand();
  if(cluster_num>1)
  {
    subcluster_clustering_->ExpandByStrongEdges();//仅使用>20（相当于0.02）的edges作为expand
    subcluster_clustering_->OutputClusteringSummary();

    inter_clusters_ = subcluster_clustering_->GetInterClusters();
    intra_clusters_ = subcluster_clustering_->GetIntraClusters();
    for (auto cluster : inter_clusters_) {
      cluster.ShowInfo();
    }

#ifdef debug_test

    int max_expanded_cluster_size = 0;
    int min_expanded_cluster_size = 1000;
    int max_expanded_cluster_id = 0;
    int min_expanded_cluster_id = 0;
    for(int i =0; i<inter_clusters_.size(); i++ )
    {
      ImageCluster inter_cluster = inter_clusters_[i];
      int cluster_id = inter_cluster.cluster_id;
      std::vector<image_t> subcluster_ids = inter_cluster.image_ids;
      //确认cluster内subcluster数量的范围
      if(subcluster_ids.size()>max_expanded_cluster_size)
      {
        max_expanded_cluster_size = subcluster_ids.size();
        max_expanded_cluster_id = i;
      }
      if(subcluster_ids.size()<min_expanded_cluster_size)
      {
        min_expanded_cluster_size = subcluster_ids.size();
        min_expanded_cluster_id = i;
      }
    }
    LOG(INFO)<<"the expanded cluster "<<max_expanded_cluster_id<<" has max "<<max_expanded_cluster_size<< " subclusters";
    LOG(INFO)<<"the expanded cluster "<<min_expanded_cluster_id<<" has min "<<min_expanded_cluster_size<< " subclusters";

    //Output expanded clusters information
    if (subcluster_cluster_options.is_output_igraph) {
      std::string cluster_results_path = subcluster_cluster_options.graph_dir + "/final_expanded_clusters.txt";
      LOG(INFO) << "Output expanded results to " << cluster_results_path;
      CreateDirIfNotExists(subcluster_cluster_options.graph_dir);
      std::ofstream cluster_out_file(cluster_results_path,std::ios::out);
      if (!cluster_out_file.is_open()) {
        LOG(WARNING) << cluster_results_path<<" can't be opened!";
      }

      // LOG(INFO)<<"cluster_num: "<<cluster_num;
      // std::vector<std::vector<int>> clusters_subcluster_map(cluster_num);//cluster id 及其对应的 subclusters ids
      for(int i =0; i<inter_clusters_.size(); i++ )
      {
        ImageCluster inter_cluster = inter_clusters_[i];
        int cluster_id = inter_cluster.cluster_id;
        std::vector<image_t> subcluster_ids = inter_cluster.image_ids;
        sort(subcluster_ids.begin(),subcluster_ids.end());
        cluster_out_file<<"cluster id: "<<cluster_id<<" with "<<subcluster_ids.size()<<" subclusters"<<std::endl;
        for(int j = 0; j<subcluster_ids.size(); j++)
        {
          image_t subcluster_id = subcluster_ids[j];
          std::vector<image_t> img_ids = subcluster_img_map_[subcluster_id];
          cluster_out_file<<"\tsubcluster id: "<<subcluster_id<<" with "<<img_ids.size()<<" images"<<std::endl;
          for( int k=0; k<img_ids.size(); k++)
          {
            image_t img_id = img_ids[k];
            cluster_out_file<<"\t\t"<<img_id<<std::endl;
          }
        }
      }
      cluster_out_file.close();
    }
#endif
  }





  


  //此处需要做两件事：
  //1、在cluster内计算subcluster为节点、subcluster间cross ratio为edge的MST
  //// Input: 每个cluster内的subclusters_id,以及他们之间的edge。
  //// Output: vector<unordered_set<ImagePair>>保存每个cluster的MST image_pair
  //2、根据MST结果获得best_edges
  //// 第一步：insert所有subcluster内部的edges
  //// 第二步：对每个cluster，都能得到一个对应的Edges best_edges,最后应该得到一个vector
  //// 这一过程需要拿到subcluster间的所有edges信息，在img_clustering_->GroupingImageEdges中的subclusters_all_edges_中有
  LOG(INFO)<<"GetBestEdges...";
  // std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
  //         view_graph_.TwoViewGeometries();
  all_best_edges_ = subcluster_clustering_->GetBestEdges(all_subclusters, inter_subcluster_edges, 
                                                          view_pairs,options_.max_relative_rotation_difference_degrees);
  LOG(INFO)<<"all best edges size: "<<all_best_edges_.size()
            << "; ratio: "<< 100.0 * (float)all_best_edges_.at(0).size()/(float)view_pairs.size() <<"%";

  // exit(EXIT_FAILURE);
}


void DistributedMapperController::SubclustersByMetricGraph(std::unordered_map<int, int>& basic_labels, int& subclusters_num)
{
  //UnionFind找到metric graph中不连通的多个components
  graph::UnionFind uf(metric_graph_.ImageIds().size());
  std::vector<size_t> tmp_nodes(metric_graph_.ImageIds().begin(), metric_graph_.ImageIds().end());
  uf.InitWithNodes(tmp_nodes);

  for (auto image_pair : metric_graph_.ImagePairs()) {
      uf.Union(image_pair.first, image_pair.second);
      // if(image_pair.first==3035 || image_pair.second==3035 || image_pair.first==3036 || image_pair.second==3036)
      //   LOG(INFO)<<"metric_graph edge: "<<image_pair.first<<", "<<image_pair.second;
  }

  std::unordered_map<size_t, std::vector<image_t>> components;
  for (auto image_id : metric_graph_.ImageIds()) {
      const size_t parent_id = uf.FindRoot(image_id);
      components[parent_id].push_back(image_id);
  }

  LOG(INFO) << "There are " << components.size() << " connected components in metric graph.";
  // cluster_num_ = components.size();

  int subcluster_id = 0;
  std::unordered_map<size_t, std::vector<image_t>> sorted_components;
  for(auto component : components)
  {
    sorted_components.insert(std::make_pair(subcluster_id,component.second));
    subcluster_id++;

    // for(auto img: component.second)
    // {
    //   if(img==3035 || img==3036)
    //     LOG(INFO)<<"current component size: "<<component.second.size();
    // }
  }
  components = sorted_components;

  


  // std::unordered_map<image_t,size_t> subcluster_labels;
  //确保subcluster的影像数量在合理范围内
  std::string subcluster_path = clustering_options_.graph_dir + "/subclusters_0" + ".txt"; 
  OutputComponents(subcluster_path, components);

  int itr_count = 0;
  // while(itr_count<5)
  // {

    //将较大的components进一步NCUT
    LOG(INFO)<<"DivideLargeComponents...";
    DivideLargeComponents(components);    
    std::string subcluster_path1 = clustering_options_.graph_dir + "/subclusters_divided"+ std::to_string(itr_count) + ".txt"; 
    OutputComponents(subcluster_path1, components);

    //将较小的components合并
    LOG(INFO)<<"MergeSmallComponents...";
    MergeSmallComponents(components);
    std::string subcluster_path2 = clustering_options_.graph_dir + "/subclusters_merged"+ std::to_string(itr_count) + ".txt"; 
    OutputComponents(subcluster_path2, components);

    int max_subcluster_size = 0;
    int min_subcluster_size = 100;
    for(auto component : components)
    {
      if(component.second.size() > max_subcluster_size)
        max_subcluster_size = component.second.size();
      if(component.second.size() < min_subcluster_size)
        min_subcluster_size = component.second.size();
    }

    if(min_subcluster_size ==1)
    {
      itr_count++;
      LOG(INFO)<<"MergeSmallComponents...";
      MergeSmallComponents(components);
      std::string subcluster_path2 = clustering_options_.graph_dir + "/subclusters_merged"+ std::to_string(itr_count) + ".txt"; 
      OutputComponents(subcluster_path2, components);

      int max_subcluster_size = 0;
      int min_subcluster_size = 100;
      for(auto component : components)
      {
        if(component.second.size() > max_subcluster_size)
          max_subcluster_size = component.second.size();
        if(component.second.size() < min_subcluster_size)
          min_subcluster_size = component.second.size();
      }
    }

    LOG(INFO)<<"smallest subcluster size: "<<min_subcluster_size;
    LOG(INFO)<<"largest subcluster size: "<<max_subcluster_size;
    // if(min_subcluster_size > 1 && max_subcluster_size < 2*clustering_options_.metric_num_imgs_ub)
    //   break;

  //   itr_count++;
  // }



  subclusters_num = components.size();
  subcluster_img_map_.clear();
  subcluster_img_map_.reserve(subclusters_num);

  subcluster_id = 0;
  for(auto component : components)
  {
    sort(component.second.begin(),component.second.end());
    subcluster_img_map_.push_back(component.second);//保存subcluster和影像的映射
    for(auto img_id : component.second)
    {
      if(basic_labels.count(img_id)==1)
        LOG(INFO)<<img_id;
      CHECK_EQ(basic_labels.count(img_id), 0);
      basic_labels.insert(std::make_pair(img_id,subcluster_id));
    }
    subcluster_id++;
  }


  // size_t subcluster_id = 0;
  // for(auto component : components)
  // {
  //     std::vector<image_t> img_ids = component.second;
  //     LOG(INFO) << "Component #" << subcluster_id << "# has "
  //         << img_ids.size() << " images.";      

  //     //1.1 独立区域的影像数量本就小于最小cluster的要求，则直接成为小cluster，无需进一步cut
  //     ///////////////////////////////////////////////////////////////////////////////////
  //     if(img_ids.size() <= metric_cluster_options.metric_num_imgs_ub)
  //     {
  //       for(int i = 0; i<img_ids.size(); i++)
  //       {
  //           image_t img_id = img_ids[i];
  //           basic_labels[img_id] = subcluster_id;
  //       }
  //       subcluster_id++;
  //     }
  //     //1.2 独立区域的影像数量大于最小cluster的要求，根据metric进一步cut
  //     ///////////////////////////////////////////////////////////////////////////////////
  //     else
  //     {
  //       //cut large connected metric graph
  //       std::set<image_t> img_set; // image set of current connnected metric graph
  //       for(int i=0; i < img_ids.size(); i++)
  //       {
  //         img_set.insert(img_ids[i]);
  //       }

  //       // Clustering images
  //       ImageCluster metric_cluster;
  //       metric_cluster.image_ids = img_ids;//current metric graph

  //       const std::unordered_map<ImagePair, TwoViewInfo>& metric_pairs =
  //           metric_graph_.TwoViewGeometries(); // all metric edges
  //       for (const auto& view_pair_it : metric_pairs) {
  //         const ImagePair view_pair = view_pair_it.first;
  //         if(img_set.count(view_pair.first) != 0
  //          || (img_set.count(view_pair.second) != 0))//当前连通区域内的边
  //          {
  //             metric_cluster.edges[view_pair] = view_pair_it.second.visibility_score;
  //          }          
  //       }

  //       metric_clustering_ = std::unique_ptr<ImageClustering>(
  //           new ImageClustering(metric_cluster_options, metric_cluster));
  //       std::unordered_map<int, int> labels;
  //       int subcluster_num = 0;
  //       metric_clustering_->CutByMetric(labels,subcluster_num); //基于metric edge将连通的metric graph划分为较小的clusters
  //       for(auto label : labels)
  //       {
  //         CHECK_EQ(basic_labels.count(label.first),0);//basic_labels中不应有当前connected graph中的影像id
  //         CHECK_LT(label.second, subcluster_num); //cluster id 小于 cluster num
  //         basic_labels[label.first] = subcluster_id + label.second; //每一个小cluster都累加上之前的cluster id，往后顺延
  //       }
  //       subcluster_id += subcluster_num; //cluster id 加上当前graph分出的子集数量
  //     }
  // }
  // int subclusters_num = subcluster_id;
  // LOG(INFO)<<"Total subclusters num: "<<subclusters_num;
}


void DistributedMapperController::DivideLargeComponents(std::unordered_map<size_t, std::vector<image_t>>& components)
{
  ImageClustering::Options metric_cluster_options = clustering_options_;
  metric_cluster_options.num_images_ub = metric_cluster_options.metric_num_imgs_ub; //可选参数，子cluster的最大size
  // metric_cluster_options.cluster_type = "NCUT";
  metric_cluster_options.cluster_type = "KMEANS";


  //   //// HERE MY CODE FOR TEST NCUT
  // // Clustering images
  // ImageCluster image_cluster_t;
  // image_cluster_t.image_ids.push_back(1729);
  // image_cluster_t.image_ids.push_back(1730);
  // image_cluster_t.image_ids.push_back(3410);
  // image_cluster_t.image_ids.push_back(3409);
  // image_cluster_t.image_ids.push_back(3411);
  // image_cluster_t.image_ids.push_back(3408);
  // image_cluster_t.image_ids.push_back(3407);
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

  //   // Clustering images
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

//   std::vector<std::pair<int, int>> image_pairs;
//   std::vector<int> weights;
// //   // Clustering images
//   image_pairs.clear();
//   weights.clear();
//   image_pairs.push_back(std::make_pair(3407, 3410));
//   weights.push_back(4009);
//     image_pairs.push_back(std::make_pair(3409, 3411));
//   weights.push_back(3764);
//     image_pairs.push_back(std::make_pair(1730, 3407));
//   weights.push_back(3818);
//     image_pairs.push_back(std::make_pair(3407, 3408));
//   weights.push_back(6683);
//     image_pairs.push_back(std::make_pair(3409, 3410));
//   weights.push_back(4642);
//     image_pairs.push_back(std::make_pair(1729, 1730));
//   weights.push_back(3530);

//   // ImageCluster image_cluster_t;
//   // image_cluster_t.image_ids.push_back(1729);
//   // image_cluster_t.image_ids.push_back(1730);
//   // image_cluster_t.image_ids.push_back(3407);
//   // image_cluster_t.image_ids.push_back(3408);
//   // image_cluster_t.image_ids.push_back(3409);
//   // image_cluster_t.image_ids.push_back(3410);
//   // image_cluster_t.image_ids.push_back(3411);
//   // image_cluster_t.edges[std::make_pair(3407, 3410)] = 4009;
//   // image_cluster_t.edges[std::make_pair(3409, 3411)] = 3764;
//   // image_cluster_t.edges[std::make_pair(1730, 3407)] = 3818;
//   // image_cluster_t.edges[std::make_pair(3407, 3408)] = 6683;
//   // image_cluster_t.edges[std::make_pair(3409, 3410)] = 4642;
//   // image_cluster_t.edges[std::make_pair(1729, 1730)] = 3530;
//   // image_clustering_ = std::unique_ptr<ImageClustering>(
//   //     new ImageClustering(metric_cluster_options, image_cluster_t));
//   // // image_clustering_->Cut();
//   //         std::unordered_map<int, int> labels_t;
//   //       int subcluster_num_t = 0;
//   // image_clustering_->CutByMetric(labels_t,subcluster_num_t);
  
//   // exit(EXIT_SUCCESS);

//   for(int i=0; i<image_pairs.size(); i++)
//   {
//     LOG(INFO)<<image_pairs[i].first<<","<<image_pairs[i].second<<","<<weights[i];
//   }

//         ImageCluster metric_cluster;
//         std::unique_ptr<ImageClustering> metric_clustering = std::unique_ptr<ImageClustering>(
//             new ImageClustering(metric_cluster_options, metric_cluster));

//   std::unordered_map<int, int> labels_t;
//   const std::unique_ptr<Cluster> cluster = metric_clustering->CreateCluster();
//   CHECK_NOTNULL(cluster.get());
//   // LOG(INFO) << "cluster num: " << num_clusters;
//   cluster->InitIGraph(image_pairs, weights);
//   // labels = cluster->ComputeCluster(image_ids, image_pairs, weights);
//   uint num_clusters_t = 2;
//   labels_t = cluster->ComputeCluster(image_pairs, weights, num_clusters_t);
//   // if (options_.is_output_igraph) {
//     // LOG(INFO) << "Output igraph to " << options_.graph_dir;
//     // CreateDirIfNotExists(options_.graph_dir);
//     // cluster->OutputIGraph(options_.graph_dir);
//     // cluster->MyOutputClusterBeforeExpand(options_.graph_dir,labels); // mycode
//   // }
//   num_clusters_t = cluster->ClusterNum();
//   LOG(INFO) << "Cutting Complete";
//   // LOG(INFO) << "Cutting Complete, grouping images...";

//   // if(root_cluster_.image_ids.size()==7)
//   {
//     for(const auto label:labels_t)
//     {
//       LOG(INFO)<<label.first<<" label:"<<label.second;
//     }
//   }

  // exit(EXIT_SUCCESS);



  size_t subcluster_id = 0;
  std::unordered_map<size_t, std::vector<image_t>> divided_components;
  for(auto component : components)
  {
      std::vector<image_t> img_ids = component.second;
      LOG(INFO) << "Component #" << component.first << "# has "
          << img_ids.size() << " images.";      

      //1.1 独立区域的影像数量本就小于最小cluster的要求，则直接成为小cluster，无需进一步cut
      /////////////////////////////////////////////////////////////////////////////////
      if(img_ids.size() <= metric_cluster_options.metric_num_imgs_ub)
      {
        // for(int i = 0; i<img_ids.size(); i++)
        // {
        //     image_t img_id = img_ids[i];
        //     basic_labels[img_id] = subcluster_id;
        // }
        divided_components.insert(std::make_pair(subcluster_id,img_ids));
        subcluster_id++;
      }
      //1.2 独立区域的影像数量大于最小cluster的要求，根据metric进一步cut
      ///////////////////////////////////////////////////////////////////////////////////
      else
      {
        //cut large connected metric graph
        std::set<image_t> img_set; // image set of current connnected metric graph
        for(int i=0; i < img_ids.size(); i++)
        {
          img_set.insert(img_ids[i]);
        }

        // if(component.first == 518)
        // {
        //   LOG(INFO)<<"img_set size: "<<img_set.size();
        //   for(auto img : img_set)
        //     LOG(INFO)<<img;
        // }

        // Clustering images
        ImageCluster metric_cluster;
        metric_cluster.image_ids = img_ids;//current metric graph

        const std::unordered_map<ImagePair, TwoViewInfo>& metric_pairs =
            metric_graph_.TwoViewGeometries(); // all metric edges
        for (const auto& view_pair_it : metric_pairs) {
          const ImagePair view_pair = view_pair_it.first;
          if(img_set.count(view_pair.first) != 0
           && (img_set.count(view_pair.second) != 0))//当前连通区域内的边
           {
              metric_cluster.edges[view_pair] = view_pair_it.second.visibility_score;
           }          
        }

        // if(component.first == 518)
        // {
        //   LOG(INFO)<<"edges size: "<<metric_cluster.edges.size();
        //   for(auto edge : metric_cluster.edges)
        //     LOG(INFO)<<edge.first.first<<", "<<edge.first.second;
        // }

        metric_clustering_ = std::unique_ptr<ImageClustering>(
            new ImageClustering(metric_cluster_options, metric_cluster));
        std::unordered_map<int, int> labels;
        int subcluster_num = 0;
        metric_clustering_->CutByMetric(labels,subcluster_num); //基于metric edge将连通的metric graph划分为较小的clusters

        // if(component.first==824)
        // {
        //   LOG(INFO)<<"here 824:";
        //   for(auto label:labels)
        //   {
        //     LOG(INFO)<<label.first<<", "<<label.second;
        //   }
        // }

        std::unordered_map<size_t, std::vector<image_t>> subsubclusters;
        for(auto label : labels)
        {
          size_t subsubcluster_id = subcluster_id + label.second;//当前label对应的id
          std::vector<image_t> &subsubclusterimages = subsubclusters[subsubcluster_id];
          subsubclusterimages.push_back(label.first);

          // if(subsubcluster_id==1050)
          // {
          //   LOG(INFO)<<subcluster_id<<", ("<<label.first<<", "<<label.second<<"), ("
          //                     <<component.first<<", "<<component.second.size()<<")";
          // }


          // CHECK_EQ(basic_labels.count(label.first),0);//basic_labels中不应有当前connected graph中的影像id
          CHECK_LT(label.second, subcluster_num); //cluster id 小于 cluster num
          // basic_labels[label.first] = subcluster_id + label.second; //每一个小cluster都累加上之前的cluster id，往后顺延
        }
        divided_components.insert(subsubclusters.begin(),subsubclusters.end());
        subcluster_id += subcluster_num; //cluster id 加上当前graph分出的子集数量
      }
  }
  int subclusters_num = subcluster_id;
  LOG(INFO)<<"Total subclusters num: "<<subclusters_num;
  components = divided_components;
}

void DistributedMapperController::MergeSmallComponents(std::unordered_map<size_t, std::vector<image_t>>& components)
{
  int raw_components_size = components.size();
  std::unordered_map<image_t,size_t> subcluster_labels;
  std::set<size_t> small_components_ids;

  for(auto component : components)
  {
    CHECK_LT(component.first, raw_components_size);
    for(auto img_id : component.second)
    {
      if(subcluster_labels.count(img_id)!= 0)
      {
        LOG(INFO)<<"img_id: "<<img_id<< " subcluster id: "<<component.first;

      }
        
      CHECK_EQ(subcluster_labels.count(img_id), 0);
      subcluster_labels.insert(std::make_pair(img_id,component.first));// image id 和 subcluster id
    }
    if(component.second.size()==1) //subcluster内只有一张，需要后续merge
    {
      small_components_ids.insert(component.first);
    }      
  }
  if(small_components_ids.size()==0)
  {
    return;
  }


  
  std::set<image_t> small_component_img_ids;//被单独分出来的images
  for(auto small_components_id : small_components_ids)
  {
    CHECK_EQ(components[small_components_id].size(),1);
    image_t img_id = components[small_components_id][0];
    small_component_img_ids.insert(img_id);

    components.erase(small_components_id); //先在components中删掉这些小的subclusters
  }

  std::unordered_map<image_t,image_t> best_matching_img_map;//单独影像和它最佳匹配影像之间的关联map
  std::unordered_map<image_t,int> best_matches_number_map;//单独影像和它最佳匹配影像之间的匹配数
  const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
      view_graph_.TwoViewGeometries();
  for (const auto& view_pair_it : view_pairs) {
    const ImagePair view_pair = view_pair_it.first;

    //当前edge的两条边都在当前的subclusters影像中（没有被之前的filter过滤掉，否则就算找到了也没用，因为它的最佳匹配已经没了）
    if(small_component_img_ids.count(view_pair.first) && subcluster_labels.count(view_pair.second))
    {
      if(best_matches_number_map.count(view_pair.first)==0) //之前没有找到过这个img
      {
        best_matching_img_map[view_pair.first] = view_pair.second;
        best_matches_number_map[view_pair.first] = view_pair_it.second.visibility_score;
      }
      else if(best_matches_number_map[view_pair.first] < view_pair_it.second.visibility_score) //之前找到过这个img，但是现在的匹配更多
      {
        best_matching_img_map[view_pair.first] = view_pair.second;
        best_matches_number_map[view_pair.first] = view_pair_it.second.visibility_score;
      }
    }

    if(small_component_img_ids.count(view_pair.second) && subcluster_labels.count(view_pair.first))
    {
      if(best_matches_number_map.count(view_pair.second)==0) //之前没有找到过这个img
      {
        best_matching_img_map[view_pair.second] = view_pair.first;
        best_matches_number_map[view_pair.second] = view_pair_it.second.visibility_score;
      }
      else if(best_matches_number_map[view_pair.second] < view_pair_it.second.visibility_score) //之前找到过这个img，但是现在的匹配更多
      {
        best_matching_img_map[view_pair.second] = view_pair.first;
        best_matches_number_map[view_pair.second] = view_pair_it.second.visibility_score;
      }
    }
  }

  // LOG(INFO)<<"best matches: 5449, "<<best_matching_img_map[5449];
  // LOG(INFO)<<"best matches: 5446, "<<best_matching_img_map[5446];
  // LOG(INFO)<<"best matches: 5447, "<<best_matching_img_map[5447];


  std::set<image_t> solo_matched_images;
  std::unordered_map<image_t,int> best_matches_subcluster_map;//单独影像和它最佳匹配影像所在的类
  for(auto best_img_map : best_matching_img_map)
  {   
    
    image_t solo_image = best_img_map.first;
    image_t match_image = best_img_map.second;


    // bool log =false;
    // if(solo_image == 1770 || solo_image == 1769 || solo_image == 1766 || solo_image == 1777
    //    || match_image == 1770 ||match_image == 1769 ||match_image == 1766 ||match_image == 1777)
    // {
    //   LOG(INFO)<<"("<<solo_image<<", "<<match_image<<")";
    //   log = true;
    // }

    // if(log)
    // {
    //   LOG(INFO)<<"merged this image? "<<solo_matched_images.count(solo_image);
    //   LOG(INFO)<<"best match image not in big cluster? "<<best_matching_img_map.count(match_image);
    //   LOG(INFO)<<"best match has been merged? "<<best_matches_subcluster_map.count(match_image);
    // }

    if(solo_matched_images.count(solo_image)!=0)//当前影像已经被之前的影像merge掉了
      continue;

    if(best_matching_img_map.count(match_image)==0)//则当前独立影像的最佳匹配影像在较大的subcluster中，直接添加进去
    {
      int best_subcluster_id = subcluster_labels[match_image];
      subcluster_labels[solo_image] = best_subcluster_id;
      std::vector<image_t>& subcluster_imgs = components[best_subcluster_id];
      subcluster_imgs.push_back(solo_image);
      best_matches_subcluster_map.insert(std::make_pair(solo_image,best_subcluster_id));//保存之前是独立影像，但现在已经被merge的影像subcluster
    }
    else //当前独立影像的最佳匹配影像也是独立影像(或者至少曾经是)
    {
      if(best_matches_subcluster_map.count(match_image)!=0)//最佳匹配已经被merged
      {
        int best_subcluster_id = best_matches_subcluster_map[match_image];
        std::vector<image_t>& subcluster_imgs = components[best_subcluster_id];
        subcluster_imgs.push_back(solo_image);
        best_matches_subcluster_map.insert(std::make_pair(solo_image,best_subcluster_id));
      }
      else
      {
        std::vector<image_t> subcluster_imgs;
        subcluster_imgs.push_back(solo_image);
        subcluster_imgs.push_back(match_image);
        CHECK_EQ(components.count(raw_components_size),0);
        components.insert(std::make_pair(raw_components_size,subcluster_imgs));//添加新的subcluster
        best_matches_subcluster_map.insert(std::make_pair(match_image,raw_components_size));
        best_matches_subcluster_map.insert(std::make_pair(solo_image,raw_components_size));
        solo_matched_images.insert(match_image);
        // solo_matched_images.insert(solo_image);
        // LOG(INFO)<<"here inserted "<<match_image;
        raw_components_size++;
      }
    }    
  }
}

void DistributedMapperController::OutputComponents(std::string filepath, std::unordered_map<size_t, std::vector<image_t>> components)
{
  LOG(INFO) << "Output results to " << filepath;
  std::ofstream subcluster_out_file(filepath,std::ios::out);
  if (!subcluster_out_file.is_open()) {
    LOG(WARNING) << filepath<<" can't be opened!";
  }


  for(auto component : components)
  {
    subcluster_out_file << "subcluster id: " <<component.first<<" with "<< component.second.size()<<" imgs"<<std::endl;
    for(auto img : component.second)
    {
      subcluster_out_file << "\t"<<img<<std::endl;
    }
  }
}



void DistributedMapperController::ClusteringScenes() {
  // Clustering images
  ImageCluster image_cluster;
  image_cluster.image_ids = view_graph_.ImageIds();

  const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
      view_graph_.TwoViewGeometries();
  for (const auto& view_pair_it : view_pairs) {
    const ImagePair view_pair = view_pair_it.first;
    image_cluster.edges[view_pair] = view_pair_it.second.visibility_score;
  }

  image_clustering_ = std::unique_ptr<ImageClustering>(
      new ImageClustering(clustering_options_, image_cluster));
  image_clustering_->Cut();
  image_clustering_->Expand();
  image_clustering_->OutputClusteringSummary();

  inter_clusters_ = image_clustering_->GetInterClusters();
  intra_clusters_ = image_clustering_->GetIntraClusters();
  for (auto cluster : inter_clusters_) {
    cluster.ShowInfo();
  }
}




// bool DistributedMapperController::ReconstructSubclusters() {
//   // if()

//   // Choose the global rotation estimation type.
//   std::unique_ptr<RotationEstimator> rotation_estimator;
//   switch (options_.global_rotation_estimator_type) {
//     case GlobalRotationEstimatorType::ROBUST_L1L2: {
//       RobustRotationEstimator::Options robust_rotation_estimator_options;
//       rotation_estimator.reset(
//           new RobustRotationEstimator(robust_rotation_estimator_options));
//       break;
//     }
//     case GlobalRotationEstimatorType::NONLINEAR: {
//       rotation_estimator.reset(new NonlinearRotationEstimator());
//       break;
//     }
//     default: {
//       LOG(FATAL) << "Invalid type of global rotation estimation chosen.";
//       break;
//     }
//   }

//   std::unordered_map<image_t, Eigen::Vector3d> local_rotations;

//   std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
//   view_graph_.TwoViewGeometries();

//   for(int i =0; i<inter_clusters_.size(); i++ )
//   {
//     ImageCluster inter_cluster = inter_clusters_[i];
//     int cluster_id = inter_cluster.cluster_id;
//     std::vector<image_t> subcluster_ids = inter_cluster.image_ids;
//     sort(subcluster_ids.begin(),subcluster_ids.end());
//     LOG(INFO)<<"current cluster id: "<<cluster_id<<" with "<<subcluster_ids.size()<<" subclusters"<<std::endl;
//     for(int j = 0; j<subcluster_ids.size(); j++)
//     {
//       std::unordered_map<ImagePair, TwoViewInfo> subcluster_view_pairs;
//       image_t subcluster_id = subcluster_ids[j];
//       std::vector<image_t> img_ids = subcluster_img_map_[subcluster_id];
//       LOG(INFO)<<"\t reconstruct subcluster id: "<<subcluster_id<<" with "<<img_ids.size()<<" images"<<std::endl;
//       for(int p = 0; p<img_ids.size(); p++)
//       {
//         image_t imgid0 = img_ids[p];
//         for(int k=p+1; k<img_ids.size(); k++)
//         {
//           image_t imgid1 = img_ids[k];
//           const ImagePair view_pair = imgid0 < imgid1
//                                       ? ImagePair(imgid0, imgid1)
//                                       : ImagePair(imgid1, imgid0);
//           if(view_pairs.count(view_pair)==1)
//             subcluster_view_pairs.insert(std::make_pair(view_pair,view_pairs[view_pair]));
//         }
//       }
//       local_rotations.clear();
//       bool success = rotation_estimator->EstimateRotations(subcluster_view_pairs, &local_rotations);
//       if (!success) {
//       LOG(INFO)<<"subcluster rotation average failed !!!";
//       }
//     }
//   }  

//   LOG(INFO)<<"Subclusters rotation computed !!!";



//   // // Filter view pairs based on the relative rotation and the
//   // // estimated global orientations.
//   // FilterViewPairsFromOrientation(
//   //     rotations_, options_.max_relative_rotation_difference_degrees,
//   //     view_graph_.TwoViewGeometries(), database_);
//   // UpdateViewGraph();
//   // Remove any disconnected views from the estimation.
//   // PrintHeading1("Extracting Largest Connected Component...");
//   // LOG(INFO) << "Total image number: " << view_graph_.ImageIds().size();
//   // view_graph_.TwoviewGeometriesToImagePairs();
//   // view_graph_.ExtractLargestCC();
//   // LOG(INFO) << "image number in largest cc: " << view_graph_.ImageIds().size();

//   return true;
// }

void DistributedMapperController::ReconstructSubclusters(
    std::unordered_map<size_t, ReconstructionManager>& reconstruction_managers,
    std::vector<Reconstruction*>& reconstructions) {
  // Determine the number of workers and threads per worker
  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
  const std::unordered_map<image_t, std::string>& image_id_to_name =
      view_graph_.ImageIdToName();
  const int kDefaultNumWorkers = 8;
  // const int num_eff_workers =
  //     std::min(static_cast<int>(inter_clusters_.size()),
  //              std::min(kDefaultNumWorkers, num_eff_threads));
    const int num_eff_workers =
               std::min(kDefaultNumWorkers, num_eff_threads);
  const int num_threads_per_worker =
      std::max(1, num_eff_threads / num_eff_workers);

  // Start reconstructing the bigger clusters first for resource usage.
  const auto cmp = [](const ImageCluster& cluster1,
                      const ImageCluster& cluster2) {
    return cluster1.image_ids.size() > cluster2.image_ids.size();
  };
  std::sort(inter_clusters_.begin(), inter_clusters_.end(), cmp);

  // Start the reconstruction workers.
  // reconstruction_managers.reserve(inter_clusters_.size());
  reconstruction_managers.reserve(subcluster_img_map_.size());

  std::unordered_map<size_t, DatabaseCache> cluster_database_caches;
  // cluster_database_caches.reserve(inter_clusters_.size());
  cluster_database_caches.reserve(subcluster_img_map_.size());

  IncrementalMapperOptions local_mapper_options;
  // for (size_t k = 0; k < inter_clusters_.size(); k++) {
  //   std::vector<std::string> image_name_list;
  //   image_name_list.reserve(inter_clusters_[k].image_ids.size());

  //   for (const auto image_id : inter_clusters_[k].image_ids) {
  //     image_name_list.push_back(image_id_to_name.at(image_id));
  //   }
  //   std::sort(image_name_list.begin(), image_name_list.end());

  //   std::unordered_set<std::string> image_name_set(image_name_list.begin(),
  //                                                  image_name_list.end());
  //   cluster_database_caches[k].Load(
  //       database_, static_cast<size_t>(local_mapper_options.min_num_matches),
  //       local_mapper_options.ignore_watermarks, image_name_set);
  // }

    for (size_t k = 0; k < subcluster_img_map_.size(); k++) {
    std::vector<std::string> image_name_list;
    image_name_list.reserve(subcluster_img_map_[k].size());

    for (const auto image_id : subcluster_img_map_[k]) {
      image_name_list.push_back(image_id_to_name.at(image_id));
    }
    std::sort(image_name_list.begin(), image_name_list.end());

    std::unordered_set<std::string> image_name_set(image_name_list.begin(),
                                                   image_name_list.end());
    cluster_database_caches[k].Load(
        database_, static_cast<size_t>(local_mapper_options.min_num_matches),
        local_mapper_options.ignore_watermarks, image_name_set);
  }

// #pragma omp parallel for shared(inter_clusters_, reconstruction_managers, cluster_database_caches, mapper_options_)
  for (size_t k = 0; k < subcluster_img_map_.size(); k++) {
    const auto& cluster = subcluster_img_map_[k];

    IncrementalMapperOptions custom_options = mapper_options_;
    custom_options.max_model_overlap = 20;
    custom_options.init_num_trials = options_.init_num_trials;
    custom_options.num_threads = num_threads_per_worker;
    custom_options.extract_colors = true;

    for (const auto image_id : cluster) {
      custom_options.image_names.insert(image_id_to_name.at(image_id));
    }

    IncrementalMapperController mapper(&custom_options,
                                       &reconstruction_managers[k]);
    mapper.SetDatabaseCache(&cluster_database_caches[k]);

    mapper.Start();
    mapper.Wait();
  }
  std::cout<<"pause"<<std::endl;
  getchar();

  for (size_t k = 0; k < inter_clusters_.size(); k++) {
    auto& recon_manager = reconstruction_managers.at(k);
    for (size_t i = 0; i < recon_manager.Size(); i++) {
      reconstructions.push_back(&recon_manager.Get(i));
    }
  }
#ifdef __DEBUG__
  LOG(INFO) << "Extracting colors for local maps.";
  for (Reconstruction* reconstruction : reconstructions) {
    ExtractColors(options_.image_path, reconstruction);
  }
  ExportUntransformedLocalRecons(reconstructions);
  LOG(INFO) << "Local reconstructions are exported to " << options_.output_path;
#endif
}

void DistributedMapperController::ReconstructClusters(
    std::unordered_map<size_t, ReconstructionManager>& reconstruction_managers,
    std::vector<Reconstruction*>& reconstructions) {
  // Determine the number of workers and threads per worker

  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
  const std::unordered_map<image_t, std::string>& image_id_to_name =
      view_graph_.ImageIdToName();
  LOG(INFO)<<"image_id_to_name size "<<image_id_to_name.size();
  const int kDefaultNumWorkers = 8;
  const int num_eff_workers =
      std::min(static_cast<int>(inter_clusters_.size()),
               std::min(kDefaultNumWorkers, num_eff_threads));
  const int num_threads_per_worker =
      std::max(1, num_eff_threads / num_eff_workers);
  LOG(INFO)<<"num_threads_per_worker: "<<num_threads_per_worker;

  // Start reconstructing the bigger clusters first for resource usage.
  const auto cmp = [](const ImageCluster& cluster1,
                      const ImageCluster& cluster2) {
    return cluster1.image_ids.size() > cluster2.image_ids.size();
  };
  std::sort(inter_clusters_.begin(), inter_clusters_.end(), cmp);
  LOG(INFO)<<inter_clusters_.size();

  // Start the reconstruction workers.
  reconstruction_managers.reserve(inter_clusters_.size());

  std::unordered_map<size_t, DatabaseCache> cluster_database_caches;
  cluster_database_caches.reserve(inter_clusters_.size());

  IncrementalMapperOptions local_mapper_options;
  for (size_t k = 0; k < inter_clusters_.size(); k++) {
    std::vector<std::string> image_name_list;
    // image_name_list.reserve(inter_clusters_[k].image_ids.size());

    for (const auto subcluster_id : inter_clusters_[k].image_ids) {
      std::vector<image_t> img_ids = subcluster_img_map_[subcluster_id];
      for(int i=0; i<img_ids.size(); i++)
      {
        image_t img_id = img_ids[i];
        image_name_list.push_back(image_id_to_name.at(img_id));
      }      
    }
    std::sort(image_name_list.begin(), image_name_list.end());

    std::unordered_set<std::string> image_name_set(image_name_list.begin(),
                                                   image_name_list.end());
    cluster_database_caches[k].Load(
        database_, static_cast<size_t>(local_mapper_options.min_num_matches),
        local_mapper_options.ignore_watermarks, image_name_set);
  }

// #pragma omp parallel for shared(inter_clusters_, reconstruction_managers, cluster_database_caches, mapper_options_)
  for (size_t k = 0; k < inter_clusters_.size(); k++) {
    const auto& cluster = inter_clusters_[k];

    IncrementalMapperOptions custom_options = mapper_options_;
    custom_options.max_model_overlap = 20;
    custom_options.init_num_trials = options_.init_num_trials;
    custom_options.num_threads = num_threads_per_worker;
    custom_options.extract_colors = true;

    for (const auto image_id : cluster.image_ids) {
      custom_options.image_names.insert(image_id_to_name.at(image_id));
    }

    IncrementalMapperController mapper(&custom_options,
                                       &reconstruction_managers[k]);
    mapper.SetDatabaseCache(&cluster_database_caches[k]);

    mapper.Start();
    mapper.Wait();
  }
  // std::cout<<"pause"<<std::endl;
  // getchar();

  for (size_t k = 0; k < inter_clusters_.size(); k++) {
    auto& recon_manager = reconstruction_managers.at(k);
    for (size_t i = 0; i < recon_manager.Size(); i++) {
      reconstructions.push_back(&recon_manager.Get(i));
    }
  }
#ifdef __DEBUG__
  LOG(INFO) << "Extracting colors for local maps.";
  for (Reconstruction* reconstruction : reconstructions) {
    ExtractColors(options_.image_path, reconstruction);
  }
  ExportUntransformedLocalRecons(reconstructions);
  LOG(INFO) << "Local reconstructions are exported to " << options_.output_path;
#endif
}


void DistributedMapperController::ReconstructPartitions(
    std::unordered_map<size_t, ReconstructionManager>& reconstruction_managers,
    std::vector<Reconstruction*>& reconstructions) {
  // Determine the number of workers and threads per worker
  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
  const std::unordered_map<image_t, std::string>& image_id_to_name =
      view_graph_.ImageIdToName();
      LOG(INFO)<<"image names map size: "<<image_id_to_name.size();
  const int kDefaultNumWorkers = 8;
  const int num_eff_workers =
      std::min(static_cast<int>(inter_clusters_.size()),
               std::min(kDefaultNumWorkers, num_eff_threads));
  const int num_threads_per_worker =
      std::max(1, num_eff_threads / num_eff_workers);

    // LOG(INFO)<<"Check";
    // for(int k=0; k<inter_clusters_.size(); k++)
    // {
    //   LOG(INFO)<<"k = "<<k<<" inter_clusters_ index: "<<inter_clusters_[k].cluster_id<<" with img size: "<<inter_clusters_[k].image_ids.size();
    // }

  // Start reconstructing the bigger clusters first for resource usage.
  const auto cmp = [](const ImageCluster& cluster1,
                      const ImageCluster& cluster2) {
    return cluster1.image_ids.size() > cluster2.image_ids.size();
  };
  std::sort(inter_clusters_.begin(), inter_clusters_.end(), cmp);
  //   //临时修改cmp为由小到大，快速确认是否可以初始化
  //   const auto cmp1 = [](const ImageCluster& cluster1,
  //                     const ImageCluster& cluster2) {
  //   return cluster1.image_ids.size() < cluster2.image_ids.size();
  // };
  // std::sort(inter_clusters_.begin(), inter_clusters_.end(), cmp1);

  // Start the reconstruction workers.
  reconstruction_managers.reserve(inter_clusters_.size());

  std::unordered_map<size_t, DatabaseCache> cluster_database_caches;
  cluster_database_caches.reserve(inter_clusters_.size());
// LOG(INFO)<<"get here1?";
  IncrementalMapperOptions local_mapper_options;

  CHECK_EQ(inter_clusters_.size(),all_best_edges_.size());

  
  for (size_t k = 0; k < inter_clusters_.size(); k++)
  {
    if(options_.test_reconstruct_one_cluster_id!=-1)//my code for only recon a certain cluster
      k = options_.test_reconstruct_one_cluster_id;
    
    // size_t k=59;
    std::vector<std::string> image_name_list;
    image_name_list.reserve(inter_clusters_[k].image_ids.size());
// LOG(INFO)<<"get here2?";
    for (const auto image_id : inter_clusters_[k].image_ids) {
      // LOG(INFO)<<"image_id: "<<image_id;
      image_name_list.push_back(image_id_to_name.at(image_id));
      // LOG(INFO)<<"image_id: "<<image_id<<" with name "<<image_id_to_name.at(image_id);
    }
        // LOG(INFO)<<image_name_list.size();
    std::sort(image_name_list.begin(), image_name_list.end());
//  LOG(INFO)<<image_name_list.size();
    std::unordered_set<std::string> image_name_set(image_name_list.begin(),
                                                  image_name_list.end());
    // LOG(INFO)<<image_name_set.size();
    // cluster_database_caches[k].Load(
    //     database_, static_cast<size_t>(local_mapper_options.min_num_matches),
    //     local_mapper_options.ignore_watermarks, image_name_set);

    int cluster_id = inter_clusters_[k].cluster_id;
    std::unordered_set<ImagePair> best_pairs;
    for(auto pair_itr:all_best_edges_[cluster_id])
    {
      best_pairs.insert(pair_itr.first);
    }
    LOG(INFO)<<"cluster id: "<<cluster_id<<" has "<<best_pairs.size()<<" best edges";

        cluster_database_caches[k].LoadWithFilteredEdges(
        database_, static_cast<size_t>(local_mapper_options.min_num_matches),
        local_mapper_options.ignore_watermarks, image_name_set, best_pairs);

        //// Load simulation data created by points and lines
        //         cluster_database_caches[k].LoadSimuData(
        // database_, static_cast<size_t>(local_mapper_options.min_num_matches),
        // local_mapper_options.ignore_watermarks, image_name_set, best_pairs);

        //     cluster_database_caches[k].Load(
        // database_, static_cast<size_t>(local_mapper_options.min_num_matches),
        // local_mapper_options.ignore_watermarks, image_name_set);

    if(options_.test_reconstruct_one_cluster_id!=-1)//my code for only recon a certain cluster
      break;
  }



  for (size_t k = 0; k < inter_clusters_.size(); k++) {
    
    if(options_.test_reconstruct_one_cluster_id!=-1)//my code for only recon a certain cluster
      k = options_.test_reconstruct_one_cluster_id;
    
    const auto& cluster = inter_clusters_[k];
    LOG(INFO)<<"sorted inter_clusters_ id: "<<k<<" size: "<<cluster.image_ids.size()<<" original cluster id: "<<cluster.cluster_id;
    
    if(options_.test_reconstruct_one_cluster_id!=-1)//my code for only recon a certain cluster
      break;
  }

  // exit(EXIT_SUCCESS);

// #pragma omp parallel for shared(inter_clusters_, reconstruction_managers, cluster_database_caches, mapper_options_)
  for (size_t k = 0; k < inter_clusters_.size(); k++)
  {
    if(options_.test_reconstruct_one_cluster_id!=-1)//my code for only recon a certain cluster
      k = options_.test_reconstruct_one_cluster_id;

    // size_t k=59;
    const auto& cluster = inter_clusters_[k];



    IncrementalMapperOptions custom_options = mapper_options_;
    custom_options.max_model_overlap = 20;
    custom_options.init_num_trials = options_.init_num_trials;
    custom_options.num_threads = num_threads_per_worker;
    custom_options.extract_colors = true;

    for (const auto image_id : cluster.image_ids) {
      custom_options.image_names.insert(image_id_to_name.at(image_id));
    }

    IncrementalMapperController mapper(&custom_options,
                                       &reconstruction_managers[k]);
    mapper.SetDatabaseCache(&cluster_database_caches[k]);


    // LOG(INFO)<<"k = "<<k<<" inter_clusters_ index: "<<inter_clusters_[k].cluster_id<<" with img size: "<<custom_options.image_names.size();
    
    
    // if(inter_clusters_[k].cluster_id==12)
    {
      std::string cluster_path = options_.output_path + "/cluster_images_" + std::to_string(k) + ".txt";
      LOG(INFO) << "Output reconstruction results to " << cluster_path;
      std::ofstream cluster_out_file(cluster_path,std::ios::out);
      if (!cluster_out_file.is_open()) {
        LOG(WARNING) << cluster_path<<" can't be opened!";
      }
      cluster_out_file<<"k = "<<k<<" inter_clusters_ index: "<<inter_clusters_[k].cluster_id<<" with img size: "<<custom_options.image_names.size()<<std::endl;
      LOG(INFO)<<"k = "<<k<<" inter_clusters_ index: "<<inter_clusters_[k].cluster_id<<" with img size: "<<custom_options.image_names.size();
      for(auto name:custom_options.image_names)
      {
        cluster_out_file<<name<<std::endl;
      }
    }   
    
    
    mapper.Start();
    mapper.Wait();

    if(options_.test_reconstruct_one_cluster_id!=-1)//my code for only recon a certain cluster
      break;
  }
  // std::cout<<"pause"<<std::endl;
  // getchar();

  std::map<int,int> recon_cluster_map;
  int recon_id = 0;
  for (size_t k = 0; k < inter_clusters_.size(); k++)
  {
    if(options_.test_reconstruct_one_cluster_id!=-1)//my code for only recon a certain cluster
      k = options_.test_reconstruct_one_cluster_id;

    // size_t k=42;
    auto& recon_manager = reconstruction_managers.at(k);
    LOG(INFO)<<"inter_clusters_["<<k<<"] has "<<recon_manager.Size()<<" reconstructions!";
    for (size_t i = 0; i < recon_manager.Size(); i++) {      
      reconstructions.push_back(&recon_manager.Get(i));
      LOG(INFO)<<recon_manager.Get(i).RegImageIds().size();
      recon_cluster_map.insert(std::make_pair(recon_id,k));
      recon_id++;
    }
    
    if(options_.test_reconstruct_one_cluster_id!=-1)//my code for only recon a certain cluster
      break;
  }

  for(auto itr : recon_cluster_map)
  {
    LOG(INFO)<<"recon id: "<<itr.first<<" and its cluster id: "<<itr.second;
  }


#ifdef __DEBUG__
  LOG(INFO) << "Extracting colors for local maps.";
  for (Reconstruction* reconstruction : reconstructions) {
    ExtractColors(options_.image_path, reconstruction);
  }
  ExportUntransformedLocalRecons(reconstructions);
  LOG(INFO) << "Local reconstructions are exported to " << options_.output_path;
#endif
  if(reconstructions.size()==1)
    exit(EXIT_SUCCESS);

  if(options_.test_reconstruct_one_cluster_id!=-1)
    exit(EXIT_SUCCESS);
}

void DistributedMapperController::MergeClusters(
    std::vector<Reconstruction*>& reconstructions, const int num_eff_threads,
    Node& anchor_node) {
  if (reconstructions.size() > 1) {
    SfMAligner::AlignOptions align_options;
    align_options.image_path = options_.image_path;
    SfMAligner sfm_aligner(reconstructions, align_options);

    LOG(INFO)<<"reonstructions before align...";
    for(int i=0;i<reconstructions.size();i++)
    {
      Reconstruction* recon_itr = reconstructions[i];
      LOG(INFO)<<"recon index: "<<i;
      LOG(INFO)<<"recon cameras:"<<recon_itr->NumCameras()<<" images:"<<recon_itr->NumImages()<<" points:"<<recon_itr->NumPoints3D();
    }
    if (sfm_aligner.Align()) {
      LOG(INFO)<<"Align success";
      anchor_node = reconstructions.size()-1;//sfm_aligner.GetAnchorNode();
      separators_ = sfm_aligner.GetSeparators();
      my_reconstructions_ = sfm_aligner.seq_merg_reconstructions_;
    }
    LOG(INFO)<<"sfm_aligner.reconstructions_ after align...";
    for(int i=0;i<sfm_aligner.seq_merg_reconstructions_.size();i++)
    {
      // Reconstruction* recon_itr = reconstructions[i];
      LOG(INFO)<<"recon index: "<<i;
      LOG(INFO)<<"recon cameras:"<<sfm_aligner.seq_merg_reconstructions_[i].NumCameras()<<" images:"<<sfm_aligner.seq_merg_reconstructions_[i].NumImages()<<" points:"<<sfm_aligner.seq_merg_reconstructions_[i].NumPoints3D();
    }
    LOG(INFO)<<"reonstructions after align...";
    for(int i=0;i<my_reconstructions_.size();i++)
    {
      // Reconstruction* recon_itr = reconstructions[i];
      LOG(INFO)<<"recon index: "<<i;
      LOG(INFO)<<"recon cameras:"<<my_reconstructions_[i].NumCameras()<<" images:"<<my_reconstructions_[i].NumImages()<<" points:"<<my_reconstructions_[i].NumPoints3D();
    }

    CHECK_NE(anchor_node.id, -1);
    // CHECK_NOTNULL(reconstructions[anchor_node.id]);
    CHECK_NOTNULL(reconstructions[0]);

    // const size_t filtered_points_num = 0;
    for(int i=0;i<my_reconstructions_.size();i++)
    {
      my_reconstructions_[i].FilterAllPoints3D(
            options_.filter_max_reproj_error, options_.filter_min_tri_angle);
    }
        LOG(INFO)<<"reonstructions after filter...";
    for(int i=0;i<my_reconstructions_.size();i++)
    {
      // Reconstruction* recon_itr = reconstructions[i];
      LOG(INFO)<<"recon index: "<<i;
      LOG(INFO)<<"recon cameras:"<<my_reconstructions_[i].NumCameras()<<" images:"<<my_reconstructions_[i].NumImages()<<" points:"<<my_reconstructions_[i].NumPoints3D();
    }

    // LOG(INFO) << filtered_points_num << " 3D points are filtered";
    LOG(INFO) <<"Filtered all points 3d";

// #ifdef __DEBUG__
//     // Reading un-transformed reconstruction partitions.
//     std::vector<Reconstruction*> trans_recons;
//     CHECK_EQ(IsPartialReconsExist(trans_recons), true);

//     std::vector<Sim3> sim3_to_anchor = sfm_aligner.GetSim3ToAnchor();
//     for (uint i = 0; i < reconstructions.size(); i++) {
//       if (static_cast<int>(i) == anchor_node.id) continue;

//       Sim3 sim3 = sim3_to_anchor[i];
//       Eigen::Vector4d qvec;
//       ceres::RotationMatrixToQuaternion(sim3.R.data(), qvec.data());
//       SimilarityTransform3 tform(sim3.s, qvec, sim3.t);
//       trans_recons[i]->Transform(tform);
//     }
// #endif
  } else {  // Only one cluster.
    anchor_node.id = 0;
  }
  LOG(INFO)<<"Align cluster id for each image";
  // Assign cluster id for each image.
  for (size_t i = 0; i < intra_clusters_.size(); i++) {
    const ImageCluster& intra_cluster = intra_clusters_[i];
    const std::vector<image_t>& image_ids = intra_cluster.image_ids;

    for (auto image_id : image_ids) {
      if (reconstructions[anchor_node.id]->ExistsImage(image_id)) {
        Image& image = reconstructions[anchor_node.id]->Image(image_id);
        image.SetClusterId(i);
      }
    }
  }
}


void DistributedMapperController::IncrementalSequentialSfM(
    std::vector<Reconstruction*>& reconstructions, const int num_eff_threads,
    Node& anchor_node) {

  //改成迭代merge的版本
  SfMAligner::AlignOptions align_options;    
  align_options.image_path = options_.image_path;
  LOG(INFO)<<"IncrementalSequentialSfM...";
  int old_recon_size = reconstructions.size();

  std::vector<Reconstruction*> temp_reconstructions = reconstructions;
  std::vector<Reconstruction> temp_seq_recons;
  while (reconstructions.size()>1) {
    
    SfMAligner sfm_aligner(temp_reconstructions, align_options);
    sfm_aligner.seq_merg_reconstructions_ = temp_seq_recons;

    if (sfm_aligner.MyAlign())
    {
      LOG(INFO)<<"Align itr success";
      LOG(INFO)<<"has "<<sfm_aligner.reconstructions_.size()<<" reconstructions";
      for(int i=0;i<sfm_aligner.reconstructions_.size();i++)
      {
        Reconstruction* recon_itr = sfm_aligner.reconstructions_[i];
        LOG(INFO)<<"recon index: "<<i;
        LOG(INFO)<<"recon cameras:"<<recon_itr->NumCameras()<<" images:"<<recon_itr->NumImages()<<" points:"<<recon_itr->NumPoints3D();
      }

      temp_reconstructions = sfm_aligner.reconstructions_;
      temp_seq_recons = sfm_aligner.seq_merg_reconstructions_;
      anchor_node = sfm_aligner.GetAnchorNode();
      separators_ = sfm_aligner.GetSeparators();

      // this->AdjustGlobalBundle();
    }

    if(temp_reconstructions.size()==old_recon_size || temp_reconstructions.size()==1)
      break;
    else
      old_recon_size = temp_reconstructions.size();
  }

  if (temp_reconstructions.size()==1) {
    LOG(INFO)<<"Align success";
    SfMAligner sfm_aligner(temp_reconstructions, align_options);
    // anchor_node = reconstructions.size()-1;//sfm_aligner.GetAnchorNode();
    anchor_node = sfm_aligner.GetAnchorNode();
    separators_ = sfm_aligner.GetSeparators();
    // my_reconstructions_ = sfm_aligner.seq_merg_reconstructions_;
    // my_reconstructions_.push_back(*(temp_reconstructions[0]));
    my_reconstructions_ = temp_seq_recons;
  }


  // if (reconstructions.size() > 1) {
  //   SfMAligner::AlignOptions align_options;
  //   align_options.image_path = options_.image_path;
  //   SfMAligner sfm_aligner(reconstructions, align_options);

  //   LOG(INFO)<<"reonstructions before align...";
  //   for(int i=0;i<reconstructions.size();i++)
  //   {
  //     Reconstruction* recon_itr = reconstructions[i];
  //     LOG(INFO)<<"recon index: "<<i;
  //     LOG(INFO)<<"recon cameras:"<<recon_itr->NumCameras()<<" images:"<<recon_itr->NumImages()<<" points:"<<recon_itr->NumPoints3D();
  //   }
  //   if (sfm_aligner.MyAlign()) {
  //     LOG(INFO)<<"Align success";
  //     anchor_node = reconstructions.size()-1;//sfm_aligner.GetAnchorNode();
  //     separators_ = sfm_aligner.GetSeparators();
  //     my_reconstructions_ = sfm_aligner.seq_merg_reconstructions_;
  //   }
  //   LOG(INFO)<<"sfm_aligner.reconstructions_ after align...";
  //   for(int i=0;i<sfm_aligner.seq_merg_reconstructions_.size();i++)
  //   {
  //     // Reconstruction* recon_itr = reconstructions[i];
  //     LOG(INFO)<<"recon index: "<<i;
  //     LOG(INFO)<<"recon cameras:"<<sfm_aligner.seq_merg_reconstructions_[i].NumCameras()<<" images:"<<sfm_aligner.seq_merg_reconstructions_[i].NumImages()<<" points:"<<sfm_aligner.seq_merg_reconstructions_[i].NumPoints3D();
  //   }
  //   LOG(INFO)<<"reonstructions after align...";
  //   for(int i=0;i<my_reconstructions_.size();i++)
  //   {
  //     // Reconstruction* recon_itr = reconstructions[i];
  //     LOG(INFO)<<"recon index: "<<i;
  //     LOG(INFO)<<"recon cameras:"<<my_reconstructions_[i].NumCameras()<<" images:"<<my_reconstructions_[i].NumImages()<<" points:"<<my_reconstructions_[i].NumPoints3D();
  //   }

    // CHECK_NE(anchor_node.id, -1);
    // CHECK_NOTNULL(reconstructions[anchor_node.id]);
    CHECK_NOTNULL(reconstructions[0]);

    // const size_t filtered_points_num = 0;
    for(int i=0;i<my_reconstructions_.size();i++)
    {
      my_reconstructions_[i].FilterAllPoints3D(
            options_.filter_max_reproj_error, options_.filter_min_tri_angle);
    }
        LOG(INFO)<<"reonstructions after filter...";
    for(int i=0;i<my_reconstructions_.size();i++)
    {
      // Reconstruction* recon_itr = reconstructions[i];
      LOG(INFO)<<"recon index: "<<i;
      LOG(INFO)<<"recon cameras:"<<my_reconstructions_[i].NumCameras()<<" images:"<<my_reconstructions_[i].NumImages()<<" points:"<<my_reconstructions_[i].NumPoints3D();
    }

    // LOG(INFO) << filtered_points_num << " 3D points are filtered";
    LOG(INFO) <<"Filtered all points 3d";

// #ifdef __DEBUG__
//     // Reading un-transformed reconstruction partitions.
//     std::vector<Reconstruction*> trans_recons;
//     CHECK_EQ(IsPartialReconsExist(trans_recons), true);

//     std::vector<Sim3> sim3_to_anchor = sfm_aligner.GetSim3ToAnchor();
//     for (uint i = 0; i < reconstructions.size(); i++) {
//       if (static_cast<int>(i) == anchor_node.id) continue;

//       Sim3 sim3 = sim3_to_anchor[i];
//       Eigen::Vector4d qvec;
//       ceres::RotationMatrixToQuaternion(sim3.R.data(), qvec.data());
//       SimilarityTransform3 tform(sim3.s, qvec, sim3.t);
//       trans_recons[i]->Transform(tform);
//     }
// #endif
  // } else {  // Only one cluster.
  //   anchor_node.id = 0;
  // }

  // LOG(INFO)<<"Align cluster id for each image";
  // // Assign cluster id for each image.
  // for (size_t i = 0; i < intra_clusters_.size(); i++) {
  //   const ImageCluster& intra_cluster = intra_clusters_[i];
  //   const std::vector<image_t>& image_ids = intra_cluster.image_ids;

  //   for (auto image_id : image_ids) {
  //     if (reconstructions[anchor_node.id]->ExistsImage(image_id)) {
  //       Image& image = reconstructions[anchor_node.id]->Image(image_id);
  //       image.SetClusterId(i);
  //     }
  //   }
  // }
}


void DistributedMapperController::IncrementalSequentialSfM(
    std::vector<Reconstruction*>& reconstructions,
    std::unordered_map<size_t, ReconstructionManager>& reconstruction_managers,
    const int num_eff_threads) {
  LOG(INFO) << "Sub-reconstructions size: " << reconstructions.size();
  Node anchor_node;
  IncrementalSequentialSfM(reconstructions, num_eff_threads, anchor_node);

  LOG(INFO) << "Adding all seq merged reconstructions...";
  // reconstructions[anchor_node.id]->ShowReconInfo();

  for(int i=0;i<my_reconstructions_.size();i++)
  {
    LOG(INFO)<<"cameras:"<<my_reconstructions_[i].NumCameras();
  }

  // Insert a new reconstruction manager for merged cluster.
  // reconstruction_managers.clear();
  
  // my_seq_merged_reconstructions_.reserve(reconstructions.size());
  my_seq_merged_reconstructions2_.reserve(my_reconstructions_.size());

  
  const size_t k = reconstruction_managers.size();
  size_t k_last = 0;
  LOG(INFO)<<"k:"<<k;
  for(int i=0;i<my_reconstructions_.size();i++)
  {    
    k_last = reconstruction_managers.size();
    LOG(INFO)<<"k1:"<<k_last;
    auto& reconstruction_manager = reconstruction_managers[k_last];
    reconstruction_manager.Add();
    LOG(INFO)<<reconstruction_manager.Size();
    reconstruction_manager.Get(reconstruction_manager.Size() - 1) =
        my_reconstructions_[i];
    LOG(INFO)<<"cameras:"<<my_reconstructions_[i].NumCameras()<<" images:"<<my_reconstructions_[i].NumImages()<<" points:"<<my_reconstructions_[i].NumPoints3D();
  }




  LOG(INFO) <<"reconstruction_managers size: "<<reconstruction_managers.size();
  LOG(INFO)<<"k:"<<k;
  for(auto& itr:reconstruction_managers)
  {
    LOG(INFO)<<itr.first;
  }


  LOG(INFO) << "Erasing clusters...";
  for (size_t i = 0; i < k; i++) {
    LOG(INFO)<<"erase index: "<<i;
    reconstruction_managers.erase(i);
    LOG(INFO)<<"index"<<i<<"erased";
  }
  // CHECK_EQ(reconstruction_managers.size(), 1);

  // for(size_t i= k_last-k; i<k_last;i++)
  for(size_t i= k; i<k_last+1;i++)
  {
    //每次都是最后一个recon
    // *reconstruction_manager_ = std::move(reconstruction_managers[i+1]);
    // LOG(INFO)<<"cameras:"<<reconstruction_manager_->Get(0).NumCameras()<<" images:"<<reconstruction_manager_->Get(0).NumImages()<<" points:"<<reconstruction_manager_->Get(0).NumPoints3D();
    // my_seq_merged_reconstructions_.push_back(reconstruction_manager_);

    //segment fault
    // LOG(INFO)<<"cameras:"<<reconstruction_managers[i+1].Get(0).NumCameras()<<" images:"<<reconstruction_managers[i+1].Get(0).NumImages()<<" points:"<<reconstruction_managers[i+1].Get(0).NumPoints3D();
    // my_seq_merged_reconstructions_.push_back(&reconstruction_managers[i+1]);
    
    //无法编译通过
    // ReconstructionManager recon = reconstruction_managers[i+1];
    // LOG(INFO)<<"cameras:"<<recon.Get(0).NumCameras()<<" images:"<<recon.Get(0).NumImages()<<" points:"<<recon.Get(0).NumPoints3D();
    // my_seq_merged_reconstructions_.push_back(&recon);    

    // ReconstructionManager recon = reconstruction_managers[i+1];
    LOG(INFO)<<"cameras:"<<reconstruction_managers[i].Get(0).NumCameras()<<" images:"<<reconstruction_managers[i].Get(0).NumImages()<<" points:"<<reconstruction_managers[i].Get(0).NumPoints3D();
    my_seq_merged_reconstructions2_.push_back(std::move(reconstruction_managers[i]));  
  }

}







void DistributedMapperController::MergeClusters(
    std::vector<Reconstruction*>& reconstructions,
    std::unordered_map<size_t, ReconstructionManager>& reconstruction_managers,
    const int num_eff_threads) {
  LOG(INFO) << "Sub-reconstructions size: " << reconstructions.size();
  Node anchor_node;
  MergeClusters(reconstructions, num_eff_threads, anchor_node);

  LOG(INFO) << "Adding all seq merged reconstructions...";
  // reconstructions[anchor_node.id]->ShowReconInfo();

  for(int i=0;i<my_reconstructions_.size();i++)
  {
    LOG(INFO)<<"cameras:"<<my_reconstructions_[i].NumCameras();
  }

  // Insert a new reconstruction manager for merged cluster.
  // reconstruction_managers.clear();
  
  // my_seq_merged_reconstructions_.reserve(reconstructions.size());
  my_seq_merged_reconstructions2_.reserve(my_reconstructions_.size());

  
  const size_t k = reconstruction_managers.size();
  size_t k_last = 0;
  LOG(INFO)<<"k:"<<k;
  for(int i=0;i<my_reconstructions_.size();i++)
  {    
    k_last = reconstruction_managers.size();
    LOG(INFO)<<"k1:"<<k_last;
    auto& reconstruction_manager = reconstruction_managers[k_last];
    reconstruction_manager.Add();
    LOG(INFO)<<reconstruction_manager.Size();
    reconstruction_manager.Get(reconstruction_manager.Size() - 1) =
        my_reconstructions_[i];
    LOG(INFO)<<"cameras:"<<my_reconstructions_[i].NumCameras()<<" images:"<<my_reconstructions_[i].NumImages()<<" points:"<<my_reconstructions_[i].NumPoints3D();
  }




  LOG(INFO) <<"reconstruction_managers size: "<<reconstruction_managers.size();
  LOG(INFO)<<"k:"<<k;
  for(auto& itr:reconstruction_managers)
  {
    LOG(INFO)<<itr.first;
  }


  LOG(INFO) << "Erasing clusters...";
  for (size_t i = 0; i < k; i++) {
    LOG(INFO)<<"erase index: "<<i;
    reconstruction_managers.erase(i);
    LOG(INFO)<<"index"<<i<<"erased";
  }
  // CHECK_EQ(reconstruction_managers.size(), 1);

  // for(size_t i= k_last-k; i<k_last;i++)
  for(size_t i= k; i<k_last+1;i++)
  {
    //每次都是最后一个recon
    // *reconstruction_manager_ = std::move(reconstruction_managers[i+1]);
    // LOG(INFO)<<"cameras:"<<reconstruction_manager_->Get(0).NumCameras()<<" images:"<<reconstruction_manager_->Get(0).NumImages()<<" points:"<<reconstruction_manager_->Get(0).NumPoints3D();
    // my_seq_merged_reconstructions_.push_back(reconstruction_manager_);

    //segment fault
    // LOG(INFO)<<"cameras:"<<reconstruction_managers[i+1].Get(0).NumCameras()<<" images:"<<reconstruction_managers[i+1].Get(0).NumImages()<<" points:"<<reconstruction_managers[i+1].Get(0).NumPoints3D();
    // my_seq_merged_reconstructions_.push_back(&reconstruction_managers[i+1]);
    
    //无法编译通过
    // ReconstructionManager recon = reconstruction_managers[i+1];
    // LOG(INFO)<<"cameras:"<<recon.Get(0).NumCameras()<<" images:"<<recon.Get(0).NumImages()<<" points:"<<recon.Get(0).NumPoints3D();
    // my_seq_merged_reconstructions_.push_back(&recon);    

    // ReconstructionManager recon = reconstruction_managers[i+1];
    LOG(INFO)<<"cameras:"<<reconstruction_managers[i].Get(0).NumCameras()<<" images:"<<reconstruction_managers[i].Get(0).NumImages()<<" points:"<<reconstruction_managers[i].Get(0).NumPoints3D();
    my_seq_merged_reconstructions2_.push_back(std::move(reconstruction_managers[i]));  
  }

}

bool DistributedMapperController::Triangulate() {
  Reconstruction& global_recon = reconstruction_manager_->Get(0);

  IncrementalTriangulator triangulator(global_recon.GetCorrespondenceGraph(),
                                       &global_recon);
  IncrementalTriangulator::Options triangulate_options;
  for (const image_t image_id : separators_) {
    triangulator.TriangulateImage(triangulate_options, image_id);
  }

  return true;
}

bool DistributedMapperController::AdjustGlobalBundle() {
  Reconstruction& reconstruction =
      reconstruction_manager_->Get(reconstruction_manager_->Size() - 1);
  std::vector<point3D_t> separator_points3D;
  for (auto image_id : separators_) {
    if (!reconstruction.ExistsImage(image_id)) {
      continue;
    }
    const std::vector<point3D_t> observed_points3D =
        reconstruction.Image(image_id).ObservedPoints3D();
    separator_points3D.insert(separator_points3D.begin(),
                              observed_points3D.begin(),
                              observed_points3D.end());
  }
  std::sort(separator_points3D.begin(), separator_points3D.end());
  separator_points3D.erase(
      separator_points3D.begin(),
      std::unique(separator_points3D.begin(), separator_points3D.end()));
  LOG(INFO) << "Total separators: " << separators_.size();
  LOG(INFO) << "Total points3D observed by separators: "
            << separator_points3D.size();
  const double separator_rmse_without_ba =
      reconstruction.ComputeMeanReprojectionError(separator_points3D);
  const double all_rmse_without_ba =
      reconstruction.ComputeMeanReprojectionError({});

  BundleAdjustmentOptions ba_options = this->GlobalBundleAdjustment();
  // ba_options.solver_options.num_threads = num_eff_threads;

  Reconstruction& global_recon = reconstruction_manager_->Get(0);

  const std::vector<image_t>& reg_image_ids = global_recon.RegImageIds();

  CHECK_GE(reg_image_ids.size(), 2)
      << "At least two images must be "
         "registered for global bundle-adjustment";

  // Avoid degeneracies in bundle adjustment.
  global_recon.FilterObservationsWithNegativeDepth();

  // Configure bundle adjustment
  BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);
  }

  // Fix 7-DOFs of the bundle adjustment problem.
  ba_config.SetConstantPose(reg_image_ids[0]);
  ba_config.SetConstantTvec(reg_image_ids[1], {0});

  EIGEN_STL_UMAP(point3D_t, Point3D) points3d = global_recon.Points3D();
  std::unordered_set<point3D_t> tracks_to_optimize;
  if (options_.select_tracks_for_bundle_adjustment) {
    SelectGoodTracksForBundleAdjustment(
        global_recon, options_.long_track_length_threshold,
        options_.image_grid_cell_size_pixels,
        options_.min_num_optimized_tracks_per_view, &tracks_to_optimize);

    LOG(INFO) << tracks_to_optimize.size() << " / "
              << global_recon.NumPoints3D() << "(" << std::setprecision(2)
              << (float)tracks_to_optimize.size() /
                     (float)global_recon.NumPoints3D()
              << "%) tracks are selected for bundle adjustment.";

    for (const auto& point3d : points3d) {
      if (tracks_to_optimize.count(point3d.first) != 0) {
        ba_config.AddVariablePoint(point3d.first);
      } else {
        ba_config.AddConstantPoint(point3d.first);
      }
    }
  } else {
    for (const auto& point3d : points3d) {
      ba_config.AddVariablePoint(point3d.first);
    }
  }

  // Run bundle adjustment.
  BundleAdjuster bundle_adjuster(ba_options, ba_config);
  if (!bundle_adjuster.Solve(&global_recon)) {
    return false;
  }

  LOG(INFO) << "RMSE for separators(before BA): " << separator_rmse_without_ba;
  LOG(INFO) << "RMSE for all images(before BA): " << all_rmse_without_ba;
  LOG(INFO) << "RMSE for separators(after BA): "
            << reconstruction.ComputeMeanReprojectionError(separator_points3D);
  LOG(INFO) << "RMSE for all images(after BA): "
            << reconstruction.ComputeMeanReprojectionError({});
  LOG(INFO) << "Selected " << tracks_to_optimize.size() << " to optimize.";
  LOG(INFO) << "Total tracks: " << points3d.size();

  // Normalize scene for numerical stability and
  // to avoid large scale changes in viewer.
  global_recon.Normalize();

  return true;
}



bool DistributedMapperController::AdjustGBAForOneMergedRecon(Reconstruction& reconstruction) {
  // Reconstruction& reconstruction =
  //     reconstruction_manager_->Get(reconstruction_manager_->Size() - 1);

  std::vector<point3D_t> separator_points3D;
  for (auto image_id : separators_) {
    if (!reconstruction.ExistsImage(image_id)) {
      continue;
    }
    const std::vector<point3D_t> observed_points3D =
        reconstruction.Image(image_id).ObservedPoints3D();
    separator_points3D.insert(separator_points3D.begin(),
                              observed_points3D.begin(),
                              observed_points3D.end());
  }
  std::sort(separator_points3D.begin(), separator_points3D.end());
  separator_points3D.erase(
      separator_points3D.begin(),
      std::unique(separator_points3D.begin(), separator_points3D.end()));
  LOG(INFO) << "Total separators: " << separators_.size();
  LOG(INFO) << "Total points3D observed by separators: "
            << separator_points3D.size();
  const double separator_rmse_without_ba =
      reconstruction.ComputeMeanReprojectionError(separator_points3D);
  const double all_rmse_without_ba =
      reconstruction.ComputeMeanReprojectionError({});

  BundleAdjustmentOptions ba_options = this->GlobalBundleAdjustment();
  // ba_options.solver_options.num_threads = num_eff_threads;

  Reconstruction& global_recon = reconstruction_manager_->Get(0);

  const std::vector<image_t>& reg_image_ids = global_recon.RegImageIds();

  CHECK_GE(reg_image_ids.size(), 2)
      << "At least two images must be "
         "registered for global bundle-adjustment";

  // Avoid degeneracies in bundle adjustment.
  global_recon.FilterObservationsWithNegativeDepth();

  // Configure bundle adjustment
  BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);
  }

  // Fix 7-DOFs of the bundle adjustment problem.
  ba_config.SetConstantPose(reg_image_ids[0]);
  ba_config.SetConstantTvec(reg_image_ids[1], {0});

  EIGEN_STL_UMAP(point3D_t, Point3D) points3d = global_recon.Points3D();
  std::unordered_set<point3D_t> tracks_to_optimize;
  if (options_.select_tracks_for_bundle_adjustment) {
    SelectGoodTracksForBundleAdjustment(
        global_recon, options_.long_track_length_threshold,
        options_.image_grid_cell_size_pixels,
        options_.min_num_optimized_tracks_per_view, &tracks_to_optimize);

    LOG(INFO) << tracks_to_optimize.size() << " / "
              << global_recon.NumPoints3D() << "(" << std::setprecision(2)
              << (float)tracks_to_optimize.size() /
                     (float)global_recon.NumPoints3D()
              << "%) tracks are selected for bundle adjustment.";

    for (const auto& point3d : points3d) {
      if (tracks_to_optimize.count(point3d.first) != 0) {
        ba_config.AddVariablePoint(point3d.first);
      } else {
        ba_config.AddConstantPoint(point3d.first);
      }
    }
  } else {
    for (const auto& point3d : points3d) {
      ba_config.AddVariablePoint(point3d.first);
    }
  }

  // Run bundle adjustment.
  BundleAdjuster bundle_adjuster(ba_options, ba_config);
  if (!bundle_adjuster.Solve(&global_recon)) {
    return false;
  }

  LOG(INFO) << "RMSE for separators(before BA): " << separator_rmse_without_ba;
  LOG(INFO) << "RMSE for all images(before BA): " << all_rmse_without_ba;
  LOG(INFO) << "RMSE for separators(after BA): "
            << reconstruction.ComputeMeanReprojectionError(separator_points3D);
  LOG(INFO) << "RMSE for all images(after BA): "
            << reconstruction.ComputeMeanReprojectionError({});
  LOG(INFO) << "Selected " << tracks_to_optimize.size() << " to optimize.";
  LOG(INFO) << "Total tracks: " << points3d.size();

  // Normalize scene for numerical stability and
  // to avoid large scale changes in viewer.
  global_recon.Normalize();

  return true;
}

void DistributedMapperController::ExportUntransformedLocalRecons(
    const std::vector<Reconstruction*>& reconstructions) const {
  for (size_t i = 0; i < reconstructions.size(); ++i) {
    const std::string reconstruction_path =
        JoinPaths(options_.output_path, "partition" + std::to_string(i));
    CreateDirIfNotExists(reconstruction_path);
    reconstructions[i]->Write(reconstruction_path);
  }
}

bool DistributedMapperController::GlobalRotationAveraging() {
  // Initialize the orientation estimations by walking along the maximum
  // spanning tree.
  // TODO: (chenyu)
  const auto& image_ids = view_graph_.ImageIds();
  for (auto image_id : image_ids) {
    rotations_[image_id] = Eigen::Vector3d::Zero();
  }
  // Clear pairs that are not included in largest connected components.
  std::unordered_set<ImagePair> view_pairs_to_remove;
  std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
      view_graph_.TwoViewGeometries();
  for (auto view_pair_it : view_pairs) {
    if (rotations_.count(view_pair_it.first.first) == 0 ||
        rotations_.count(view_pair_it.first.second) == 0) {
      view_pairs_to_remove.insert(view_pair_it.first);
    }
  }
  for (const ImagePair view_pair : view_pairs_to_remove) {
    view_pairs.erase(view_pair);
    database_.DeleteMatches(view_pair.first, view_pair.second);
    database_.DeleteInlierMatches(view_pair.first, view_pair.second);
  }

  // Choose the global rotation estimation type.
  std::unique_ptr<RotationEstimator> rotation_estimator;
  switch (options_.global_rotation_estimator_type) {
    case GlobalRotationEstimatorType::ROBUST_L1L2: {
      RobustRotationEstimator::Options robust_rotation_estimator_options;
      rotation_estimator.reset(
          new RobustRotationEstimator(robust_rotation_estimator_options));
      break;
    }
    case GlobalRotationEstimatorType::NONLINEAR: {
      rotation_estimator.reset(new NonlinearRotationEstimator());
      break;
    }
    default: {
      LOG(FATAL) << "Invalid type of global rotation estimation chosen.";
      break;
    }
  }
  LOG(INFO)<<"rotations size: "<<rotations_.size();
  LOG(INFO)<<"view_pairs size: "<<view_pairs.size();
  bool success = rotation_estimator->EstimateRotations(view_pairs, &rotations_);

  if (!success) {
    return false;
  }

  // Filter view pairs based on the relative rotation and the
  // estimated global orientations.  
  FilterViewPairsFromOrientation(
      rotations_, options_.max_relative_rotation_difference_degrees,
      view_graph_.TwoViewGeometries(), database_);
  UpdateViewGraph();
  // Remove any disconnected views from the estimation.
  PrintHeading1("Extracting Largest Connected Component...");
  LOG(INFO) << "Total image number: " << view_graph_.ImageIds().size();
  view_graph_.TwoviewGeometriesToImagePairs();
  view_graph_.ExtractLargestCC();
  LOG(INFO) << "image number in largest cc: " << view_graph_.ImageIds().size();

  return true;
}

void DistributedMapperController::UpdateViewGraph() {
  const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs =
      view_graph_.TwoViewGeometries();
  for (auto view_pair_it : view_pairs) {
    const ImagePair view_pair = view_pair_it.first;
    TwoViewInfo& two_view_info = view_pair_it.second;

    const Eigen::Vector3d R1 = rotations_[view_pair.first];
    const Eigen::Vector3d R2 = rotations_[view_pair.second];
    Eigen::Vector3d R12 = RelativeRotationFromTwoRotations(R1, R2);
    two_view_info.rotation_2 = R12;
  }
}

}  // namespace DAGSfM