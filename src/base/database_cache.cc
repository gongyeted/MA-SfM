// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "base/database_cache.h"

#include <unordered_set>

#include "feature/utils.h"
#include "util/string.h"
#include "util/timer.h"

namespace colmap {

DatabaseCache::DatabaseCache() {}

void DatabaseCache::AddCamera(const class Camera& camera) {
  CHECK(!ExistsCamera(camera.CameraId()));
  cameras_.emplace(camera.CameraId(), camera);
}

void DatabaseCache::AddImage(const class Image& image) {
  CHECK(!ExistsImage(image.ImageId()));
  images_.emplace(image.ImageId(), image);
  correspondence_graph_.AddImage(image.ImageId(), image.NumPoints2D());
}

void DatabaseCache::Load(const Database& database, const size_t min_num_matches,
                         const bool ignore_watermarks,
                         const std::unordered_set<std::string>& image_names) {
  //////////////////////////////////////////////////////////////////////////////
  // Load cameras
  //////////////////////////////////////////////////////////////////////////////

  Timer timer;

  timer.Start();
  std::cout << "Loading cameras..." << std::flush;

  {
    const std::vector<class Camera> cameras = database.ReadAllCameras();
    cameras_.reserve(cameras.size());
    for (const auto& camera : cameras) {
      cameras_.emplace(camera.CameraId(), camera);
    }
  }

  std::cout << StringPrintf(" %d in %.3fs", cameras_.size(),
                            timer.ElapsedSeconds())
            << std::endl;

  //////////////////////////////////////////////////////////////////////////////
  // Load matches
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Loading matches..." << std::flush;

  std::vector<image_pair_t> image_pair_ids;
  std::vector<TwoViewGeometry> two_view_geometries;
  database.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

  std::cout << StringPrintf(" %d in %.3fs", image_pair_ids.size(),
                            timer.ElapsedSeconds())
            << std::endl;

  auto UseInlierMatchesCheck = [min_num_matches, ignore_watermarks](
                                   const TwoViewGeometry& two_view_geometry) {
    return static_cast<size_t>(two_view_geometry.inlier_matches.size()) >=
               min_num_matches &&
           (!ignore_watermarks ||
            two_view_geometry.config != TwoViewGeometry::WATERMARK);
  };

  //////////////////////////////////////////////////////////////////////////////
  // Load images
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Loading images..." << std::flush;

  std::unordered_set<image_t> image_ids;

  {
    const std::vector<class Image> images = database.ReadAllImages();

    // Determines for which images data should be loaded.
    if (image_names.empty()) {
      for (const auto& image : images) {
        image_ids.insert(image.ImageId());
      }
    } else {
      for (const auto& image : images) {
        if (image_names.count(image.Name()) > 0) {
          image_ids.insert(image.ImageId());
        }
      }
    }

    // Collect all images that are connected in the correspondence graph.
    std::unordered_set<image_t> connected_image_ids;
    connected_image_ids.reserve(image_ids.size());
    for (size_t i = 0; i < image_pair_ids.size(); ++i) {
      if (UseInlierMatchesCheck(two_view_geometries[i])) {
        image_t image_id1;
        image_t image_id2;
        Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
        if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
          connected_image_ids.insert(image_id1);
          connected_image_ids.insert(image_id2);
        }
      }
    }

    // Load images with correspondences and discard images without
    // correspondences, as those images are useless for SfM.
    images_.reserve(connected_image_ids.size());
    for (const auto& image : images) {
      if (image_ids.count(image.ImageId()) > 0 &&
          connected_image_ids.count(image.ImageId()) > 0) {
        images_.emplace(image.ImageId(), image);
        const FeatureKeypoints keypoints =
            database.ReadKeypoints(image.ImageId());
        const std::vector<Eigen::Vector2d> points =
            FeatureKeypointsToPointsVector(keypoints);
        images_[image.ImageId()].SetPoints2D(points);
      }
    }

    std::cout << StringPrintf(" %d in %.3fs (connected %d)", images.size(),
                              timer.ElapsedSeconds(),
                              connected_image_ids.size())
              << std::endl;

              LOG(INFO) << images.size()<< " in "<< timer.ElapsedSeconds()<<" s with connected "<<
                              connected_image_ids.size();
  }

  //////////////////////////////////////////////////////////////////////////////
  // Build correspondence graph
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Building correspondence graph..." << std::flush;

  for (const auto& image : images_) {
    correspondence_graph_.AddImage(image.first, image.second.NumPoints2D());
  }

  size_t num_ignored_image_pairs = 0;
  LOG(INFO)<<"image_pair_ids.size(): "<<image_pair_ids.size();
  size_t num_loaded_edges = 0;
  for (size_t i = 0; i < image_pair_ids.size(); ++i) {
    if (UseInlierMatchesCheck(two_view_geometries[i])) {
      image_t image_id1;
      image_t image_id2;
      Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
      if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
        num_loaded_edges++;
        correspondence_graph_.AddCorrespondences(
            image_id1, image_id2, two_view_geometries[i].inlier_matches);
      } else {
        num_ignored_image_pairs += 1;
      }
    } else {
      num_ignored_image_pairs += 1;
    }
  }
  LOG(INFO)<<"num_loaded_edges: "<<num_loaded_edges;
  correspondence_graph_.Finalize();

  // Set number of observations and correspondences per image.
  for (auto& image : images_) {
    image.second.SetNumObservations(
        correspondence_graph_.NumObservationsForImage(image.first));
    image.second.SetNumCorrespondences(
        correspondence_graph_.NumCorrespondencesForImage(image.first));
  }

  std::cout << StringPrintf(" in %.3fs (ignored %d)", timer.ElapsedSeconds(),
                            num_ignored_image_pairs)
            << std::endl;
}




void DatabaseCache::LoadWithFilteredEdges(const Database& database, const size_t min_num_matches,
                         const bool ignore_watermarks,
                         const std::unordered_set<std::string>& image_names,
                         const std::unordered_set<ImagePair>& best_pairs) {
  //////////////////////////////////////////////////////////////////////////////
  // Load cameras
  //////////////////////////////////////////////////////////////////////////////
  
  Timer timer;

  timer.Start();
  std::cout << "Loading cameras..." << std::flush;

  {
    const std::vector<class Camera> cameras = database.ReadAllCameras();
    cameras_.reserve(cameras.size());
    for (const auto& camera : cameras) {
      cameras_.emplace(camera.CameraId(), camera);
    }
  }

  std::cout << StringPrintf(" %d in %.3fs", cameras_.size(),
                            timer.ElapsedSeconds())
            << std::endl;

  //////////////////////////////////////////////////////////////////////////////
  // Load matches
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Loading matches..." << std::flush;

  std::vector<image_pair_t> image_pair_ids;
  std::vector<TwoViewGeometry> two_view_geometries;
  database.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

  std::cout << StringPrintf(" %d in %.3fs", image_pair_ids.size(),
                            timer.ElapsedSeconds())
            << std::endl;

  auto UseInlierMatchesCheck = [min_num_matches, ignore_watermarks](
                                   const TwoViewGeometry& two_view_geometry) {
    return static_cast<size_t>(two_view_geometry.inlier_matches.size()) >=
               min_num_matches &&
           (!ignore_watermarks ||
            two_view_geometry.config != TwoViewGeometry::WATERMARK);
  };

  //////////////////////////////////////////////////////////////////////////////
  // Load images
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Loading images..." << std::flush;

  std::unordered_set<image_t> image_ids;

  {
    const std::vector<class Image> images = database.ReadAllImages();

    // Determines for which images data should be loaded.
    if (image_names.empty()) {
      for (const auto& image : images) {
        image_ids.insert(image.ImageId());
      }
    } else {
      for (const auto& image : images) {
        if (image_names.count(image.Name()) > 0) {
          image_ids.insert(image.ImageId());
        }
      }
    }

    // Collect all images that are connected in the correspondence graph.
    std::unordered_set<image_t> connected_image_ids;
    connected_image_ids.reserve(image_ids.size());
    for (size_t i = 0; i < image_pair_ids.size(); ++i) {
      if (UseInlierMatchesCheck(two_view_geometries[i])) {
        image_t image_id1;
        image_t image_id2;
        Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
        if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
          connected_image_ids.insert(image_id1);
          connected_image_ids.insert(image_id2);
        }
      }
    }

    // Load images with correspondences and discard images without
    // correspondences, as those images are useless for SfM.
    images_.reserve(connected_image_ids.size());
    for (const auto& image : images) {
      if (image_ids.count(image.ImageId()) > 0 &&
          connected_image_ids.count(image.ImageId()) > 0) {
        images_.emplace(image.ImageId(), image);
        const FeatureKeypoints keypoints =
            database.ReadKeypoints(image.ImageId());
        const std::vector<Eigen::Vector2d> points =
            FeatureKeypointsToPointsVector(keypoints);
        images_[image.ImageId()].SetPoints2D(points);
      }
    }

    std::cout << StringPrintf(" %d in %.3fs (connected %d)", images.size(),
                              timer.ElapsedSeconds(),
                              connected_image_ids.size())
              << std::endl;

              LOG(INFO) << images.size()<< " in "<< timer.ElapsedSeconds()<<" s with connected "<<
                              connected_image_ids.size();
  }

  //////////////////////////////////////////////////////////////////////////////
  // Build correspondence graph
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Building correspondence graph..." << std::flush;


  for (const auto& image : images_) {
    correspondence_graph_.AddImage(image.first, image.second.NumPoints2D());
    // if(image.first==4846 || image.first==4839)
    // {      
    //   LOG(INFO)<<"image :"<< image.first << " num_observations: "<<image.second.NumPoints2D();
    // }
  }
  // LOG(INFO)<< correspondence_graph_.ExistsImage(4839);
  // LOG(INFO)<<"NumObservationsForImage 4839 :"<< correspondence_graph_.NumObservationsForImage(4839);
  
  size_t num_ignored_image_pairs = 0;
  LOG(INFO)<<"image_pair_ids.size(): "<<image_pair_ids.size();
  size_t num_loaded_edges = 0;
  for (size_t i = 0; i < image_pair_ids.size(); ++i) {
    if (UseInlierMatchesCheck(two_view_geometries[i])) {
      image_t image_id1;
      image_t image_id2;

      Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
            // LOG(INFO)<<"img1, img2:"<<image_id1<<" "<<image_id2;

      if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
        //add best edge filter
        ImagePair pair1 = std::make_pair(image_id1, image_id2);
        ImagePair pair2 = std::make_pair(image_id2, image_id1);
        if(best_pairs.count(pair1)!=0 || best_pairs.count(pair2)!=0)
        {
          num_loaded_edges++;
          //  LOG(INFO)<<"AddCorrespondences: "<<image_id1<<" "<<image_id2<<" "<<two_view_geometries[i].inlier_matches.size();
          correspondence_graph_.AddCorrespondences(
              image_id1, image_id2, two_view_geometries[i].inlier_matches);
        }
      } else {
        num_ignored_image_pairs += 1;
      }
    } else {
      num_ignored_image_pairs += 1;
    }
  }
  LOG(INFO)<<"num_loaded_edges: "<<num_loaded_edges;
  // LOG(INFO)<< correspondence_graph_.ExistsImage(4839);
  // LOG(INFO)<<"NumObservationsForImage 4839 :"<< correspondence_graph_.NumObservationsForImage(4839);
  correspondence_graph_.Finalize();
  // Set number of observations and correspondences per image.
  // LOG(INFO)<<"images_ size: "<<images_.size();
  for (auto& image : images_) {
    if(correspondence_graph_.ExistsImage(image.first))
    {
      // LOG(INFO)<<"begin";
      // LOG(INFO)<<"SetNumObservations: img id: "<<image.first;
      // LOG(INFO)<<"SetNumObservations: img id: "<<image.first<<" NumObservationsForImage: "<<correspondence_graph_.NumObservationsForImage(image.first);
      image.second.SetNumObservations(
          correspondence_graph_.NumObservationsForImage(image.first));
          // LOG(INFO)<<"SetNumCorrespondences: img id: "<<image.first<<" NumCorrespondencesForImage: "<<correspondence_graph_.NumObservationsForImage(image.first);
      image.second.SetNumCorrespondences(
          correspondence_graph_.NumCorrespondencesForImage(image.first));
          // LOG(INFO)<<"SetNumCorrespondences end";
    }
  }
  std::cout << StringPrintf(" in %.3fs (ignored %d)", timer.ElapsedSeconds(),
                            num_ignored_image_pairs)
            << std::endl;
}


void DatabaseCache::LoadSimuData(const Database& database, const size_t min_num_matches,
                         const bool ignore_watermarks,
                         const std::unordered_set<std::string>& image_names,
                         const std::unordered_set<ImagePair>& best_pairs) {
  //////////////////////////////////////////////////////////////////////////////
  // Load cameras
  //////////////////////////////////////////////////////////////////////////////


  int img_size = 600;
  int point_size = 446;
  double f = 1000.0;
  int cx = 540;
  int cy = 480;
  int width = 2*cx;
  int height = 2*cy;

  LOG(INFO)<<"Reading data file...";
  std::string rootpath = "/home/gy/Redundancy_simulation/data_simulation/bin/";
  std::vector<std::vector<Eigen::Vector2d>> all_observations; 

  LOG(INFO)<<"Reading observations file...";
  for(int i = 0; i < img_size; i++)
  {
    std::string obs_path = rootpath + "sigma2.0/all_points_" + std::to_string(i) + ".txt";
    std::ifstream fin(obs_path, std::ios::in);
    if (!fin) {
      std::cerr << "Error: unable to open obsv file " << obs_path;
      return;
    };

    std::vector<Eigen::Vector2d> obs;
    for (int j = 0; j < point_size; ++j) {
      int p3d_id;
      double x,y;
      fin >> p3d_id >> x >> y;//注意此处需修改生成的数据，将每行前4维改成id
      // FscanfOrDie(fptr, "%d", &p3d_id); 
      // FscanfOrDie(fptr, "%lf", &x);
      // FscanfOrDie(fptr, "%lf", &y);
      Eigen::Vector2d ob(x,y);
      obs.push_back(ob);
    }

    all_observations.push_back(obs);
    fin.close();
  }


  LOG(INFO)<<"Reading poses file...";
  std::vector<Eigen::Vector4d> Qvecs;
  std::vector<Eigen::Vector3d> tvecs;
  // for(int i = 0; i < img_size; i++)
  {
    std::string poses_path = rootpath + "cam_pose_tum.txt";
    std::ifstream fin(poses_path, std::ios::in);
    if (!fin) {
      std::cerr << "Error: unable to open pose file " << poses_path;
      return;
    };


    for (int i = 0; i < img_size; ++i) {

      double time,tx,ty,tz,qx,qy,qz,qw;
      fin >> time >> tx >> ty >> tz
          >> qx >> qy >> qz >> qw;
      Eigen::Vector4d Qvec(qw, qx, qy, qz);
      Eigen::Vector3d tvec(tx, ty, tz);

      Qvecs.push_back(Qvec);
      tvecs.push_back(tvec);
    }
    fin.close();
  }





  Timer timer;

  timer.Start();
  std::cout << "Loading cameras..." << std::flush;





  //set cameras
  std::vector<double> cam_params = {(double)f, (double)cx , (double)cy, (double)0.0}; // camera parameters gt
  for(int i=0; i<img_size; i++)
  {
    // Camera cam(i,2,1080,960,false); 
    class Camera cam;
    cam.SetParams(cam_params);
    cam.SetCameraId(i);
    cam.SetModelId(2);//SIMPLE_RADIAL
    cam.SetWidth(width);
    cam.SetHeight(height);
     
    cameras_.emplace(i, cam);
  }



  // {
    // const std::vector<class Camera> cameras = database.ReadAllCameras();
  //   cameras_.reserve(cameras.size());
  //   for (const auto& camera : cameras) {
  //     cameras_.emplace(camera.CameraId(), camera);
  //   }
  // }

  std::cout << StringPrintf(" %d in %.3fs", cameras_.size(),
                            timer.ElapsedSeconds())
            << std::endl;

  //////////////////////////////////////////////////////////////////////////////
  // Load matches
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Loading matches..." << std::flush;

  // std::vector<image_pair_t> image_pair_ids;
  // std::vector<TwoViewGeometry> two_view_geometries;
  // database.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

  // std::cout << StringPrintf(" %d in %.3fs", image_pair_ids.size(),
  //                           timer.ElapsedSeconds())
  //           << std::endl;

  // auto UseInlierMatchesCheck = [min_num_matches, ignore_watermarks](
  //                                  const TwoViewGeometry& two_view_geometry) {
  //   return static_cast<size_t>(two_view_geometry.inlier_matches.size()) >=
  //              min_num_matches &&
  //          (!ignore_watermarks ||
  //           two_view_geometry.config != TwoViewGeometry::WATERMARK);
  // };

  //////////////////////////////////////////////////////////////////////////////
  // Load images
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Loading images..." << std::flush;

  // std::unordered_set<image_t> image_ids;
  std::vector<image_t> image_ids;
  {
    // const std::vector<class Image> images = database.ReadAllImages();

    // // Determines for which images data should be loaded.
    // if (image_names.empty()) {
    //   for (const auto& image : images) {
    //     image_ids.insert(image.ImageId());
    //   }
    // } else {
    //   for (const auto& image : images) {
    //     if (image_names.count(image.Name()) > 0) {
    //       image_ids.insert(image.ImageId());
    //     }
    //   }
    // }

    // // Collect all images that are connected in the correspondence graph.
    // std::unordered_set<image_t> connected_image_ids;
    // connected_image_ids.reserve(image_ids.size());
    // for (size_t i = 0; i < image_pair_ids.size(); ++i) {
    //   if (UseInlierMatchesCheck(two_view_geometries[i])) {
    //     image_t image_id1;
    //     image_t image_id2;
    //     Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
    //     if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
    //       connected_image_ids.insert(image_id1);
    //       connected_image_ids.insert(image_id2);
    //     }
    //   }
    // }

    // // Load images with correspondences and discard images without
    // // correspondences, as those images are useless for SfM.
    // images_.reserve(connected_image_ids.size());
    // for (const auto& image : images) {
    //   if (image_ids.count(image.ImageId()) > 0 &&
    //       connected_image_ids.count(image.ImageId()) > 0) {
    //     images_.emplace(image.ImageId(), image);
    //     const FeatureKeypoints keypoints =
    //         database.ReadKeypoints(image.ImageId());
    //     const std::vector<Eigen::Vector2d> points =
    //         FeatureKeypointsToPointsVector(keypoints);
    //     images_[image.ImageId()].SetPoints2D(points);
    //   }
    // }


    for(int i=0; i<img_size; i++)
    {
      class Image img;
      img.SetCameraId(i);
      img.SetImageId(i);
      img.SetName(std::to_string(i));
      img.SetNumCorrespondences(0); //注意这个correspondences指的是与其他所有影像的匹配数量之和，会在AddCorrespondences函数里计算
      img.SetNumObservations(point_size);//这个point_size中包含负数坐标的观测，后面需要减掉
      // img.SetQvec();
      img.SetQvecPrior(Qvecs[i]);
      // img.SetTvec();
      img.SetTvecPrior(tvecs[i]);
      const std::vector<Eigen::Vector2d> points = all_observations[i]; //第i帧的观测点，但是其中包含负数坐标点
      img.SetPoints2D(points);//同样包含false观测，需要减掉
      
      // if(i<50 || (i>50 && i<=300 && i%5==0)) //前50帧 + 50~250帧中均匀采样1/5,共100张
      // if ( i<300 && i%3==0) //0~300均匀采样100张
      // if( i<=300 && i%5==0) //子集，60张
      
      //random 100 frames
      std::ifstream rin("/home/gy/Redundancy_simulation/data_simulation/bin/random_0_300.txt", std::ios::in);
      if(!rin)
      {
        std::cout<<"can't open random file"<<std::endl;
        exit(EXIT_FAILURE);
      }
      std::set<int> imgid_list;
      while(!rin.eof())
      {
        int temp;
        rin>>temp;
        imgid_list.insert(temp);
      }

      if(imgid_list.count(i)!=0) //如果是随机影像之一
      { 
        images_.emplace(img.ImageId(), img);
        image_ids.push_back(img.ImageId());
      }
    }    
    LOG(INFO) << "loaded "<< images_.size()<< " images ";

    // std::cout << StringPrintf(" %d in %.3fs (connected %d)", images.size(),
    //                           timer.ElapsedSeconds(),
    //                           connected_image_ids.size())
    //           << std::endl;

              // LOG(INFO) << images.size()<< " in "<< timer.ElapsedSeconds()<<" s with connected "<<
              //                 connected_image_ids.size();
  }

  //////////////////////////////////////////////////////////////////////////////
  // Build correspondence graph
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Building correspondence graph..." << std::flush;


  for (const auto& image : images_) {
    correspondence_graph_.AddImage(image.first, image.second.NumPoints2D());
    // if(image.first==4846 || image.first==4839)
    // {      
    //   LOG(INFO)<<"image :"<< image.first << " num_observations: "<<image.second.NumPoints2D();
    // }
  }

  for (int i=0; i< image_ids.size(); i++)
  {
    image_t imgid1 = image_ids[i];
    // class Image img1 = images_.at(imgid1);
    // std::cout<<"img1: "<<img1.ImageId()<<std::endl;
    std::vector<Eigen::Vector2d> features1 = all_observations[imgid1];
    for (int j=i+1; j< image_ids.size(); j++)
    {
      image_t imgid2 = image_ids[j];
      // class Image img2 = images_.at(imgid2);
      // std::cout<<"img2: "<<img2.ImageId()<<std::endl;
      std::vector<Eigen::Vector2d> features2 = all_observations[imgid2];
      FeatureMatches matches;
      for( int k=0; k<point_size; k++)
      {
        Eigen::Vector2d p2d_1 = features1[k]; //第i张影像的第k个点
        Eigen::Vector2d p2d_2 = features2[k]; //第j张影像的第k个点
        if (p2d_1[0]>0 && p2d_1[0]<width && p2d_1[1]>0 && p2d_1[1]<height
          && p2d_2[0]>0 && p2d_2[0]<width && p2d_2[1]>0 && p2d_2[1]<height)
        {
          FeatureMatch match;
          match.point2D_idx1 = k;
          match.point2D_idx2 = k;
          matches.push_back(match);          
        }        
      }
      correspondence_graph_.AddCorrespondences(
              imgid1, imgid2, matches);
    }
  }
  std::cout<<"graph built";

  // // LOG(INFO)<< correspondence_graph_.ExistsImage(4839);
  // // LOG(INFO)<<"NumObservationsForImage 4839 :"<< correspondence_graph_.NumObservationsForImage(4839);
  
  // size_t num_ignored_image_pairs = 0;
  // LOG(INFO)<<"image_pair_ids.size(): "<<image_pair_ids.size();
  // size_t num_loaded_edges = 0;
  // for (size_t i = 0; i < image_pair_ids.size(); ++i) {
  //   if (UseInlierMatchesCheck(two_view_geometries[i])) {
  //     image_t image_id1;
  //     image_t image_id2;

  //     Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
  //           // LOG(INFO)<<"img1, img2:"<<image_id1<<" "<<image_id2;

  //     if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
  //       //add best edge filter
  //       ImagePair pair1 = std::make_pair(image_id1, image_id2);
  //       ImagePair pair2 = std::make_pair(image_id2, image_id1);
  //       if(best_pairs.count(pair1)!=0 || best_pairs.count(pair2)!=0)
  //       {
  //         num_loaded_edges++;
  //         //  LOG(INFO)<<"AddCorrespondences: "<<image_id1<<" "<<image_id2<<" "<<two_view_geometries[i].inlier_matches.size();
  //         correspondence_graph_.AddCorrespondences(
  //             image_id1, image_id2, two_view_geometries[i].inlier_matches);
  //       }
  //     } else {
  //       num_ignored_image_pairs += 1;
  //     }
  //   } else {
  //     num_ignored_image_pairs += 1;
  //   }
  // }
  // LOG(INFO)<<"num_loaded_edges: "<<num_loaded_edges;
  // LOG(INFO)<< correspondence_graph_.ExistsImage(4839);
  // LOG(INFO)<<"NumObservationsForImage 4839 :"<< correspondence_graph_.NumObservationsForImage(4839);
  correspondence_graph_.Finalize();
  // Set number of observations and correspondences per image.
  // LOG(INFO)<<"images_ size: "<<images_.size();
  for (auto& image : images_) {
    if(correspondence_graph_.ExistsImage(image.first))
    {
      // LOG(INFO)<<"begin";
      // LOG(INFO)<<"SetNumObservations: img id: "<<image.first;
      // LOG(INFO)<<"SetNumObservations: img id: "<<image.first<<" NumObservationsForImage: "<<correspondence_graph_.NumObservationsForImage(image.first);
      image.second.SetNumObservations(
          correspondence_graph_.NumObservationsForImage(image.first));
          // LOG(INFO)<<"SetNumCorrespondences: img id: "<<image.first<<" NumCorrespondencesForImage: "<<correspondence_graph_.NumObservationsForImage(image.first);
      image.second.SetNumCorrespondences(
          correspondence_graph_.NumCorrespondencesForImage(image.first));
          // LOG(INFO)<<"SetNumCorrespondences end";
    }
  }
  // std::cout << StringPrintf(" in %.3fs (ignored %d)", timer.ElapsedSeconds(),
  //                           num_ignored_image_pairs)
  //           << std::endl;
}


const class Image* DatabaseCache::FindImageWithName(
    const std::string& name) const {
  for (const auto& image : images_) {
    if (image.second.Name() == name) {
      return &image.second;
    }
  }
  return nullptr;
}

}  // namespace colmap
