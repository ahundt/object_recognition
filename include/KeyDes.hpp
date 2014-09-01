#ifndef KEYDES_HPP
#define KEYDES_HPP

#include "define.h"
#include <pcl/features/narf_descriptor.h>
#include <pcl/correspondence.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/ppfrgb.h>
#include <pcl/features/ppf.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/pfh.h>
#include <pcl/features/impl/ppfrgb.hpp>
#include <pcl/features/shot_omp.h>


template<class T, class Estimator>
class KeyDes
{
  public:
    typedef pcl::PointCloud<T> PD;
    typedef pcl::PointCloud<PointType> P;
    typedef pcl::PointCloud<NormalType> PN;
    typename PD::Ptr model_descriptors_;
    typename PD::Ptr scene_descriptors_;
    typename P::Ptr model_;
    typename P::Ptr model_keypoints_;
    typename P::Ptr scene_;
    typename P::Ptr scene_keypoints_;
    typename PN::Ptr model_normals_;
    typename PN::Ptr scene_normals_;
    bool created;

    KeyDes (P::Ptr model, P::Ptr model_keypoints, P::Ptr scene, P::Ptr scene_keypoints, PN::Ptr model_normals, PN::Ptr scene_normals) :
        model_descriptors_ (new PD ()), scene_descriptors_ (new PD ()), model_ (model), model_keypoints_ (model_keypoints), scene_ (scene), scene_keypoints_ (scene_keypoints), model_normals_ (model_normals), scene_normals_ (scene_normals), created (false)
    {
    }

    pcl::CorrespondencesPtr
    Run ()
    {

      pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

      //create scene descriptors
      std::cout << "calculating scene descriptors " << std::endl;
      Estimator est;
      est.setInputCloud (scene_keypoints_);
      est.setSearchSurface (scene_);
      est.setInputNormals (scene_normals_);

      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      est.setSearchMethod (tree);
      est.setRadiusSearch (descr_rad);
      est.compute (*scene_descriptors_);

      if (!created)
      {
        //create model descriptors
        std::cout << "calculating model descriptors " << std::endl;
        est.setInputCloud (model_keypoints_);
        est.setSearchSurface (model_);
        est.setInputNormals (model_normals_);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);
        est.setSearchMethod (tree2);
        est.compute (*model_descriptors_);
        created = true;
      }

      pcl::KdTreeFLANN<T> match_search;

      //  Find Model-Scene Correspondences with KdTree
      std::cout << "calculating correspondences " << std::endl;

      match_search.setInputCloud (model_descriptors_);

      //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
      #pragma omp parallel for 
      for (size_t i = 0; i < scene_descriptors_->size (); ++i)
      {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (match_search.point_representation_->isValid (scene_descriptors_->at (i)))
        {
          int found_neighs = match_search.nearestKSearch (scene_descriptors_->at (i), 1, neigh_indices, neigh_sqr_dists);
          if (found_neighs == 1 && neigh_sqr_dists[0] < descriptor_distance)
          {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            #pragma omp critical
            model_scene_corrs->push_back (corr);
          }
        }
      }

      std::cout << "\tFound " << model_scene_corrs->size () << " correspondences " << std::endl;
      return (model_scene_corrs);

    }
};

#endif