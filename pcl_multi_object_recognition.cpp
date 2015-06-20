#include <thread>

#include "find_function.h"
#include "Visualizer.h"
#include "freenect_grabber.hpp"



int
main (int argc, char** argv)
{


  // Parse input and set algorithm variables
  ParseCommandLine (argc, argv);
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr complete_scene (new pcl::PointCloud<PointType> ());

  std::vector<float> * filters = new std::vector<float>();
  std::vector<int> * icp_iterations = new std::vector<int>();
  // Load the input model (n models but for now only one is used)
  std::vector < pcl::PointCloud < PointType > ::Ptr > model_list = ReadModels (argv, *filters, *icp_iterations);
  if (model_list.size () == 0) {
    std::cout << " no models loaded " << std::endl;
    return 1;
  }

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;

  freenectGrabber<pcl::PointXYZRGB> c;

  cloud = c.get_point_cloud(distance, true);


  cloud->sensor_orientation_.w () = 0.0;
  cloud->sensor_orientation_.x () = 1.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;
  
  int num_threads = model_list.size();
  Visualizer visualizer;
  Semaphore s(num_threads);
  std::vector<std::thread> thread_list(num_threads);
  std::vector<ClusterType> found_models(num_threads);
  ErrorWriter e;
  int frame_index = 0;;

  /// @todo support recording and playback of data
  cloud = c.get_point_cloud(distance, true);
  frame_index++;
  copyPointCloud (*cloud, *complete_scene);

  for(int i = 0; i < num_threads; ++i)
      thread_list[i] = std::thread(FindObject, model_list[i], std::ref(scene), std::ref(s), std::ref(found_models), i, filters->at(i), icp_iterations->at(i), std::ref(e), std::ref(frame_index));
    
  // Start the main detection loop
  // 1- wait for the threads to find all the objects
  // 2- visualize the scene and the found models
  // 3- read a new scene
  // 4- wake up threads
    
  while(!visualizer.viewer_.wasStopped ()){
    //wait for the threads to complete
    s.Wait4threads();
    
    //Visualizing the model, scene and the estimated model position
    SetViewPoint (complete_scene);
    visualizer.Visualize (model_list, found_models, complete_scene);
    
    copyPointCloud (*cloud, *complete_scene);
    frame_index++;
    // Wake up the threads
    s.Notify2threads();
  }

  // Notifies all the threads top stop and waits for them to join
  s.SetStop();
  for(int i = 0; i < num_threads; ++i)
      thread_list[i].join();

  return (0);
}

