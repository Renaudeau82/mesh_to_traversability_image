#include <ros/ros.h>
#include <mesh_to_traversability_image/mesh_to_traversability_image.hpp>

// Standard C++ entry point
int main(int argc, char **argv) {

  // Announce this program to the ROS master
  ros::init(argc, argv, "mesh_to_traversability_image_node");
  ros::NodeHandle nh; 
  ros::NodeHandle private_nh("~");

  // Creating the object to do the work.
  mesh_to_traversability::MeshToImageMapConverter mesh_to_image_map_converter(nh, private_nh);

  ros::spinOnce();
  // auto-call of service if "load_from_file"
  if(mesh_to_image_map_converter.load_from_file)
  {
      const pcl::PointCloud<pcl::PointXYZRGB> fake_mesh_msg;
      mesh_to_image_map_converter.meshCallback(fake_mesh_msg);
  }

  ros::spin();
  return 0;
}

