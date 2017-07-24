#ifndef MESH_TO_TRAVERSABILITY_IMAGE_H
#define MESH_TO_TRAVERSABILITY_IMAGE_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl/filters/extract_indices.h>

#include <std_srvs/Empty.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace mesh_to_image_map {

std::string fileType(std::string fichier);
void gammaCorrection(cv::Mat& src, cv::Mat& dst, float fGamma);
constexpr bool kDefaultVerbose = false;
constexpr bool kDefaultLoadFromFile = false;
constexpr bool kDefaultAutomaticPub = false;
static double kDefaultImageScale = 20.0;
static double kDefaultZThreshold = 100.0;
static const std::string kDefaultFile = "/home/eth/mesh.pcd";

class MeshToImageMapConverter {

public:
  bool load_from_file;
  MeshToImageMapConverter(ros::NodeHandle nh, ros::NodeHandle nh_private);
  // service callBack
  bool loadMeshCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); // service CallBak to load mesh from a file
  bool publishCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  // Datacallback
  void meshCallback(const pcl::PointCloud<pcl::PointXYZRGB>& mesh);

private:
  // Initial interactions with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void getParametersFromRos();

  // Node Handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers.
  ros::Subscriber mesh_sub_;
  // Publisher
  ros::Publisher pcd_pub_;
  ros::Publisher transform_pub_;
  image_transport::Publisher  img_pub;
  // services
  ros::ServiceServer load_mesh_srv_;
  ros::ServiceServer publish_srv_;
  ros::ServiceClient client;

  // Params
  bool verbose_;
  bool automatic_pub_;
  bool pub_;
  std::string file_;
  double image_scale_;
  double z_threshold_;
};


} // namespace mesh_to_image_map

#endif //
