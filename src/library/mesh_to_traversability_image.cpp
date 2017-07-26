/*--
 *      This node compute the traversability from the pointCloud map from voxblox or file
 *      Using only opencv tools
 *
 *      Subscrib :
 *          /voxblox_node/mesh_pointcloud point cloud from voxblox reconstruction
 *
 *      Publish :
 *          /image_traversability the binary image of Traversability
 *          /transform an array of float containing the info to go back into world frame
 *          /point_cloud_map the map as a point cloud, to isualize on rviz if load from file
 *
 *      Service :
 *          /load_mesh_srv: to trigger the meshCallBack without msg (used when load from file)
 *          /publish_srv: to ask for the traversability if the auto_pub param is not true
 *
 *      Parameters :
 *          verbose: to show steps, time consuming, image processed
 *          automatic_pub: if false you have to call the service to publish info
 *          image_scale: resolution of the image (pixels/m)
 *          load_from_file: if you want to load mesh from file instead of voxblox
 *          z_threshold: The value of the threshold on traverasbility altitude
 *          file: The path to the file you want to use
 *
 *      Approach :
 *          1) convert mesh into depth image
 *          2) compute normals using opencv tools : gradiant + orientation
 *          3) compute traversability by thresholding point cloud on slope and altitude
 *          4) filtering traversability images
 */
#include <mesh_to_traversability_image/mesh_to_traversability_image.hpp>

namespace mesh_to_traversability {

MeshToImageMapConverter::MeshToImageMapConverter(ros::NodeHandle nh,
                                                 ros::NodeHandle nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(kDefaultVerbose),
      load_from_file(kDefaultLoadFromFile),
      automatic_pub_(kDefaultAutomaticPub),
      image_scale_(kDefaultImageScale),
      z_threshold_(kDefaultZThreshold),
      file_(kDefaultFile)
{
    // Initial interaction with ROS
    getParametersFromRos();
    subscribeToTopics();
    advertiseTopics();

    // Initialise service
    load_mesh_srv_= nh_private_.advertiseService("load_mesh_srv", &MeshToImageMapConverter::loadMeshCallback, this);
    publish_srv_  = nh_private_.advertiseService("publish_srv", &MeshToImageMapConverter::publishCallback, this);
}

void MeshToImageMapConverter::subscribeToTopics() {
    mesh_sub_ = nh_.subscribe("mesh", 10, &MeshToImageMapConverter::meshCallback, this);
}

void MeshToImageMapConverter::advertiseTopics() {
    if(load_from_file ) pcd_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud_map", 1, true);
    transform_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/transform", 1, true);

    image_transport::ImageTransport it(nh_);
    img_pub = it.advertise("/image_traversability", 1);
}

void MeshToImageMapConverter::getParametersFromRos() {
    nh_private_.param("verbose", verbose_, verbose_);
    nh_private_.param("load_from_file", load_from_file, load_from_file);
    nh_private_.param("automatic_pub", automatic_pub_, automatic_pub_);
    nh_private_.param("image_scale", image_scale_, image_scale_);
    nh_private_.param("z_threshold", z_threshold_, z_threshold_);
    nh_private_.param("file", file_, file_);
}

// service call back to load a mesh from a file (.pcd or .ply)
bool MeshToImageMapConverter::loadMeshCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    const pcl::PointCloud<pcl::PointXYZRGB> fake_mesh_msg;
    meshCallback(fake_mesh_msg);
    return true;
}

// service call back to compute traversability in the next mesh (when automatic_pub_ = false)
bool MeshToImageMapConverter::publishCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    pub_ = true;
}

// meshCallback main function where the traversability is computed
void MeshToImageMapConverter::meshCallback(const pcl::PointCloud<pcl::PointXYZRGB>& mesh_msg)
{
    if(!automatic_pub_ && !pub_) return; // no need to compute the traversability if you don't send it

    if (verbose_) ROS_INFO("Mesh received, starting treatment.");
    ros::Time time0, time1, time2;
    double duration;
    time0 = ros::Time::now();

    /// Converting from message to an object
    time1 = ros::Time::now();
    pcl::PolygonMesh polygon_mesh;
    //pcl_conversions::toPCL(mesh_msg, polygon_mesh);
    if(load_from_file)ROS_INFO_STREAM("From file :"<<file_<<" - file type : "<<fileType(file_));
    if(load_from_file && fileType(file_) == "ply") pcl::io::loadPolygonFilePLY (file_, polygon_mesh);

    /// Load input mesh into a PointCloud<T> with an appropriate type
    if (verbose_) ROS_INFO("Convertion in pointCloud");
    pcl::PCLPointCloud2 cloud_blob;
    if(load_from_file && fileType(file_) == "pcd") pcl::io::loadPCDFile (file_, cloud_blob);
    else cloud_blob = polygon_mesh.cloud;
    // convert ros::pointcloud2 to pcl::pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(load_from_file) pcl::fromPCLPointCloud2 (cloud_blob, *cloud); // the data is available in cloud
    else pcl::copyPointCloud(mesh_msg,*cloud);
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec");

    /// clean outliers (z<-0.6) (points might be under the ground at the landing points)
    pcl::PointIndices::Ptr indices_in (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> eifilter (false);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      pcl::PointXYZ point = cloud->points[i];
      if(point.data[2]>-0.6)
          indices_in->indices.push_back(i);
    }
    eifilter.setIndices(indices_in);
    eifilter.filterDirectly(cloud);

//--------------------- PointCloud treatment ---------------------------------------------------//

    /// Make mesh into depth image
    if (verbose_) ROS_INFO("Compution of mesh image");
    time1 = ros::Time::now();
    // find limits of pcd
    double xmin = 1000;
    double xmax =-1000;
    double ymin = 1000;
    double ymax =-1000;
    double zmin = 1000;
    double zmax =-1000;
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZ point = cloud->points[i];
        if( point.data[0] < xmin) xmin = point.data[0];
        if( point.data[0] > xmax) xmax = point.data[0];
        if( point.data[1] < ymin) ymin = point.data[1];
        if( point.data[1] > ymax) ymax = point.data[1];
        if( point.data[2] < zmin) zmin = point.data[2];
        if( point.data[2] > zmax) zmax = point.data[2];
    }
    int width = image_scale_*(xmax-xmin)+1;
    int height = image_scale_*(ymax-ymin)+1;
    zscale_ = 255/(zmax-zmin); // full scale grey pixels/m
    if (verbose_) ROS_INFO_STREAM("zone : "<<xmin<<"-"<<xmax<<" , "<<ymin<<"-"<<ymax<<" , "<<zmin<<"-"<<zmax<<" -> scale="<<image_scale_<<" ("<<width<<","<<height<<") zscale="<<zscale_);
    if(width < 0 || height < 0) return;
    cv::Mat mesh_img = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat mesh_in = cv::Mat::zeros(height, width, CV_8UC1);
    // projection of the height in the image
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZ point = cloud->points[i];

        unsigned int y = (unsigned int)(image_scale_*(point.data[1]-ymin));
        unsigned int x = (width-1)-(unsigned int)(image_scale_*(point.data[0]-xmin));
        if(y<0 || y>height-1) ROS_ERROR_STREAM("erreur en y : "<<y);
        if(x<0 || x>width-1) ROS_ERROR_STREAM("erreur en x : "<<x);
        //mesh_img.at<unsigned char>(y,x) = (unsigned char)(zscale*(cloud->points[i].data[2]-zmin));
        cv::circle(mesh_img,cv::Point(x,y),2,(unsigned char)(zscale_*(cloud->points[i].data[2]-zmin)),-1); // draw circle because voxblox voxel size is 0.1 m
        mesh_in.at<unsigned char>(y,x) = 255;
    }
    // image processing to improve the results
    //gammaCorrection(mesh_img, mesh_img, 2.0);
    cv::GaussianBlur(mesh_img,mesh_img,cv::Size(5,5),0,0);
    if(verbose_) cv::imshow("mesh_img",mesh_img);
    // fill the holes of the map
    int element_size0 = 1;
    cv::Mat element0 = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*element_size0 + 1, 2*element_size0+1 ), cv::Point( element_size0, element_size0 ) );
    cv::morphologyEx( mesh_in, mesh_in, cv::MORPH_DILATE, element0 );
    cv::medianBlur(mesh_in,mesh_in,3);
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if(verbose_) ROS_INFO_STREAM(duration<<"sec");

    /// using Opencv to compute slope
    if (verbose_) ROS_INFO("Compution of normals");
    time1 = ros::Time::now();
    cv::Mat modifiedImage = mesh_img.clone();
    // gradiant
    int ddepth = CV_32F;
    cv::Mat grad_x, grad_y;
    cv::Scharr( modifiedImage, grad_x, ddepth, 1, 0, 1, 0, cv::BORDER_DEFAULT );
    cv::Scharr( modifiedImage, grad_y, ddepth, 0, 1, 1, 0, cv::BORDER_DEFAULT );
    cv::Mat gradiant = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_32F);
    cv::magnitude(grad_x,grad_y,gradiant);
    //cv::normalize(gradiant, gradiant,0.0, 255.0, cv::NORM_MINMAX, CV_32F);
    //cv::normalize(gradiant, gradiant,0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
    // smouthing the gradiant
    cv::GaussianBlur(gradiant,gradiant,cv::Size(3,3),0,0);

    // orientation
    cv::Mat orientation = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_32F);
    cv::phase(grad_x, grad_y, orientation,true);
    cv::normalize(orientation, orientation, 0x00, 0xFF, cv::NORM_MINMAX, CV_8U);

    // Compute the normals and store the z component in another image
    cv::Mat normal_z = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_32F);
    double coef = image_scale_  * zscale_ / 16; // 16 for half the sum of scharr pattern
    if (verbose_) ROS_INFO_STREAM("Coef = "<<coef);
    for (int i=0; i < modifiedImage.rows;i++)
    {
        for (int j=0; j < modifiedImage.cols;j++)
        {
            if(modifiedImage.at<uchar>(i,j) != 0)
            {
                double angle = std::atan(gradiant.at<float>(i,j)/coef);
                Eigen::Vector3d normal(sin(angle) * sin(orientation.at<uchar>(i,j)*2*M_PI/255.0) , sin(angle) * cos(orientation.at<uchar>(i,j)*2*M_PI/255.0) , cos(angle));
                normal.normalize();
                normal_z.at<float>(i,j) = (float)normal.z();
            }
        }
    }
    //cv::normalize(normal_z, normal_z, 0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if(verbose_) ROS_INFO_STREAM(duration<<"sec");

    /// Computation of image traversability
    if (verbose_) ROS_INFO("Compution of traversability");
    time1 = ros::Time::now();
    cv::Mat traversability_img = cv::Mat::zeros(modifiedImage.rows, modifiedImage.cols, CV_8U);
    // thresholding on normal_z to get traversability
    for (int i=0; i < modifiedImage.rows;i++)
    {
        for (int j=0; j < modifiedImage.cols;j++)
        {
            if(mesh_img.at<uchar>(i,j) < z_threshold_*zscale_ && mesh_in.at<uchar>(i,j)>0  && normal_z.at<float>(i,j) > 0.8) //
                traversability_img.at<uchar>(i,j) = 255;
        }
    }
    //cv::imshow("traversability_image_raw",traversability_img);
    // median filter to remove small black pixels
    cv::medianBlur(traversability_img,traversability_img,5);
    // erode to make bigger the places to evode
    int element_size = 1;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*element_size + 1, 2*element_size+1 ), cv::Point( element_size, element_size ) );
    cv::morphologyEx( traversability_img, traversability_img, cv::MORPH_ERODE, element );
    time2 = ros::Time::now();
    duration = time2.toSec() - time1.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec");
    //cv::imshow("traversability",traversability_img);

    // colorfull drawing
    cv::normalize(gradiant, gradiant,0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
    int amplify_magnitude = 20; // for a better visualisation
    for(unsigned int i=0;i<grad_x.rows;i++)
    {
        for(unsigned int j=0;j<grad_y.cols;j++)
        {
            if(gradiant.at<uchar>(i,j)*amplify_magnitude < 255)
                gradiant.at<uchar>(i,j) = gradiant.at<uchar>(i,j)*amplify_magnitude;
            else
                gradiant.at<uchar>(i,j) = 255;
        }
    }
    cv::Mat fusion = cv::Mat::zeros(grad_x.rows, grad_y.cols, CV_8UC3);
    cv::cvtColor(fusion,fusion,cv::COLOR_BGR2HSV);
    for(unsigned int i=0;i<grad_x.rows;i++)
    {
        for(unsigned int j=0;j<grad_y.cols;j++)
        {
            fusion.at<cv::Vec3b>(i,j) = cv::Vec3b(orientation.at<unsigned char>(i,j),200,(gradiant.at<unsigned char>(i,j)));
        }
    }
    cv::cvtColor(fusion,fusion,cv::COLOR_HSV2BGR);
    if(verbose_) cv::imshow("fusion", fusion);
    //cv::imshow("gradiant",gradiant);
    //cv::imshow("orientation",orientation);
    cv::normalize(normal_z, normal_z, 0x00, 0xFF, cv::NORM_MINMAX, CV_8U);
    cv::imshow("normal_z",normal_z);

//--------------------- Publishing ---------------------------------------------------//
    if (verbose_) ROS_INFO("Publishing ");
    time1 = ros::Time::now();
    /// Publishing PointCloud original map
    if(load_from_file)
    {
        cloud->header.frame_id="map";
        pcd_pub_.publish(*cloud);
    }

    /// publishig info for metric transformation
    std_msgs::Float32MultiArray msgTransform;
    msgTransform.data.push_back(xmin);
    msgTransform.data.push_back(1.0 / image_scale_); // we need m/pixel
    msgTransform.data.push_back(ymin);
    msgTransform.data.push_back(1.0 / image_scale_);
    msgTransform.data.push_back(zmin);
    msgTransform.data.push_back(1.0 / zscale_);
    transform_pub_.publish(msgTransform);

    /// publishing image_traversability
    sensor_msgs::ImagePtr msgPublish;
    msgPublish = cv_bridge::CvImage(std_msgs::Header(), "mono8", traversability_img).toImageMsg();
    img_pub.publish (msgPublish);

    if (verbose_) ROS_INFO("Treatment of the mesh ok!");
    pub_ = false;
    time2 = ros::Time::now();
    duration = time2.toSec() - time0.toSec();
    if (verbose_) ROS_INFO_STREAM(duration<<"sec\n");
    if (verbose_) cv::waitKey(10);

}


//-------------- Methodes annexes ---------------------------//
std::string fileType(std::string fichier)
{
    // on découpe par rapport au "/" et on prend le dernier morceau
    std::vector<std::string> tab;
    boost::split(tab, fichier, boost::is_any_of("/"));
    std::string nameExtension = tab[tab.size()-1];
    // on redécoupe le dernier morceau pour lit l'extention, on prend le dernier morceau
    tab.clear();
    boost::split(tab, nameExtension, boost::is_any_of("."));
    std::string type = tab[1];

    return type;
}

void gammaCorrection(cv::Mat& src, cv::Mat& dst, float fGamma)
{
    unsigned char lut[256];
    for (int i = 0; i < 256; i++)
    {
        lut[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
    }
    dst = src.clone();
    cv::MatIterator_<uchar> it, end;
    for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)
        *it = lut[(*it)];

    return;
}

}  // namespace mesh_to_image_map
