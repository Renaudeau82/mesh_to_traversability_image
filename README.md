# mesh_to_traversability_image #
ROS package containing 1 node :

*  **mesh_to_traversability_image_node :**  Compute the traversability as an image using reconstruction from voxblox

## Dependences ##

* Opencv, pcl, voxblox

## Usage ##

* **roslaunch mesh_to_traversability_image.launch :** use voxblox_node tu create 3D mesh then subscrib to /mesh_pointcloud

Launch rosbag (you have to use yours) 

dense_stereo (you have to use your calibration file .yaml)

voxblox_node (to build the 3D reconstruction)

mesh_to_traversability_image_node (this pkg node)

* **roslaunch pcd_to_traversability_image.launch :** 

open a .pcd or .ply file to compute traversability using mesh_to_traversability_image_node

## Parameters ##

* **load_from_file :** true to load from file, false to connect to voxblox mesh

* **file :** the file to open

* **image_scale :** resolution of the image (pixels/m)

* **z_threshold :** The value to threshold the altitude of the traversability

* **automatic_pub :** if false, you need to use service call (~/publish_srv) for the publishment

