# SF32D
3D to 2D sensor fusion method for automotive applications

This package realizes the fusion of data from LiDAR and Yolo v7 detection. The sensor fusion process consists of six steps: time synchronization of the data-flows, preprocessing of the LiDAR cloud points, distance calculation of the cloud points, projection of the 3D cloud points to 2D Yolov7 detection images, the distance measurement of the detected object, and finally the output of the detection information.
1.	Time synchronization of the data-flows
    The dataflows from LiDAR and Yolo v7 are at different rates, so the first step is to synchronize the two data sources based on the timestamp in the data frames. The LiDAR outputs point cloud data at 10 Hz, while the Yolov7 outputs image with detection information at about 3 to 4 Hz due to the limited performance of our current computing platform. We utilize the time synchronizer provided in the “message_filters” library and adopt the ApproximateTime policy which uses an adaptive algorithm to match messages based on their timestamps. Only the data frames from the two sources generated approximately at the same time can be passed into the fusion algorithm.
2.	Preprocessing of the LiDAR cloud points
    The fusion algorithm, which is the callback function in our ROS node, starts with the preprocessing of the LiDAR point cloud. The message type is sensor_msgs :: PointCloud2, and we use the PCL library to convert it into PointCloud < pcl :: PointXYZRGB > data type. In this way, we can get a point cloud data with the coordinate information of x, y, z, and create a RGB field for each point. which can be used for visualization. Then the pcl :: PassThrough is implemented on x axis and z axis to limit the FOV and filter out the ground points. The next step is the downsampling using the pcl :: VoxelGrid function to reduces the number of points while preserving important features and structures, and using the KdTree and pcl :: EuclideanClusterExtraction to reorganize the points into clusters which represent the detected objects.
3.	Distance calculation of the cloud points
    The distance can be either defined as the 3D Euclidean distance, which can be calculated using the x, y, z coordinate, or the longitudinal distance using the distance in Y axis directly.
4.	Projection of the 3D cloud points to 2D Yolov7 detection images
    Since we already have the rotation vector and translation vector between the LiDAR and the camera, and the intrinsic matrix and distortion coefficients of the camera, we use the cv :: projectPoints function to project the 3D point onto the 2D Yolo v7 detection image.
5.	The distance measurement of the detected object
    In this step, we iterate through the detection results to extract the distance of each detected object. We can easily get the projected LiDAR points fall within the bounding boxes, but there might be some blank spaces at the corner of the bounding box, or some background points fall inside the bounding box. To mitigate this kind of outliers, we apply a rescale factor to both the height and width of the bounding box to shrink it. Then the minimum distance in the bounding box can represent the distance of the detected object. (Then we can either use the minimum distance or calculate the average value of the distance to represent the distance of the object.)
6.	The output of the detection information
    Finally, we can choose to output any necessary information according to the different requirements of the project at different development stages. In the testing stage, we need to publish point cloud after the down-sampling and clustering to tune the parameters, and visualize the projected LiDAR points on the image to check if the projection is correct. We also need to print the detection information, such as object class, confidence score, and distance information, and save that information to a local CSV file for post-processing to evaluate the performance of the algorithm.

Dependencies
This package works in ROS1 environment, and it subscribes to sensor_msgs::PointCloud2 and vision_msgs::Detection2DArray message types, and publishes pcl::PointCloud<pcl::PointXYZRGB> and sensor_msgs::Image message types.
Required libraries:
- sensor_msgs https://github.com/ros/common_msgs
- vision_msgs https://github.com/ros-perception/vision_msgs
- PCL https://github.com/ros-perception/perception_pcl
- OpenCV and cv_bridge https://github.com/ros-perception/vision_opencv
- message_filters https://github.com/ros/ros_comm

Quick start
1.	First, we need to run the roscore to start the ROS1 service, and run rviz to visualize the data that will be published by the sensors.
2.	The second step is to start the nodes of LiDAR, camera, and Yolo v7, and check their running status in rviz.
3.	Then, we can start the fusion node by running roslaunch depth_make depth_make.launch, if all the other nodes works fine. The fusion node will publish fused point cloud and image for visualization, print the information of detected objects with distance in the terminal, and save the information in the csv file for post-processing.

Configurations in the launch file
In the launch file, we can set and adjust some parameters which are used in the preprocessing step.
- ground_level: 
    This parameter indicates the height of the LiDAR from the ground, and it is used for filtering out the points on the ground. In our case, the ground level is set to -2.0m.
- leaf_size: 
    This parameter indicates the leaf size of the voxel grid, which means we divide the space into voxel grids, and use the centroid as the representative point of all the points inside the voxel. We set the leaf size to 0.1m.
- cluster_tolerance: 
    This parameter indicates the clustering distance threshold and it is set to 0.3m, which means if the distance between two points is larger than 30cm, they will not be regarded as the same object.
- MinClusterSize: 
    This parameter controls the minimum number of points required for a cluster to be considered valid. Clusters with points less than the minimum value will be discarded. We set the minimum to 20 points.
    bbox_rescale: This parameter is the re-scale factor to both the height and width of the bounding box, which allows us to focus more on the points around the center of the object. In our algorithm, we set the re-scale factor to 0.9.
