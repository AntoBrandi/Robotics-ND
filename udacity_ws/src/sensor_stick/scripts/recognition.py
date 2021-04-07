#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import *


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    leaf_size = 0.01
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    cloud_vox = vox.filter()

    # PassThrough Filter
    passthrough = cloud_vox.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(0.76, 1.1)
    cloud_pass = passthrough.filter()

    # RANSAC Plane Segmentation
    seg = cloud_pass.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)
    

    # Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_table = cloud_pass.extract(inliers, negative=False)
    cloud_objects = cloud_pass.extract(inliers, negative=True)

    # Euclidean Clustering
    # convert XYZRGB t RGB
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    # convert to a tree structure
    tree = white_cloud.make_kdtree()
    # Perform the clustering
    cloud_clustering = white_cloud.make_EuclideanClusterExtraction()
    cloud_clustering.set_ClusterTolerance(0.05)
    cloud_clustering.set_MinClusterSize(10)
    cloud_clustering.set_MaxClusterSize(5000)
    cloud_clustering.set_SearchMethod(tree)
    cluster_indices = cloud_clustering.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color to each cluster
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    rospy.loginfo('cluster indeces size %s' % len(cluster_indices))

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    
    # Create new cloud with all the colored clusters
    cloud_cluster_colored = pcl.PointCloud_PointXYZRGB()
    cloud_cluster_colored.from_list(color_cluster_point_list)  


    # Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_cluster = pcl_to_ros(cloud_cluster_colored)

    # Publish ROS messages
    pcl_object_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cloud_cluster)


    # Classify the clusters!
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)

        # Convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # Complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud', pc2.PointCloud2, pcl_callback, queue_size = 1)

    # Create Publishers
    pcl_object_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # Load the SVM trained model from Disk
    model = pickle.load(open('/home/omncbar/Robotics-ND/udacity_ws/model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
