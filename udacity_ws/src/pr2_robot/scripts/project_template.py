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

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    ### Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    ### Statistical Outlier Filtering
    # sof = cloud.make_statistical_outlier_filter()
    # sof.set_mean_k(50)                              # number of neighboring points
    # sof.set_std_dev_mul_thresh(0.6)                 # threshold scale factor
    # cloud = sof.filter()

    ### Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    leaf_size = 0.01                                # dimension of each voxel in m
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    cloud = vox.filter()

    ### PassThrough Filter
    # Z-Axis Filter
    passthrough_z = cloud.make_passthrough_filter()
    passthrough_z.set_filter_field_name('z')          # direction of the filter
    passthrough_z.set_filter_limits(0.6, 1.4)        # min and max value of the filter
    cloud = passthrough_z.filter()
    # Y-Axis Filter
    passthrough_y = cloud.make_passthrough_filter()
    passthrough_y.set_filter_field_name('y')          # direction of the filter
    passthrough_y.set_filter_limits(-0.5, 0.5)        # min and max value of the filter
    cloud = passthrough_y.filter()

    ### RANSAC Plane Segmentation
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)

    ### Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_table = cloud.extract(inliers, negative=False)
    cloud_objects = cloud.extract(inliers, negative=True)

    ### Statistical Outlier Filtering - only on the extracted objects
    sof = cloud_objects.make_statistical_outlier_filter()
    sof.set_mean_k(50)                              # number of neighboring points
    sof.set_std_dev_mul_thresh(0.6)                 # threshold scale factor
    cloud_objects = sof.filter()

    ### Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)      # use only the XYZ information for clustering
    tree = white_cloud.make_kdtree()                # convert to a tree structure
    cloud_clustering = white_cloud.make_EuclideanClusterExtraction()
    cloud_clustering.set_ClusterTolerance(0.013)
    cloud_clustering.set_MinClusterSize(10)
    cloud_clustering.set_MaxClusterSize(2000)
    cloud_clustering.set_SearchMethod(tree)
    cluster_indices = cloud_clustering.Extract()

    ### Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    rospy.loginfo('number of clusters %s' % len(cluster_indices))

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    
    # Create new cloud with all the colored clusters
    cloud_cluster_colored = pcl.PointCloud_PointXYZRGB()
    cloud_cluster_colored.from_list(color_cluster_point_list) 

    ### Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_cluster = pcl_to_ros(cloud_cluster_colored)

    ### Publish ROS messages
    pcl_object_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cloud_cluster)

    ### Classify the clusters! (loop through each detected cluster one at a time)
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

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    WORLD_NUMBER = 1
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    dict_list = []

    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    test_scene_num.data = WORLD_NUMBER

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # Parse parameters into individual variables
    dropbox_left = dropbox_param[0]['position']
    dropbox_right = dropbox_param[1]['position']

    # Rotate PR2 in place to capture side tables for the collision map
    pr2_angle_pub.publish(-1.57)
    rospy.sleep(10)
    pr2_angle_pub.publish(1.57)

    # Get the PointCloud for a given object and obtain it's centroid
    for obj in object_list:
        labels.append(obj.label)
        points_arr = ros_to_pcl(obj.cloud).to_array()
        centroid = np.mean(points_arr, axis=0)[:3]
        centroids.append(centroid)

    # Loop through the pick list
    for i, obj in enumerate(object_list_param):
        object_name.data = object_list_param[i]['name']

        # Assign the arm to be used for pick_place
        if object_list_param[i]['group']   == 'red':
            arm_name.data = 'left'
        else:
            arm_name.data = 'right'

        try:
            detected_idx = labels.index(object_name)

            # Create 'pick_pose' for the object
            pick_pose.position.x = np.asscalar(centroids[detected_idx][0])
            pick_pose.position.y = np.asscalar(centroids[detected_idx][1])
            pick_pose.position.z = np.asscalar(centroids[detected_idx][2])

            # Create 'place_pose' for the object
            if object_group == 'red':
                place_pose.position.x = dropbox_left[0]
                place_pose.position.y = dropbox_left[1]
                place_pose.position.z = dropbox_left[2]
            else:
                place_pose.position.x = dropbox_right[0]
                place_pose.position.y = dropbox_right[1]
                place_pose.position.z = dropbox_right[2]

        except:
            pass
        
        finally:

            # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
            dict_list.append(yaml_dict)
      

    # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')

    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        # Insert your message variables to be sent as a service request
        resp = pick_place_routine(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        print ("Response: ", resp.success)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    send_to_yaml('/home/omncbar/Robotics-ND/udacity_ws/output_{}.yaml'.format(WORLD_NUMBER), dict_list)



if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('pr2_perception', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size = 1)

    # Create Publishers
    pcl_object_pub = rospy.Publisher('/pr2/world/objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pr2/world/table', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pr2/world/cluster', PointCloud2, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=10)
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=10)
    pr2_angle_pub = rospy.Publisher('/pr2/world_joint_controller/command', Float64, queue_size=10)

    # Load Model From disk
    model = pickle.load(open('/home/omncbar/Robotics-ND/udacity_ws/model_3.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
