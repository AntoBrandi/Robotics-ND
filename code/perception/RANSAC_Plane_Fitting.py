import sys
import pcl


if __name__ == '__main__':

    if len(sys.argv)>1:
        max_distance = float(sys.argv[1])
    else:
        # Default 
        max_distance = 0.01

        
    ### Load Point Cloud file
    cloud = pcl.load_XYZRGB('tabletop.pcd')



    ### Voxel Grid filter
    # Create a VoxelGrid filter object
    vox = cloud.make_voxel_grid_filter()
    # Set the voxel grid size (leaf)
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Apply the filter to the loaded point cloud
    cloud_filtered = vox.filter()



    ### Pass Through Filter
    # Create a PassThrough Filter object
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign the axis ranges for the filter
    filter_axis = 'z'
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)

    # Apply the passthrough filter
    cloud_filtered = passthrough.filter()



    ### RANSAC Plane Fitting
    # Create a segmentation object
    seg = cloud_filtered.make_segmenter()

    # Select the model that will be fitted on the data
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Set the max distance for a point to be considered inlier
    seg.set_distance_threshold(max_distance)

    # Apply the segmentation
    inliers, coefficients = seg.segment()
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)


    # Save pcd for table
    filename = 'ransac_inlier_filter_{}.pcd'.format(max_distance)
    pcl.save(extracted_inliers, filename)
    filename = 'ransac_outlier_filter_{}.pcd'.format(max_distance)
    pcl.save(extracted_outliers, filename)



