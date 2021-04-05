import sys
import pcl


if __name__ == '__main__':

    if len(sys.argv)>2:
        axis_min = float(sys.argv[1])
        axis_max = float(sys.argv[2])
    else:
        # Default 
        axis_min = 0.6
        axis_max = 1.1

        
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
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)

    # Apply the passthrough filter
    cloud_filtered = passthrough.filter()


    # Save pcd for table
    filename = 'pass_through_filtered_{}_{}.pcd'.format(axis_min, axis_max)
    pcl.save(cloud_filtered, filename)



