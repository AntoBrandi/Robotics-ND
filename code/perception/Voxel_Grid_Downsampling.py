import sys
import pcl


if __name__ == '__main__':

    if len(sys.argv)>1:
        LEAF_SIZE = float(sys.argv[1])
    else:
        # Default 
        LEAF_SIZE = 0.01

    ### Load Point Cloud file
    cloud = pcl.load_XYZRGB('tabletop.pcd')


    ### Voxel Grid filter
    # Create a VoxelGrid filter object
    vox = cloud.make_voxel_grid_filter()
    # Set the voxel grid size (leaf)
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Apply the filter to the loaded point cloud
    cloud_filtered = vox.filter()
    filename = 'voxel_downsampled_{}m.pcd'.format(LEAF_SIZE)


    # Save pcd for table
    pcl.save(cloud_filtered, filename)



