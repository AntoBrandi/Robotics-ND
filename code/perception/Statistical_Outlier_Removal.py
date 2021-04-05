import sys
import pcl


if __name__ == '__main__':

    ### Load Point Cloud file
    cloud = pcl.load('tabletop_raw.pcd')


    ### Statistical Outlier Removal
    # Create a VoxelGrid filter object
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points
    outlier_filter.set_mean_k(50)
    
    # Set the threshold scale factor
    x = 1.0

    # Any point with a mean distance longer than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Apply the filter
    cloud_filtered = outlier_filter.filter()
    

    # Save pcd for table
    filename = 'tabletop_outlier_removed.pcd'
    pcl.save(cloud_filtered, filename)

