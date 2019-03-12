#ifndef PCL_SEGMENTATION_H
#define PCL_SEGMENTATION_H

#include "string.h"
#include <iostream>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/common/distances.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl_ros/point_cloud.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>

class PCL_SEGMENTATION
{
public:
    pcl::PointCloud<pcl::PointXYZ> _voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr input);
    pcl::PointCloud<pcl::PointXYZ> _depth_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr input,std::string filter_field_name, float min_limit, float max_limit);
    pcl::PointCloud<pcl::PointXYZ> _comparison_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, std::string comparison_field, float upper_limit, float lower_limit);
    std::vector<pcl::PointIndices> _ktree_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr input, float cluster_tolerance);
    pcl::PointXYZ _cluster_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr input);
    pcl::PointCloud<pcl::PointXYZ> _cluster_selection(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ref_input, std::vector<pcl::PointIndices> cluster_indices);
    pcl::PointCloud<pcl::PointXYZ> _box_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ref_input);
    pcl::PointCloud<pcl::PointXYZ> _seg_hand_out(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr hand_point_cloud);


private:
/* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dep;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_com;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clus;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obj;
*/
    pcl::PointXYZ minPT, maxPT;

    void _get_minmax3D(pcl::PointCloud<pcl::PointXYZ> input);
};

#endif