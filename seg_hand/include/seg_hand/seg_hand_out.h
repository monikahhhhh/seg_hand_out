#ifndef SEG_HAND_OUT_H_
#define SEG_HAND_OUT_H_

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

//int seg_hand_out(const pcl::PointCloud<pcl::PointXYZ>& scene_point_cloud, const pcl::PointCloud<pcl::PointXYZ>& hand_point_cloud);
namespace Seg_Hand_Out
{
    int seg_hand_out(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > scene_point_cloud, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > hand_point_cloud) ;
}

#endif