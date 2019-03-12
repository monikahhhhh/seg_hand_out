#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "image_transport/image_transport.h"
#include "opencv/cv.h"
#include "sensor_msgs/image_encodings.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
//#include "seg_hand/seg_hand_out.h"
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
#include "message_filters/sync_policies/approximate_time.h"
#include <seg_hand/pcl_segmentation.h>
#include <seg_hand/subscribeandpublish.h>
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace cv;




/*
typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> MySyncPolicy;

class SubscribeAndPublish
{
private:
    ros::NodeHandle  nh;
    ros::Publisher  object_point_cloud_publisher;
    message_filters::Subscriber<PointCloud2>* point_cloud_subscriber;
    message_filters::Subscriber<PointCloud2>* hand_point_cloud_subscriber;
   
    message_filters::Synchronizer<MySyncPolicy>* syn;
public:
    SubscribeAndPublish()
    {
        //object_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("object_pcd_topic", 10);
        object_point_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("object_pcd_topic",2);
        point_cloud_subscriber = new message_filters::Subscriber<PointCloud2>(nh,"/camera/depth_registered/points",1);
        hand_point_cloud_subscriber = new message_filters::Subscriber<PointCloud2> (nh,"/hand_pcd",1);

        syn = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*point_cloud_subscriber, *hand_point_cloud_subscriber);
        syn->registerCallback(boost::bind(&SubscribeAndPublish::callback,this, _1,_2));
    
    }
    
    void callback(const PointCloud2ConstPtr& point_cloud_msg, const PointCloud2ConstPtr& hand_point_cloud_msg)
    {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr hand_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::PointCloud<pcl::PointXYZ> object_point_cloud;
        PCL_SEGMENTATION seg_hand;
        try
        {
            pcl::fromROSMsg(*point_cloud_msg, *scene_point_cloud);
            pcl::fromROSMsg(*hand_point_cloud_msg, *hand_point_cloud);
    
            //*object_point_cloud = seg_hand_out(scene_point_cloud, hand_point_cloud);
            *object_point_cloud = seg_hand._seg_hand_out(scene_point_cloud, hand_point_cloud);
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>("pass.pcd", *object_point_cloud);
        
          
           pcl_conversions::toPCL(ros::Time::now(), object_point_cloud->header.stamp);
           object_point_cloud -> header.frame_id = hand_point_cloud_msg->header.frame_id;
           object_point_cloud_publisher.publish(object_point_cloud);
           
        }

        
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    
    } 
};
/*
//+++++++++++++++++++++++++++++++++++++++++original++++++++++++++++++++++++++++++++++++++++++++++++++

    pcl::PointCloud<pcl::PointXYZ> seg_hand_out(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > scene_point_cloud, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > hand_point_cloud) 
    {
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg(new pcl::PointCloud<pcl::PointXYZ>), cloud_dep (new pcl::PointCloud<pcl::PointXYZ>), 
        cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);

        //preprocess on input,  ctreate filtering object: downsample the dataset using leaf siye of 1 cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(scene_point_cloud);
        vg.setLeafSize(0.01f,0.01f,0.01f);
        vg.filter(*cloud_vg);


        // MAYBE should remove outlier on cloud_h
        //get the depth range of the hand  
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*hand_point_cloud, minPt, maxPt);

        //segmentation based on depth(z-axis)
        //create the filter
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_vg);
        pass.setFilterFieldName("z");  
        //pass.setFilterLimits(minPt.z-0.05, maxPt.z+0.05);
        //identify  the closest set of points (z ranges widely)
        pass.setFilterLimits(minPt.z-0.3, minPt.z+0.3);
        pass.setFilterLimitsNegative(false);
        pass.filter(*cloud_dep);
        
        
        pcl::PCDWriter writer;
        //writer.write<pcl::PointXYZ>("test.pcd", *cloud_dep, false);
        

        //remove the point cloud belongs to other objects according to the hand pose(x_axis)
        // use conditional segmentation
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ> ::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps:: GT, -0.1)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ> ::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps:: LT, 0.1)));
        //can also add condition along y-axis
        
        pcl::ConditionalRemoval<pcl::PointXYZ> condrm;
        condrm.setInputCloud(cloud_dep);
        condrm.setCondition(range_cond);
        condrm.setKeepOrganized(false);
        condrm.filter(*cloud_seg);
        //writer.write<pcl::PointXYZ>("test1.pcd", *cloud_seg, false);

        //cluster the remaining point cloud, select the cluster which contains object
        //creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>:: Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_seg);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setInputCloud(cloud_seg);
        ec.setClusterTolerance(0.02f);
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(50000);
        ec.setSearchMethod(tree);
        ec.extract(cluster_indices);
    
        pcl::CentroidPoint<pcl::PointXYZ> h_centroid;
        for (size_t i = 0; i < hand_point_cloud -> points.size(); ++i )
        {
            h_centroid.add (hand_point_cloud -> points[i]);
        }
        pcl::PointXYZ c_h;
        h_centroid.get(c_h);
        
        
        pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_h_o(new pcl::PointCloud<pcl::PointXYZ>);
        float min_dis = 0.0;
        int cluster_num = 0;
        int j=0;
        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>:: const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                    cloud_cluster->points.push_back(cloud_seg->points[*pit]);
            }

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            //calculate the centroid for each cluster
            pcl::CentroidPoint<pcl::PointXYZ> c_centroid;
            for (size_t i = 0; i < cloud_cluster -> points.size(); ++i)
            {
                c_centroid.add(cloud_cluster -> points[i]);
            }
            pcl::PointXYZ c_c;
            c_centroid.get(c_c);

            if(min_dis >= pcl::euclideanDistance(c_h,c_c))
            {
                min_dis = pcl::euclideanDistance(c_h,c_c);
                cluster_num = j;
            }
           
            std::stringstream ss;
            ss << "cloud_cluster"<< j <<".pcd";
            writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false); 
            
            
            j++;
            
        }
    


        
        std::stringstream ss;
        ss << "cloud_cluster" << cluster_num << ".pcd";
        reader.read(ss.str(),*cloud_h_o);
       
        //writer.write<pcl::PointXYZ>("object.pcd", *cloud_h_o, false);
        return *cloud_h_o;
    

*/

    /*
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_o (new pcl::PointCloud<pcl::PointXYZ>);
        pass.setInputCloud(cloud_h_o);
        pass.setFilterFieldName("x");
        pass.setFilterFieldName("y");
        pass.setFilterLimits(minPt.x, maxPt.x);
        pass.setFilterLimits(minPt.y, maxPt.y);
        pass.setFilterLimitsNegative(false);
        pass.filter(*cloud_o);
        return *cloud_o;
        //writer.write<pcl::PointXYZ>("object.pcd", *cloud_o, false);
    */
 //   }

//};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_segmentor_node");
    
    SubscribeAndPublish Pub_Object_pcd;
    ros::spin();
    return 0;
    
}


