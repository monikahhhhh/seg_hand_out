#include "seg_hand/pcl_segmentation.h"
#include "seg_hand/subscribeandpublish.h"


//typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> MySyncPolicy;



SubscribeAndPublish::SubscribeAndPublish()
    {
        //object_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("object_pcd_topic", 10);
        object_point_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("object_pcd_topic",2);
        point_cloud_subscriber = new message_filters::Subscriber<PointCloud2>(nh,"/camera/depth_registered/points",1);
        hand_point_cloud_subscriber = new message_filters::Subscriber<PointCloud2> (nh,"/hand_pcd",1);

        syn = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*point_cloud_subscriber, *hand_point_cloud_subscriber);
        syn->registerCallback(boost::bind(&SubscribeAndPublish::callback,this, _1,_2));
        std::cout<<"sss"<<endl;
    }
    
void SubscribeAndPublish::callback(const PointCloud2ConstPtr& point_cloud_msg, const PointCloud2ConstPtr& hand_point_cloud_msg)
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
    
            
            *object_point_cloud = seg_hand._seg_hand_out(scene_point_cloud, hand_point_cloud);
           
        
          
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
