#ifndef SUBSCRIBEANDPUBLISH_H
#define SUBSCRIBEANDPUBLISH_H

#include "string.h"
#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv/cv.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "opencv/highgui.h"
#include "image_transport/image_transport.h"
#include "opencv/cv.h"
#include "sensor_msgs/image_encodings.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "seg_hand/pcl_segmentation.h"
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace cv;

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
    //SubscribeAndPublish();
    void callback(const PointCloud2ConstPtr& point_cloud_msg, const PointCloud2ConstPtr& hand_point_cloud_msg);


};

#endif
