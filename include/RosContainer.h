#ifndef ROSCONTAINER_H
#define ROSCONTAINER_H

#include<string>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include "ORB_SLAM2/KeyFrameMsg.h"
//#include "KeyFrameMsg.h"
#include<std_msgs/Int32.h>
#include "MultiLoopClosing.h"
#include "Tracking.h"

namespace ORB_SLAM2
{

class MultiLoopClosing;

class RosContainer {
public:
    RosContainer(int systemId, ros::NodeHandle* nodeHandler, MultiLoopClosing* loopCloser);

    int GetSystemId();

    ros::Publisher keyFramePublisher;
    ros::Publisher pointCloudPublisher;
    ros::Publisher pointCloud2Publisher;
    ros::Publisher cameraPosePublisher;
    ros::Publisher stateChangePublisher;
//    ros::Publisher octomapPublisher;

    // Get and increment a seq id
    int GetKeyFrameSeq();
    int GetPointCloudSeq();

    void InsertKeyFrame(KeyFrame *pKF);

    // Publish a tracking state whenever it is changed. Of type class Tracking::eTrackingState
    void NotifyStateChange(int state);

private:
    std::string topicStem; // The topic stem, ie /orb1/
    int systemId;
    ros::NodeHandle* nodeHandler;
    MultiLoopClosing* globalLoopCloser;

    void InitCloudPublisher();
    void InitCameraPosePublisher();
    void InitKeyFramePublisher();
    void InitStateChangePublisher();

    void PublishKeyFrame(KeyFrame *pKF);
    void UpdateKeyFrame(KeyFrame *pKF);

    geometry_msgs::Point32 MatToPoint32(cv::Mat mat);

    int keyFrameSeq;
    int pointCloudSeq;

//    std::map<long,std::tuple<octomap::Pointcloud, octomap::point3d>> keyFrameOctoDb;
};

} //namespace ORB_SLAM

#endif //ROSCONTAINER_H
