#ifndef ROSCONTAINER_H
#define ROSCONTAINER_H

#include<string>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud.h>
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
    ros::Publisher cameraPosePublisher;
    ros::Publisher stateChangePublisher;

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

    int keyFrameSeq;
    int pointCloudSeq;
};

} //namespace ORB_SLAM

#endif //ROSCONTAINER_H
