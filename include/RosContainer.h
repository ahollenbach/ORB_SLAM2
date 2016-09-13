#ifndef ROSCONTAINER_H
#define ROSCONTAINER_H

#include<string>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud.h>
#include "MultiLoopClosing.h"

namespace ORB_SLAM2
{

class MultiLoopClosing;

class RosContainer {
public:
    RosContainer(int systemId, ros::NodeHandle* nodeHandler, MultiLoopClosing* loopCloser);

    int GetSystemId();

    ros::Publisher keyFramePublisher;
    ros::Publisher pointCloudPublisher;

    // Get and increment a seq id
    int GetKeyFrameSeq();
    int GetPointCloudSeq();

    void InsertKeyFrame(KeyFrame *pKF);

private:
    std::string topicStem; // The topic stem, ie /orb1/
    int systemId;
    ros::NodeHandle* nodeHandler;
    MultiLoopClosing* globalLoopCloser;

    void InitCloudPublisher();
    void InitKeyFramePublisher();

    int keyFrameSeq;
    int pointCloudSeq;
};

} //namespace ORB_SLAM

#endif //ROSCONTAINER_H
