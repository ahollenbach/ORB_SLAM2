#include "RosContainer.h"

using namespace std;

namespace ORB_SLAM2
{

RosContainer::RosContainer(int systemId, ros::NodeHandle* nodeHandler, MultiLoopClosing* loopCloser):
        systemId(systemId), nodeHandler(nodeHandler), globalLoopCloser(loopCloser)
{
    std::ostringstream topicStemStream;
    topicStemStream << "/orb" << systemId << "/";
    topicStem = topicStemStream.str();

    RosContainer::InitCloudPublisher();
    RosContainer::InitCameraPosePublisher();
    RosContainer::InitKeyFramePublisher();
    RosContainer::InitStateChangePublisher();

    keyFrameSeq = 0;
    pointCloudSeq = 0;
}

void RosContainer::InitCloudPublisher()
{
    string topic = topicStem + "cloud";
    pointCloudPublisher = nodeHandler->advertise<sensor_msgs::PointCloud>(topic, 10);
    pointCloud2Publisher = nodeHandler->advertise<sensor_msgs::PointCloud2>(topic + "2", 10);
}

void RosContainer::InitCameraPosePublisher()
{
    string topic = topicStem + "pose";
    cameraPosePublisher = nodeHandler->advertise<geometry_msgs::PoseStamped>(topic, 10);
}

void RosContainer::InitKeyFramePublisher()
{
    string topic = topicStem + "keyframe";
    keyFramePublisher = nodeHandler->advertise<sensor_msgs::PointCloud>(topic, 10);
}

void RosContainer::InitStateChangePublisher()
{
    string topic = topicStem + "trackingState";
    stateChangePublisher = nodeHandler->advertise<std_msgs::Int32>(topic, 1, true);
}

int RosContainer::GetKeyFrameSeq()
{
    return keyFrameSeq++;
}

int RosContainer::GetPointCloudSeq()
{
    return pointCloudSeq++;
}

void RosContainer::InsertKeyFrame(KeyFrame *pKF)
{
    globalLoopCloser->InsertKeyFrame(systemId, pKF);
}

void RosContainer::NotifyStateChange(int state)
{
    stateChangePublisher.publish(state);
}

}
