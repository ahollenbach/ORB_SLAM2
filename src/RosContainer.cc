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
    RosContainer::InitKeyFramePublisher();

    keyFrameSeq = 0;
    pointCloudSeq = 0;
}

void RosContainer::InitCloudPublisher()
{
    string topic = topicStem + "cloud";
    pointCloudPublisher = nodeHandler->advertise<sensor_msgs::PointCloud>(topic, 10);
}

void RosContainer::InitKeyFramePublisher()
{
    string topic = topicStem + "keyframe";
    keyFramePublisher = nodeHandler->advertise<sensor_msgs::PointCloud>(topic, 10);
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
}
