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

    PublishKeyFrame(pKF);
}

void RosContainer::UpdateKeyFrame(KeyFrame *pKF)
{
    // By pushing new one, any listeners know that we've updated this frame
    PublishKeyFrame(pKF);
}

void RosContainer::PublishKeyFrame(KeyFrame *pKF)
{
    // Please fix this.

    set<MapPoint*> points = pKF->GetMapPoints();
    vector<geometry_msgs::Point32> vPoints;

    // NO THANK YOU
    vPoints.push_back(MatToPoint32(pKF->GetTranslation()));
    for(auto point : points)
    {
        vPoints.push_back(MatToPoint32(point->GetWorldPos()));
    }

    sensor_msgs::PointCloud cloud;
    // LIKE SERIOUSLY, set keyframe id as "frame_id" -_-
    cloud.header.frame_id = pKF->mnFrameId;
    cloud.header.seq = GetKeyFrameSeq();
    cloud.points = vPoints;
    keyFramePublisher.publish(cloud);

    // All of this is because I can't get a custom message type to build properly
    // I am so sorry.
}

geometry_msgs::Point32 RosContainer::MatToPoint32(cv::Mat mat)
{
    geometry_msgs::Point32 p;
    p.x = mat.at<float>(0);
    p.y = mat.at<float>(1);
    p.z = mat.at<float>(2);

    return p;
}

void RosContainer::NotifyStateChange(int state)
{
    stateChangePublisher.publish(state);
}

}
