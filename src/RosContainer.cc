#include "RosContainer.h"

using namespace std;

namespace ORB_SLAM2
{

RosContainer::RosContainer(int systemId, ros::NodeHandle* nodeHandler, MultiLoopClosing* loopCloser):
        systemId(systemId), nodeHandler(nodeHandler), globalLoopCloser(loopCloser), nNextLocalId(0)
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
    keyFramePublisher = nodeHandler->advertise<ORB_SLAM2::KeyFrameMsg>(topic, 10);
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
    pKF->localId = nNextLocalId++;
    globalLoopCloser->InsertKeyFrame(systemId, pKF);

    PublishKeyFrame(pKF);
}

void RosContainer::UpdateKeyFrame(KeyFrame *pKF)
{
    PublishKeyFrame(pKF);
}

void RosContainer::PublishKeyFrame(KeyFrame *pKF)
{
    set<MapPoint*> points = pKF->GetMapPoints();
    vector<geometry_msgs::Point32> vPoints;

    for(auto point : points)
    {
        // Attempt to select only good points that have been verified by multiple frames
        if (!point->isBad() && point->nObs > 1)
        {
            vPoints.push_back(MatToPoint32(point->GetWorldPos()));
        }
    }

    ORB_SLAM2::KeyFrameMsg kf;
    kf.header.frame_id = "world";
    kf.header.seq = GetKeyFrameSeq();
    kf.key_frame_id = pKF->mnFrameId;
    kf.origin = MatToPoint32(pKF->GetPoseInverse().rowRange(0,3).col(3)); // translation (x,y,z)
    kf.points = vPoints;

    keyFramePublisher.publish(kf);
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
