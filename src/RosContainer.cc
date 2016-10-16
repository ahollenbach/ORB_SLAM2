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

    octree = new octomap::OcTree(0.001);
}

void RosContainer::InitCloudPublisher()
{
    string topic = topicStem + "cloud";
    pointCloudPublisher = nodeHandler->advertise<sensor_msgs::PointCloud>(topic, 10);
    pointCloud2Publisher = nodeHandler->advertise<sensor_msgs::PointCloud2>(topic + "2", 10);
    octomapPublisher = nodeHandler->advertise<octomap_msgs::Octomap>(topic + "/octomap",10);
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

    cv::Mat originMat = pKF->GetTranslation();
    octomap::point3d origin(originMat.at<float>(0),originMat.at<float>(1), originMat.at<float>(2));

    set<MapPoint*> points = pKF->GetMapPoints();
    octomap::Pointcloud cloud;

    for(auto point : points)
    {
        cv::Mat targetMat = point->GetWorldPos();
        cloud.push_back(targetMat.at<float>(0),targetMat.at<float>(1), targetMat.at<float>(2));
    }
    octree->insertPointCloud(cloud, origin);
    octree->updateInnerOccupancy();
    keyFrameOctoDb[pKF->mnId] = std::make_tuple(cloud, origin);

    octomap_msgs::Octomap bmap_msg;
    octomap_msgs::binaryMapToMsg(*octree, bmap_msg);
    octomapPublisher.publish(bmap_msg);
}

void RosContainer::NotifyStateChange(int state)
{
    stateChangePublisher.publish(state);
}

}
