/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<sensor_msgs/PointCloud.h>

#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>
#include <include/System.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::Publisher* mapPub):mpSLAM(pSLAM), mapPub(mapPub){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* mapPub;
    ros::Publisher* posePub;

private:
    void PublishCameraPoseTf(cv::Mat pose);
    void PublishMapPoints(unsigned int seq);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MonoMultiple");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 MonoMultiple path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    ros::NodeHandle nodeHandler;

    ros::Publisher pub = nodeHandler.advertise<sensor_msgs::PointCloud>("/orb/cloud", 10);
    ImageGrabber igb(&SLAM, &pub);
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    if (pose.empty())
        return;

//    PublishCameraPoseTf(pose);
    PublishMapPoints(cv_ptr->header.seq);

//    cv::Mat Rwc = mTcw.rowRange(0,3).colRange(0,3).t();
//    cv::Mat twc = -Rwc*mTcw.rowRange(0,3).col(3);
//    tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
//                    Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
//                    Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
//    tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));
//
//    tf::Transform tfTcw(M,V);
//
//    tf::Quaternion Q = tfTcw.getRotation();
}

void ImageGrabber::PublishMapPoints(unsigned int seq) {
    // Get points and publish them
    const vector<ORB_SLAM2::MapPoint*> vpMPs = mpSLAM->GetPoints();
    const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpSLAM->GetReferencePoints();
    set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    vector<geometry_msgs::Point32> points;
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
//        cout << "Point is bad: " << vpMPs[i]->isBad() << endl;
//        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
//            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        geometry_msgs::Point32 point;
        point.x = pos.at<float>(0);
        point.y = pos.at<float>(1);
        point.z = pos.at<float>(2);
        points.push_back(point);
    }
    cout << "num points: " << points.size() << endl;

    sensor_msgs::PointCloud pointCloudMsg;
    pointCloudMsg.header.stamp = ros::Time::now();
    pointCloudMsg.header.frame_id = "camera_pose";
    pointCloudMsg.header.seq = seq;
    pointCloudMsg.points = points;

    mapPub->publish(pointCloudMsg);
}