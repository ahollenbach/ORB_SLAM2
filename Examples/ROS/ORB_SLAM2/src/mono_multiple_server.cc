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

#include<opencv2/core/core.hpp>

#include <include/System.h>

using namespace std;

class MonoServer
{
public:
    MonoServer(ORB_SLAM2::System* pSLAMcore, ORB_SLAM2::System* pSLAMs[]):mpSLAMserver(pSLAMcore), mpSLAMs(pSLAMs){}

    void GrabImage(int source, const sensor_msgs::ImageConstPtr& msg);

    void GrabImage1(const sensor_msgs::ImageConstPtr& msg);
    void GrabImage2(const sensor_msgs::ImageConstPtr& msg);


    ORB_SLAM2::System* mpSLAMserver;
    ORB_SLAM2::System* mpSLAMs[];
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MonoServer");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 MonoServer path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAMserver(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Make a list of clients
    const int NUM_CLIENTS = 1; // TODO: don't hardcode
    ORB_SLAM2::System* SLAMclients = new ORB_SLAM2::System[NUM_CLIENTS];

    for (int i = 0; i < NUM_CLIENTS; ++i) {
        SLAMclients[i] = ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    }

    MonoServer igb(&SLAMserver, &SLAMclients);

    ros::NodeHandle nodeHandler;
    nodeHandler.subscribe("/camera1/image_raw", 1, &MonoServer::GrabImage1,&igb);
    nodeHandler.subscribe("/camera2/image_raw", 1, &MonoServer::GrabImage2,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

// Messy, but let's do this for now to sort where things are coming from
void MonoServer::GrabImage1(const sensor_msgs::ImageConstPtr& msg)
{
    MonoServer::GrabImage(1, msg);
}

void MonoServer::GrabImage2(const sensor_msgs::ImageConstPtr& msg)
{
    MonoServer::GrabImage(2, msg);
}

void MonoServer::GrabImage(int source, const sensor_msgs::ImageConstPtr& msg)
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

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


