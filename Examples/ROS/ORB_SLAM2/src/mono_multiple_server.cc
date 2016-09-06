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

    MonoServer(ORB_SLAM2::System* pSLAMcore, std::vector<ORB_SLAM2::System*> pSLAMs):mpSLAMserver(pSLAMcore), mpSLAMs(pSLAMs) {}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg, int source);

    // tmp
    void GrabImage1(const sensor_msgs::ImageConstPtr& msg);
    void GrabImage2(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAMserver;
    std::vector<ORB_SLAM2::System*> mpSLAMs;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MonoServer");
    ros::start();

    int num_clients = 1;
    if(argc == 4)
    {
        num_clients = atoi(argv[3]);
    }
    else if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 MonoServer path_to_vocabulary path_to_settings [num_clients=1]" << endl;
        ros::shutdown();
        return 1;
    }

    cout << "Will initiate " << num_clients << " clients" << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAMserver(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Make a list of clients
    std::vector<ORB_SLAM2::System*> SLAMclients(num_clients);

    for (int i = 0; i < num_clients; ++i)
    {
        cout << "Initializing client " << i << endl;
        SLAMclients[i] = new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    }

    MonoServer server(&SLAMserver, SLAMclients);

    ros::NodeHandle nodeHandler;
//    for (int i = 0; i < num_clients; ++i)
//    {
//        std::ostringstream o;
//        o << "/camera" << i << "/image_raw"; // Make topic for each source at /camera<i>/image_raw
//
//        // Wrap image fetcher with custom param for the source camera
////            nodeHandler.subscribe<sensor_msgs::ImageConstPtr>(o.str(), 1, boost::bind(&MonoServer::GrabImage, this, _1, i));
//        auto fn = i == 0 ? &MonoServer::GrabImage1 : &MonoServer::GrabImage2;
//
//        nodeHandler.subscribe(o.str(), 1, fn, &server);
//    }
    string topic = "/ardrone/image_raw";
    nodeHandler.subscribe(topic, 1, &MonoServer::GrabImage1, &server);

    cout << "Subscribing to: " << topic << endl;

    ros::spin();

    // Stop all threads
    SLAMserver.Shutdown();

    for (int i = 0; i < num_clients; ++i)
    {
        SLAMclients[i]->Shutdown();
    }

    ros::shutdown();

    return 0;
}

void MonoServer::GrabImage1(const sensor_msgs::ImageConstPtr& msg)
{
    GrabImage(msg, 1);
}
void MonoServer::GrabImage2(const sensor_msgs::ImageConstPtr& msg)
{
    GrabImage(msg, 2);
}

void MonoServer::GrabImage(const sensor_msgs::ImageConstPtr& msg, int sourceIdx)
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

    mpSLAMs[sourceIdx]->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


