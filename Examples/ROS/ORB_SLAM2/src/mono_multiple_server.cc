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
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/PointCloud.h>

#include<opencv2/core/core.hpp>

#include<include/System.h>
#include<include/RosContainer.h>
#include<include/MultiViewer.h>
#include<include/MultiLoopClosing.h>

using namespace std;

class MonoServer
{
public:

    MonoServer(ORB_SLAM2::MultiLoopClosing* pGlobalLoopCloser, std::vector<ORB_SLAM2::System*> pSLAMs, const string &strSettingsFile):
            mpGlobalLoopCloser(pGlobalLoopCloser), mpSLAMs(pSLAMs) {
        viewer = new ORB_SLAM2::MultiViewer(pSLAMs, strSettingsFile);
        mptViewer = new thread(&ORB_SLAM2::MultiViewer::Run, viewer);

        mptGlobalLoopCLoser = new thread(&ORB_SLAM2::MultiLoopClosing::Run, mpGlobalLoopCloser);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg, int source);

    void Shutdown();

    void SetRosContainers(std::vector<ORB_SLAM2::RosContainer*> containers);

    ORB_SLAM2::MultiLoopClosing* mpGlobalLoopCloser;
    std::thread* mptGlobalLoopCLoser;

    std::vector<ORB_SLAM2::System*> mpSLAMs;

    ORB_SLAM2::MultiViewer* viewer;
    std::thread* mptViewer;

private:
    void PublishMapPoints(int idx, unsigned int seq);

    std::vector<ORB_SLAM2::RosContainer*> rosContainers;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MonoServer");
    ros::start();

    int num_clients = 2;
    if(argc == 4)
    {
        num_clients = atoi(argv[3]);
    }
    else if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 MonoServer path_to_vocabulary path_to_settings [num_clients=2]" << endl;
        ros::shutdown();
        return 1;
    }

    cout << "Initiating " << num_clients << " clients" << endl;

    // Some lovely colors
    vector<ORB_SLAM2::MapDrawer::Color*> colors;
    colors.push_back(new ORB_SLAM2::MapDrawer::Color(1.0f, 0.0f, 1.0f));
    colors.push_back(new ORB_SLAM2::MapDrawer::Color(0.0f, 1.0f, 1.0f));
    colors.push_back(new ORB_SLAM2::MapDrawer::Color(1.0f, 0.0f, 0.0f));

    // Make a list of clients
    std::vector<ORB_SLAM2::System*> SLAMclients(num_clients);

    for (int i = 0; i < num_clients; ++i)
    {
        cout << "Initializing client " << i << endl;
        SLAMclients[i] = new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,false,i);
        SLAMclients[i]->mpMapDrawer->SetPointColor(colors[i%colors.size()]);
    }

    ORB_SLAM2::MultiLoopClosing* pGlobalLoopCloser = new ORB_SLAM2::MultiLoopClosing(SLAMclients, false);

    MonoServer server(pGlobalLoopCloser, SLAMclients, argv[2]);

    ros::NodeHandle nodeHandler;
    std::vector<ORB_SLAM2::RosContainer*> containers(num_clients);
    std::vector<ros::Subscriber> subscribers(num_clients);
    for (int i = 0; i < num_clients; ++i)
    {
        // Construct topics for each client
        //  - Incoming feed will be read at /camera<i>/image_raw
        //  - Outgoing cloud is at /orb<i>/cloud
        std::ostringstream o;
        o << "/camera" << i << "/image_raw";

        subscribers[i] = nodeHandler.subscribe<sensor_msgs::Image>(o.str(), 1, boost::bind(&MonoServer::GrabImage, &server, _1, i));
        cout << "Client " << i << " subscribing to: " << o.str() << endl;

        containers[i] = new ORB_SLAM2::RosContainer(i, &nodeHandler, pGlobalLoopCloser);
    }

    server.SetRosContainers(containers);

    ros::spin();

    server.Shutdown();
    ros::shutdown();

    return 0;
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

    PublishMapPoints(sourceIdx, cv_ptr->header.seq);

}

void MonoServer::Shutdown()
{
    mpGlobalLoopCloser->RequestFinish();
    viewer->RequestFinish();

    for (std::size_t i = 0, max = mpSLAMs.size(); i < max; ++i)
    {
        mpSLAMs[i]->Shutdown();
    }

    // Wait until all thread have effectively stopped
    while(!mpGlobalLoopCloser->isFinished() || !viewer->isFinished())
    {
        usleep(5000);
    }

    pangolin::BindToContext("ORB-SLAM2: Map MultiViewer"); // Why?
}


void MonoServer::PublishMapPoints(int idx, unsigned int seq) {
    // Get points and publish them
    const vector<ORB_SLAM2::MapPoint*> vpMPs = mpSLAMs[idx]->GetPoints();
    const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpSLAMs[idx]->GetReferencePoints();
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

    sensor_msgs::PointCloud pointCloudMsg;
    pointCloudMsg.header.stamp = ros::Time::now();
    if(idx == 1)
    {
        pointCloudMsg.header.frame_id = "0to1";
    } else
    {
        pointCloudMsg.header.frame_id = "world";
    }
    pointCloudMsg.header.seq = seq;
    pointCloudMsg.points = points;

    rosContainers[idx]->pointCloudPublisher.publish(pointCloudMsg);
}

void MonoServer::SetRosContainers(std::vector<ORB_SLAM2::RosContainer*> containers)
{
    rosContainers = containers;

    for(size_t i=0, iend=mpSLAMs.size(); i<iend;i++) {
        mpSLAMs[i]->SetRosContainer(rosContainers[i]);
    }
}