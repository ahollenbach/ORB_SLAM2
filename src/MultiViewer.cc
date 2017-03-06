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

#include "MultiViewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

MultiViewer::MultiViewer(std::vector<ORB_SLAM2::System*> pSystems, const string &strSettingPath, bool showCameraFeeds):
    mpSystems(pSystems), mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false),
    showCameraFeeds(showCameraFeeds)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];



    // Set title string vars
    mapViewerTitle = "ORB-SLAM2: Map MultiViewer";
    displayName = "display";
    menuName = "menu";

    for (std::size_t i = 0, max = mpSystems.size(); i < max; ++i)
    {
        std::ostringstream oFrameTitle;
        oFrameTitle << "ORB-SLAM2: Current Frame " << mpSystems[i]->GetSystemId();
        frameTitles.push_back(oFrameTitle.str());
    }
}

void MultiViewer::Run()
{
    mbFinished = false;

    pangolin::CreateWindowAndBind(mapViewerTitle,1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel(menuName).SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera(menuName + ".Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints(menuName + ".Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames(menuName + ".Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph(menuName + ".Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode(menuName + ".Localization Mode",false,true);
    pangolin::Var<bool> menuReset(menuName + ".Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
            pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
    );

    pangolin::View& d_cam = pangolin::Display(displayName)
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    if(showCameraFeeds)
    {
        for (std::size_t i = 0, max = mpSystems.size(); i < max; ++i)
        {
            cv::namedWindow(frameTitles[i]);
            cv::moveWindow(frameTitles[i], 1921, 10 + i*540); // TODO temp
        }
    }

    bool bFollow = true;
    bool bLocalizationMode = false;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpSystems[0]->mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            for (std::size_t i = 0, max = mpSystems.size(); i < max; ++i)
            {
                mpSystems[i]->ActivateLocalizationMode();
            }
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            for (std::size_t i = 0, max = mpSystems.size(); i < max; ++i)
            {
                mpSystems[i]->DeactivateLocalizationMode();
            }
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);

        glClearColor(1.0f,1.0f,1.0f,1.0f);
        for (std::size_t i = 0, max = mpSystems.size(); i < max; ++i)
        {
            mpSystems[i]->mpMapDrawer->DrawCurrentCamera(Twc);
            if(menuShowKeyFrames || menuShowGraph)
                mpSystems[i]->mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
            if(menuShowPoints)
                mpSystems[i]->mpMapDrawer->DrawMapPoints();

            if(showCameraFeeds)
            {
                cv::Mat im = mpSystems[i]->mpFrameDrawer->DrawFrame();
                cv::imshow(frameTitles[i],im);
            }
            cv::waitKey(mT);
        }
        pangolin::FinishFrame();


        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
            {
                for (std::size_t i = 0, max = mpSystems.size(); i < max; ++i)
                {
                    mpSystems[i]->DeactivateLocalizationMode();
                }
            }
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            for (std::size_t i = 0, max = mpSystems.size(); i < max; ++i)
            {
                mpSystems[i]->Reset();
            }
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void MultiViewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool MultiViewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void MultiViewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool MultiViewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void MultiViewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool MultiViewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool MultiViewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void MultiViewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
