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

#include "MultiLoopClosing.h"
#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "ORBmatcher.h"

#include<mutex>
#include<thread>
#include <fstream>
#include <ctime>


namespace ORB_SLAM2
{

MultiLoopClosing::MultiLoopClosing(vector<System*> pSystems, const bool bFixScale, const string &strSettingPath):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpSystems(pSystems), mLastLoopKFid(0),
    mbRunningGBA(false), mbFinishedGBA(true), mbStopGBA(false), mbFixScale(bFixScale)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mnCovisibilityConsistencyTh = fSettings["ORB.covisibilityConsistencyTh"]; // 3
    mpMatchedKF = NULL;

    // Queues for each of the sets of frames to pull off and join
    mlpLoopKeyFrameQueues = std::vector<std::list<KeyFrame*>>(pSystems.size(), std::list<KeyFrame*>());

    // Each map pairing gets a unique loop state to track similarity status for new KFs
    // This includes opposites (i.e. both map 0 -> map 1 and map 1 -> map 0 have loop states)
    loopStates = std::vector<std::vector<LoopState*>>(pSystems.size(), std::vector<LoopState*>());
    for (std::size_t i = 0, max = loopStates.size(); i < max; ++i)
    {
        for (std::size_t j = 0; j < max; ++j)
        {
            if(i == j)
            {
                loopStates[i].push_back(nullptr); // nullify self comparisons, as they won't be used
            } else
            {
                loopStates[i].push_back(new LoopState(i, j));
            }
        }
    }
}

void MultiLoopClosing::Run()
{
    mbFinished = false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
        }

        // ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void MultiLoopClosing::InsertKeyFrame(int sourceId, KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
    {
        mlpLoopKeyFrameQueues[sourceId].push_back(pKF);
    }
}

bool MultiLoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);

    // Search n->end, then 0->n, cycling through our maps, where n is +1 from where we last popped
    currentFrameQueueIndex = (currentFrameQueueIndex + 1) % mlpLoopKeyFrameQueues.size();
    for (std::size_t i = currentFrameQueueIndex, max = mlpLoopKeyFrameQueues.size(); i < max; ++i)
    {
        if(!mlpLoopKeyFrameQueues[i].empty())
        {
            currentFrameQueueIndex = i;
            return true;
        }
    }
    for (std::size_t i = 0, max = currentFrameQueueIndex; i < max; ++i)
    {
        if(!mlpLoopKeyFrameQueues[i].empty())
        {
            currentFrameQueueIndex = i;
            return true;
        }
    }

    return false;
}

bool MultiLoopClosing::DetectLoop()
{
    // Set the current KF and call DetectLoop(int, int) with source index and a
    // comparison map index to detect loops between maps
    uint sourceSystemIdx = currentFrameQueueIndex;

    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueues[sourceSystemIdx].front();
        mlpLoopKeyFrameQueues[sourceSystemIdx].pop_front();
        // Don't let keyframe be erased while it is being processed by this thread
        mpCurrentKF->SetNotErase();
    }

    for(std::size_t targetIdx=0; targetIdx<mpSystems.size(); targetIdx++)
    {
        if(sourceSystemIdx == targetIdx)
        {
            continue;
        }
        // TODO return array of loops to handle correctly
        if(DetectLoop(sourceSystemIdx, targetIdx))
        {
            return true;
        }
    }

    return false;
}


bool MultiLoopClosing::DetectLoop(int sourceSystemIdx, int comparisonSystemIdx)
{
    // Lock it to prevent other threads from double checking same state
    unique_lock<mutex> lock(mMutexLoopState);

    // If the map contains less than n KF or less than n KF have passed from last loop detection
    if(mpCurrentKF->localId < loopStates[sourceSystemIdx][comparisonSystemIdx]->mLastLoopKFid + 4)
    {
        mpCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(std::size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpSystems[sourceSystemIdx]->mpVocabulary->score(CurrentBowVec, BowVec); // TODO

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpSystems[comparisonSystemIdx]->mpKeyFrameDatabase->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
//        activeLoopState->mvConsistentGroups.clear(); TODOUNCOMM
        mpCurrentKF->SetErase();
        return false;
    }

    // Once we've settled on a candidate, set it as the active state so all methods can access it
    activeLoopState = loopStates[sourceSystemIdx][comparisonSystemIdx];

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    activeLoopState->mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(activeLoopState->mvConsistentGroups.size(),false);
    for(std::size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(std::size_t iG=0, iendG=activeLoopState->mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = activeLoopState->mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = activeLoopState->mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    activeLoopState->mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    activeLoopState->mvConsistentGroups = vCurrentConsistentGroups;

    if(activeLoopState->mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        // cout << sourceSystemIdx << "." << mpCurrentKF->localId << ": " << loopStates[sourceSystemIdx][comparisonSystemIdx]->mLastLoopKFid << endl;

        currentComparisonSystemIndex = comparisonSystemIdx;
        activeLoopState->mLastLoopKFid = mpCurrentKF->localId;

        // Correct the flip side too, so if we match 0.12 with 1.41, 1.41-1.51 won't match back again
        long unsigned int lastId = loopStates[comparisonSystemIdx][sourceSystemIdx]->mLastLoopKFid;
        for(size_t i=0;i<activeLoopState->mvpEnoughConsistentCandidates.size();i++)
        {
            if(activeLoopState->mvpEnoughConsistentCandidates[i]->localId > lastId)
            {
                loopStates[comparisonSystemIdx][sourceSystemIdx]->mLastLoopKFid = activeLoopState->mvpEnoughConsistentCandidates[i]->localId;
                lastId = loopStates[comparisonSystemIdx][sourceSystemIdx]->mLastLoopKFid;
            }
        }
        // cout << sourceSystemIdx << "." << mpCurrentKF->localId << ": " << loopStates[sourceSystemIdx][comparisonSystemIdx]->mLastLoopKFid << endl;

        return true;
    }
}

bool MultiLoopClosing::ComputeSim3()
{
    const int nInitialCandidates = activeLoopState->mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = activeLoopState->mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = activeLoopState->mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(std::size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    activeLoopState->mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
            activeLoopState->mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    activeLoopState->mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(std::size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    activeLoopState->mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, activeLoopState->mvpLoopMapPoints, activeLoopState->mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(std::size_t i=0; i<activeLoopState->mvpCurrentMatchedPoints.size(); i++)
    {
        if(activeLoopState->mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(activeLoopState->mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                activeLoopState->mvpEnoughConsistentCandidates[i]->SetErase();

        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            activeLoopState->mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }
}


int GBA_RUN_NUM = 0;
void logSim3(ofstream &ostream, int sourceIdx, int targetIdx, KeyFrame *parentKF, KeyFrame *adjustedKF, g2o::Sim3 *origS, g2o::Sim3 *newS)
{
    timeval curTime;
    gettimeofday(&curTime, NULL);
    long int ms = curTime.tv_sec * 1000 + curTime.tv_usec / 1000;

    // int milli = curTime.tv_usec / 1000;
    // char buffer [80];
    // strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));
    // char currentTime[84] = "";
    // sprintf(currentTime, "%s:%d", buffer, milli);

    ostream << ms << "|" <<
            GBA_RUN_NUM << "|" <<
            sourceIdx << "|" << 
            targetIdx << "|" << 
            parentKF->localId << "|" << parentKF->mnId << "|" << 
            adjustedKF->localId << "|" << adjustedKF->mnId << "|" <<
            // corrected << "|" << 
            newS->rotation().coeffs()[0] - origS->rotation().coeffs()[0] << "|" << 
            newS->rotation().coeffs()[1] - origS->rotation().coeffs()[1] << "|" << 
            newS->rotation().coeffs()[2] - origS->rotation().coeffs()[2] << "|" << 
            newS->rotation().coeffs()[3] - origS->rotation().coeffs()[3] << "|" << 
            newS->translation()[0] - origS->translation()[0] << "|" << 
            newS->translation()[1] - origS->translation()[1] << "|" << 
            newS->translation()[2] - origS->translation()[2] << "|" << 
            newS->scale() - origS->scale() << endl;
}

void MultiLoopClosing::CorrectLoop()
{
    // cout << "Correcting loop " << activeLoopState->sourceIdx << " to match " << activeLoopState->targetIdx << endl;
    cout << activeLoopState->sourceIdx << " -> " << activeLoopState->targetIdx << endl;
    // return;

    System* systemToCorrect = mpSystems[activeLoopState->sourceIdx];

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    systemToCorrect->mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        mbStopGBA = true;

        while(!isFinishedGBA())
            usleep(5000);

        mpThreadGBA->join();
        delete mpThreadGBA;
    }

    // Wait until Local Mapping has effectively stopped
    cout << activeLoopState->sourceIdx << " starting to wait" << endl;
    while(!systemToCorrect->mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    cout << activeLoopState->sourceIdx << " done waiting" << endl;

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrieve keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    activeLoopState->mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    activeLoopState->mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();

    {
        // Get Map Mutex
        unique_lock<mutex> lock(systemToCorrect->mpMap->mMutexMapUpdate);

        // ofstream outfile("/home/ahollenbach/data/results/gba/gba.log", ios::app);
        // if(!outfile.is_open())
        //     cout << "Couldn't open log file!!" << endl;
        // if(GBA_RUN_NUM == 0)
        // {
        //     timeval curTime;
        //     gettimeofday(&curTime, NULL);
        //     long int ms = curTime.tv_sec * 1000 + curTime.tv_usec / 1000;
        //     outfile << "------------------ " << ms << " ------------------" << endl;
        // }

        vector<KeyFrame*> allKFs = mpSystems[activeLoopState->sourceIdx]->mpMap->GetAllKeyFrames();
        for(vector<KeyFrame*>::iterator vit=allKFs.begin(), vend=allKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
            // g2o::Sim3 diff = CorrectedSim3[pKFi] - NonCorrectedSim3[pKFi];
            // logSim3(outfile, activeLoopState->sourceIdx, activeLoopState->targetIdx, mpCurrentKF, pKFi, &CorrectedSim3[pKFi], &NonCorrectedSim3[pKFi]);
        }
        // outfile.close();

        // Correct all MapPoints observed by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(std::size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }

        for (std::size_t i = 0, max = mpSystems.size(); i < max; ++i)
        {
            mpSystems[i]->SaveTrajectoryForClosure(GBA_RUN_NUM);
        }
        GBA_RUN_NUM++;

//        // Start Loop Fusion
//        // Update matched map points and replace if duplicated
//        for(std::size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
//        {
//            if(mvpCurrentMatchedPoints[i])
//            {
//                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
//                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
//                if(pCurMP)
//                    pCurMP->Replace(pLoopMP);
//                else
//                {
//                    mpCurrentKF->AddMapPoint(pLoopMP,i);
//                    pLoopMP->AddObservation(mpCurrentKF,i);
//                    pLoopMP->ComputeDistinctiveDescriptors();
//                }
//            }
//        }

        // This part is to figure out how many potential merges were made/turn them red in the UI
        /////////////////////////// QUERY ALL //////////////////////////////////////
        // vector<vector<KeyFrame*>> allCandidateKFs;

        // cout << "============= POTENTIAL ==============" << endl;
        // allKFs = mpSystems[activeLoopState->sourceIdx]->mpMap->GetAllKeyFrames();
        // for(vector<KeyFrame*>::iterator vit=allKFs.begin(), vend=allKFs.end(); vit!=vend; vit++)
        // {
        //      KeyFrame* pKFi = *vit;

        //      if(pKFi==mpCurrentKF)
        //      {
        //          continue;
        //      }

        //      const vector<KeyFrame*> vpConnectedKeyFrames = pKFi->GetVectorCovisibleKeyFrames();
        //      const DBoW2::BowVector &CurrentBowVec = pKFi->mBowVec;
        //      float minScore = 1;
        //      for(std::size_t i=0; i<vpConnectedKeyFrames.size(); i++)
        //      {
        //          KeyFrame* pKF = vpConnectedKeyFrames[i];
        //          if(pKF->isBad())
        //              continue;
        //          const DBoW2::BowVector &BowVec = pKF->mBowVec;

        //          float score = mpSystems[activeLoopState->sourceIdx]->mpVocabulary->score(CurrentBowVec, BowVec); // TODO

        //          if(score<minScore)
        //              minScore = score;
        //      }

        //     pKFi->isMergeCandidate = true;
        //     allCandidateKFs.push_back(mpSystems[activeLoopState->targetIdx]->mpKeyFrameDatabase->DetectLoopCandidates(pKFi, minScore));
        //     for(std::size_t i=0;i<allCandidateKFs[allCandidateKFs.size() - 1].size();i++)
        //     {
        //         allCandidateKFs[allCandidateKFs.size() - 1][i]->isMergeCandidate = true;
        //     }
        // }

        // int total = 0;
        // for(std::size_t i=0;i<allCandidateKFs.size();i++)
        // {
        //     cout << allCandidateKFs[i].size() << " ";
        //     total += allCandidateKFs[i].size();
        // }
        // cout << endl << "Total: " << total << endl;
        // cout << "=============================" << endl;
        //////////////////////////////////////////////////////////////////////////////
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    // SearchAndFuse(CorrectedSim3);


   // // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
   // map<KeyFrame*, set<KeyFrame*> > LoopConnections;

   // for(vector<KeyFrame*>::iterator vit=activeLoopState->mvpCurrentConnectedKFs.begin(), vend=activeLoopState->mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
   // {
   //     KeyFrame* pKFi = *vit;
   //     vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

   //     // Update connections. Detect new links.
   //     pKFi->UpdateConnections();
   //     LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
   //     for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
   //     {
   //         LoopConnections[pKFi].erase(*vit_prev);
   //     }
   //     for(vector<KeyFrame*>::iterator vit2=activeLoopState->mvpCurrentConnectedKFs.begin(), vend2=activeLoopState->mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
   //     {
   //         LoopConnections[pKFi].erase(*vit2);
   //     }
   // }

    // Optimize graph
   // Optimizer::OptimizeEssentialGraph(systemToCorrect->mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    // Add loop edge
    // mpMatchedKF->AddLoopEdge(mpCurrentKF);
    // mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&MultiLoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    systemToCorrect->mpLocalMapper->Release();

    cout << "MultiLoop Closed!" << endl;

    activeLoopState->mLastLoopKFid = mpCurrentKF->localId;
}

void MultiLoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(activeLoopState->mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,activeLoopState->mvpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        System* systemToCorrect = mpSystems[activeLoopState->sourceIdx];

        unique_lock<mutex> lock(systemToCorrect->mpMap->mMutexMapUpdate);
        const int nLP = activeLoopState->mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(activeLoopState->mvpLoopMapPoints[i]);
            }
        }
    }
}

vector<KeyFrame*> MultiLoopClosing::DetectAllRelocalizationCandidates(int sourceId, Frame *F)
{
    for(std::size_t idx=0; idx<mpSystems.size(); idx++)
    {
        int iidx = static_cast<int>(idx); // Silence warning
        if(iidx == sourceId)
        {
            continue;
        }

        vector<KeyFrame*> results = mpSystems[idx]->mpKeyFrameDatabase->DetectRelocalizationCandidates(F);
        if (results.size() > 0)
            return results;
    }

    return vector<KeyFrame*>();
}


void MultiLoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void MultiLoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        // TODO reset all
        mlpLoopKeyFrameQueues[0].clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void MultiLoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    System* systemToCorrect = mpSystems[activeLoopState->sourceIdx];

    Optimizer::GlobalBundleAdjustemnt(systemToCorrect->mpMap,20,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);


        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;

            // Wait until Local Mapping has effectively stopped
            systemToCorrect->mpLocalMapper->RequestStop();
            while(!systemToCorrect->mpLocalMapper->isStopped() && !systemToCorrect->mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(systemToCorrect->mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe, being sure not to double-correct any vertices
            list<KeyFrame*> lpKFtoCheck(systemToCorrect->mpMap->mvpKeyFrameOrigins.begin(),systemToCorrect->mpMap->mvpKeyFrameOrigins.end());
            set<int> visitedIds;
            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                visitedIds.insert(pKF->mnId);
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    if (visitedIds.count(pChild->mnId) == 0)
                    {
                        lpKFtoCheck.push_back(pChild);
                    }
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = systemToCorrect->mpMap->GetAllMapPoints();

            for(std::size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }

            systemToCorrect->mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void MultiLoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool MultiLoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void MultiLoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool MultiLoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

// http://docs.ros.org/indigo/api/rgbdslam/html/misc_8cpp_source.html#l00105
tf::Transform MultiLoopClosing::g2o2TF(const g2o::Sim3 sim3) {
    tf::Transform result;
    tf::Vector3 translation;
    translation.setX(sim3.translation().x());
    translation.setY(sim3.translation().y());
    translation.setZ(sim3.translation().z());
    tf::Quaternion rotation;
    rotation.setX(sim3.rotation().x());
    rotation.setY(sim3.rotation().y());
    rotation.setZ(sim3.rotation().z());
    rotation.setW(sim3.rotation().w());
    result.setOrigin(translation);
    result.setRotation(rotation);
    //printTransform("from conversion", result);
    return result;
}


} //namespace ORB_SLAM
