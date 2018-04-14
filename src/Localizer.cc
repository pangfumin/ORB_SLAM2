#include "Localizer.h"

#include <iostream>
#include <fstream>

#include "Map.h"
#include "KeyFrameDatabase.h"
#include "ORBmatcher.h"
#include "ORBextractor.h"
#include "PnPsolver.h"
#include "Optimizer.h"


namespace  ORB_SLAM2 {
    Localizer::Localizer(const std::string& mapFile,
                         const std::string& configFile,
                         const std::string& vocFile) {

        cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if(k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);


        cout << endl << "Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if(DistCoef.rows==5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;

        // Load ORB parameters

        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);


        cout << endl  << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;


        mpMap = new Map();
        mpORBVocabulary = new ORBVocabulary();
        mpORBVocabulary->loadFromBinaryFile(vocFile);
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpORBVocabulary);
        LoadMap(mapFile);
        RecoverMap();
    }

    bool Localizer::LocateFrame(const cv::Mat & frame, const double ts, Eigen::Isometry3d& T_wc ) {
        // Warp to Frame
        mCurrentFrame = Frame(frame,ts,mpORBextractor,mpORBVocabulary,mK,mDistCoef,0,0);
        bool success = Relocalization();

       return true;
    }


    void Localizer::LoadMap(const std::string &filename)
    {
        {
            std::ifstream is(filename);
            boost::archive::binary_iarchive ia(is, boost::archive::no_header);
            //ia >> mpKeyFrameDatabase;
            ia >> mpMap;
        }

        std::cout << std::endl << filename <<" : Map Loaded!" << std::endl;


    }

    void Localizer::SaveMap(const std::string &filename)
    {
        std::ofstream os(filename);
        {
            ::boost::archive::binary_oarchive oa(os, ::boost::archive::no_header);
            //oa << mpKeyFrameDatabase;
            oa << mpMap;
        }
        std::cout << std::endl << "Map saved to " << filename << std::endl;

    }

    void Localizer::RecoverMap(){
        vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it) {
            (*it)->SetKeyFrameDatabase(mpKeyFrameDatabase);
            (*it)->SetORBvocabulary(mpORBVocabulary);
            (*it)->SetMap(mpMap);
            (*it)->ComputeBoW();
            mpKeyFrameDatabase->add(*it);
            (*it)->SetMapPoints(mpMap->GetAllMapPoints());
            (*it)->SetSpanningTree(vpKFs);
            (*it)->SetGridParams(vpKFs);
        }

        // Reconstruct map points Observation
        vector<ORB_SLAM2::MapPoint*> vpMPs = mpMap->GetAllMapPoints();
        for (vector<ORB_SLAM2::MapPoint*>::iterator mit = vpMPs.begin(); mit != vpMPs.end(); ++mit) {
            (*mit)->SetMap(mpMap);
            (*mit)->SetObservations(vpKFs);
        }

        for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it) {
            (*it)->UpdateConnections();
        }


//        uint cnt = 0;
//        for (auto i : vpKFs){
//            //std::cout<<"Keyframe "<< i->mnId <<" is at "<<cnt << " with "<<i->GetMapPoints().size()<<" mappoints"<<std::endl;
//            mKeyframe_id_index[i->mnId] = cnt;
//            cnt ++;
//        }
//
//        cnt = 0;
//        for (auto i : vpMPs){
//
//            //std::cout<<"Mappoint "<< i->mnId <<" is at "<<cnt << " with "<<i->GetObservations().size()<<" observations"<<std::endl;
//            mMappoint_id_index[i->mnId] = cnt;
//            cnt ++;
//
//        }

//    for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it) {
//        if((*it)->mHasPrevKf){
//            long unsigned int prevKfId = (*it)->mPrevKeyframeId;
//            long unsigned int kfIndex = mPtrMap->mmKfIdIndexMap[prevKfId];
//            (*it)->SetPrevKeyFrame(vpKFs.at(kfIndex));
//
//            (*it)->ComputePreInt();
//        }
//    }
    }

    bool Localizer::Relocalization()
    {
        // Compute Bag of Words Vector
        mCurrentFrame.ComputeBoW();
        //cout << "Relocalization Initiated" << endl;

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame*> vpCandidateKFs
                = mpKeyFrameDatabase->DetectRelocalizationCandidates(&mCurrentFrame);

        if(vpCandidateKFs.empty())
            return false;


        const int nKFs = vpCandidateKFs.size();

        //cout << "Relocalization: candidates =  " << nKFs  << endl;

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75,true);

        vector<PnPsolver*> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint*> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates=0;



        for(int i=0; i<nKFs; i++)
        {
            KeyFrame* pKF = vpCandidateKFs[i];
            if(pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
                if(nmatches<15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {

                    PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }


        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9,true);

        while(nCandidates>0 && !bMatch)
        {

            for(int i=0; i<nKFs; i++)
            {
                if(vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver* pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if(bNoMore)
                {
                    vbDiscarded[i]=true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if(!Tcw.empty())
                {
                    Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint*> sFound;

                    const int np = vbInliers.size();

                    for(int j=0; j<np; j++)
                    {
                        if(vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j]=NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if(nGood<10)
                        continue;

                    for(int io =0; io<mCurrentFrame.N; io++)
                        if(mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    if(nGood<50)
                    {
                        //cout << "Relocalization:  inliers < 50 : nGood = " << nGood << endl;

                        int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                        if(nadditional+nGood>=50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if(nGood>30 && nGood<50)
                            {
                                sFound.clear();
                                for(int ip =0; ip<mCurrentFrame.N; ip++)
                                    if(mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                                // Final optimization
                                if(nGood+nadditional>=50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for(int io =0; io<mCurrentFrame.N; io++)
                                        if(mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io]=NULL;
                                }
                            }
                        }
                    }


                    // If the pose is supported by enough inliers stop ransacs and continue
                    if(nGood>=50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if(!bMatch)
        {
            return false;
        }
        else
        {
            cout << "Relocated" << endl;
            return true;
        }

    }
}