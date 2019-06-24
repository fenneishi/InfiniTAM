// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdexcept>

#include "../Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.h"
#include "../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../Trackers/Interface/ITMTracker.h"
#include "../Utils/ITMLibSettings.h"

/*--------------------------------------------------------------------------------------------------------------------------------------------long
VO所需的头文件*/
// 各种头文件
// C++标准库
#include <fstream>
#include <vector>
#include <map>
using namespace std;

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/common/transforms.h>
//#include <pcl/visualization/cloud_viewer.h> 在mac上，这个玩意一include进来，就会导致大量报错
#include <pcl/filters/voxel_grid.h>

//Sophus
//#include <sophus/so3.h>
//#include <sophus/se3.h>

// InfiniTAM 内部头文件
#include "../Utils/ITMImageTypes.h"
#include "../../ORUtils/MemoryDeviceType.h" // 调用T GetElement(int n, MemoryDeviceType memoryType)接口所需。
#include "../Utils/ITMMath.h"
#include "../../ORUtils/SE3Pose.h"
//------------------------------------------------------------------------------------------------------------------------------------------long




namespace ITMLib
{
	/** \brief
	*/


	class ITMTrackingController
	{
	private:
		const ITMLibSettings *settings;
		ITMTracker *tracker;
	public:

        void ITMUChar4Image_to_Mat(const ITMUChar4Image *rgb,cv::Mat &rgb_Mat)
        {
            std::cout<<"size.x:"<<rgb->noDims.x<<",size.y:"<<rgb->noDims.y<<std::endl;
            for (int m = 0; m < rgb->noDims.y; m++)
            {
                for (int n=0; n < rgb->noDims.x; n++)
                {
                    rgb_Mat.ptr<uchar>(m)[n*3+2]=rgb->GetElement(m*(rgb->noDims.x)+n,MEMORYDEVICE_CPU).r;
                    rgb_Mat.ptr<uchar>(m)[n*3+1]=rgb->GetElement(m*(rgb->noDims.x)+n,MEMORYDEVICE_CPU).g;
                    rgb_Mat.ptr<uchar>(m)[n*3]  =rgb->GetElement(m*(rgb->noDims.x)+n,MEMORYDEVICE_CPU).b;
//                    std::cout<<"r"<<(int)rgb->GetElement(m*(rgb->noDims.x)+n,MEMORYDEVICE_CPU).r;
//                    std::cout<<"g"<<(int)rgb->GetElement(m*(rgb->noDims.x)+n,MEMORYDEVICE_CPU).g;
//                    std::cout<<"b"<<(int)rgb->GetElement(m*(rgb->noDims.x)+n,MEMORYDEVICE_CPU).b<<" ";
                }
//                std::cout<<std::endl;
            }
        }

        void ITMFloatImage_to_Mat(const ITMFloatImage *depth,cv::Mat &depth_Mat)
        {
            for (int m = 0; m < depth->noDims.y; m++)
            {
                for (int n = 0; n < depth->noDims.x; n++) {
                    depth_Mat.ptr<float>(m)[n] = (float)depth->GetElement(m * (depth->noDims.x) + n, MEMORYDEVICE_CPU);
                }
            }
        }

        int RANSAC_long(vector< cv::KeyPoint > &kp_pre,vector< cv::KeyPoint > &kp_cur,vector< cv::DMatch > &goodMatches,const cv::Mat &rgb_prev_Mat,const cv::Mat &rgb_curr_Mat)
        {

            vector<cv::DMatch> m_Matches;
            m_Matches = goodMatches;
            int ptCount = goodMatches.size();
            if (ptCount < 100)
            {
                cout << "Don't find enough match points" << endl;
                return 0;
            }

            //坐标转换为float类型
            vector <cv::KeyPoint> RAN_KP1, RAN_KP2;
            //size_t是标准C库中定义的，应为unsigned int，在64位系统中为long unsigned int,在C++中为了适应不同的平台，增加可移植性。
            for (size_t i = 0; i < m_Matches.size(); i++)
            {
                RAN_KP1.push_back(kp_pre[goodMatches[i].queryIdx]);
                RAN_KP2.push_back(kp_cur[goodMatches[i].trainIdx]);
                //RAN_KP1是要存储img01中能与img02匹配的点
                //goodMatches存储了这些匹配点对的img01和img02的索引值
            }
            //坐标变换
            vector <cv::Point2f> p01, p02;
            for (size_t i = 0; i < m_Matches.size(); i++)
            {
                p01.push_back(RAN_KP1[i].pt);
                p02.push_back(RAN_KP2[i].pt);
            }
            /*vector <Point2f> img1_corners(4);
            img1_corners[0] = Point(0,0);
            img1_corners[1] = Point(img_1.cols,0);
            img1_corners[2] = Point(img_1.cols, img_1.rows);
            img1_corners[3] = Point(0, img_1.rows);
            vector <Point2f> img2_corners(4);*/
            ////求转换矩阵
            //Mat m_homography;
            //vector<uchar> m;
            //m_homography = findHomography(p01, p02, RANSAC);//寻找匹配图像
            //求基础矩阵 Fundamental,3*3的基础矩阵
            vector<uchar> RansacStatus;
            cv::Mat Fundamental = findFundamentalMat(p01, p02, RansacStatus, cv::FM_RANSAC);
            //重新定义关键点RR_KP和RR_matches来存储新的关键点和基础矩阵，通过RansacStatus来删除误匹配点
//            vector <cv::KeyPoint> RR_KP1, RR_KP2;
            kp_pre.clear();
            kp_cur.clear();
//            vector <cv::DMatch> RR_matches;
            goodMatches.clear();
            int index = 0;
            for (size_t i = 0; i < m_Matches.size(); i++)
            {
                if (RansacStatus[i] != 0)
                {
                    kp_pre.push_back(RAN_KP1[i]);
                    kp_cur.push_back(RAN_KP2[i]);
                    m_Matches[i].queryIdx = index;
                    m_Matches[i].trainIdx = index;
                    goodMatches.push_back(m_Matches[i]);
                    index++;
                }
            }
            cout << "RANSAC后匹配点数" <<goodMatches.size()<< endl;
            cv::Mat img_RR_matches;
            drawMatches(rgb_prev_Mat, kp_pre, rgb_curr_Mat,kp_cur, goodMatches, img_RR_matches);
            imshow("After RANSAC",img_RR_matches);
            cv::imwrite( "../../goodmatche_RANSAC.png", img_RR_matches);
            //等待任意按键按下
//            cv::waitKey(0);

        }

        int align(cv::KeyPoint &kp,const Eigen::Matrix3d &camera_matrix_depth,const Eigen::Matrix3d  &camera_matrix_rgb,const Eigen::Matrix4d  &depth_to_rgb)
        {
//          camera_matrix_depth检验
            if(!(                               camera_matrix_depth(0,1)==0&&
                camera_matrix_depth(1,0)==0&&
                camera_matrix_depth(2,0)==0&&   camera_matrix_depth(2,1)==0&&   camera_matrix_depth(2,2)==1
                ))
            {
                std::cout<<"camera_matrix_depth is wrong"<<std::endl;
                return -1;
            }
//          camera_matrix_rgb检验
            if(!(                             camera_matrix_rgb(0,1)==0&&
                camera_matrix_rgb(1,0)==0&&
                camera_matrix_rgb(2,0)==0&&   camera_matrix_rgb(2,1)==0&&   camera_matrix_rgb(2,2)==1
            ))
            {
                std::cout<<"camera_matrix_rgb is wrong"<<std::endl;
                return -1;
            }
//          depth_to_rgb检验
            if(!(depth_to_rgb(3,0)==0&&camera_matrix_depth(2,1)==0&&   camera_matrix_depth(2,2)==1
            ))
            {
                std::cout<<"depth_to_rgb is wrong"<<std::endl;
                return -1;
            }


//          rgb参考系：齐次化(u,v)-->(u,v,1)
            Eigen::Vector3d keypoint;
            keypoint<<kp.pt.x,kp.pt.y,1;
//          rgb参考系:归一化平面坐标
            keypoint=camera_matrix_rgb.inverse()*keypoint;
//          rgb参考系:齐次归一化平面坐标
            Eigen::Vector4d keypoint_1;
            keypoint_1<<keypoint[0],keypoint[1],keypoint[2],1;


//          depth参考系:齐次归一化平面坐标
            keypoint_1=depth_to_rgb*keypoint_1;
//          depth参考系:归一化平面坐标
            keypoint<<keypoint_1[0],keypoint_1[1],keypoint_1[2];
//          depth参考系:齐次像素坐标
            keypoint=camera_matrix_depth*keypoint;
            keypoint<<keypoint[0]/keypoint[2],keypoint[1]/keypoint[2],1;
//          depth参考系:像素坐标
            kp.pt.x=keypoint[0];
            kp.pt.y=keypoint[1];

            return 0;

        }

        // 彩色图对齐到深度图(仅特征点）
        int rgb_align_to_depth(const ITMView *view,vector< cv::DMatch > &gooMatches,  vector< cv::KeyPoint > &kps_pre,vector< cv::KeyPoint > &kps_curr )
        {

            // 深度相机内参
            double fx=(double)view->calib.intrinsics_d.projectionParamsSimple.fx;
            std::cout<<"fx"<<fx<<std::endl;
            double fy=(double)view->calib.intrinsics_d.projectionParamsSimple.fy;
            double cx=(double)view->calib.intrinsics_d.projectionParamsSimple.px;
            double cy=(double)view->calib.intrinsics_d.projectionParamsSimple.py;
            Eigen::Matrix3d camera_matrix_depth;
            camera_matrix_depth<<   fx,  0,   cx,
                                    0,   fy,  cy,
                                    0,   0,   1;
            std::cout<<"camera_matrix_depth:"<<camera_matrix_depth<<std::endl;


            // 彩色相机内参
            fx=view->calib.intrinsics_rgb.projectionParamsSimple.fx;
            fy=view->calib.intrinsics_rgb.projectionParamsSimple.fy;
            cx=view->calib.intrinsics_rgb.projectionParamsSimple.px;
            cy=view->calib.intrinsics_rgb.projectionParamsSimple.py;
            Eigen::Matrix3d  camera_matrix_rgb;
            camera_matrix_rgb << fx,  0,   cx,
                                 0,   fy,  cy,
                                 0,   0,   1 ;
            std::cout<<"camera_matrix_rgb:"<<camera_matrix_rgb<<std::endl;


            //  彩色相机_深度相机转换矩阵的逆
            Matrix4f calib_inv=view->calib.trafo_rgb_to_depth.calib_inv; //Matrix4f是infiniTAM自己定义的类型
            Eigen::Matrix4d  depth_to_rgb;
            depth_to_rgb<<
            calib_inv(0,0), calib_inv(0,1),calib_inv(0,2),calib_inv(0,3),
            calib_inv(1,0), calib_inv(1,1),calib_inv(0,2),calib_inv(0,3),
            calib_inv(2,0), calib_inv(2,1),calib_inv(2,2),calib_inv(2,3),
            calib_inv(3,0), calib_inv(3,1),calib_inv(3,2),calib_inv(3,3);
            std::cout<<"depth_to_rgb:"<<depth_to_rgb<<std::endl;



            for (size_t i=0; i<gooMatches.size(); i++)
            {
                align(kps_pre[gooMatches[i].queryIdx],camera_matrix_depth,camera_matrix_rgb,depth_to_rgb);
                align(kps_curr[gooMatches[i].queryIdx],camera_matrix_depth,camera_matrix_rgb,depth_to_rgb);
            }

            return 0;
        }

        int changeTrackingState(ITMTrackingState *trackingState,const cv::Mat &rvec,const cv::Mat &tvec)
        {
//            cout<<"R(0,0)"<<rvec.ptr<float>(0)[0];
//            cout<<"R(0,2)"<<rvec.ptr<float>(0)[1];
//            cout<<"t[1]"<<tvec.ptr<float>(0)[0];
//            cout<<"t[2]"<<tvec.ptr<float>(0)[1];
//            cout<<"t[3]"<<tvec.ptr<float>(0)[2];
            // R 李群上
            ORUtils::Matrix3<float> R(rvec.ptr<float>(0)[0],rvec.ptr<float>(0)[1],rvec.ptr<float>(0)[2],
                             rvec.ptr<float>(1)[0],rvec.ptr<float>(1)[1],rvec.ptr<float>(1)[2],
                             rvec.ptr<float>(2)[0],rvec.ptr<float>(2)[1],rvec.ptr<float>(2)[2]);
            // t 利群上
            ORUtils::Vector3<float> t(tvec.ptr<float>(0)[0],tvec.ptr<float>(0)[1],tvec.ptr<float>(0)[2]);

            // 相对位姿 李代数上
            ORUtils::SE3Pose pose_relative(R,t);

            // 当前帧位姿 李代数上（问题：上一帧位姿用trackingState->pose_pointCloud是否准确？）
            pose_relative.MultiplyWith(trackingState->pose_pointCloud);

            // 更新当前帧位姿
            trackingState->pose_d->SetFrom(  &pose_relative  );

            return 0;

        }


        void VO_initialize(ITMTrackingState *trackingState, const ITMView *view)
        {
            // 建立特征提取器与描述子提取器(ORB)
            cv::Ptr<cv::FeatureDetector> detector;
            cv::Ptr<cv::DescriptorExtractor> descriptor;
            detector = cv::ORB::create();
            descriptor = cv::ORB::create();
//          detector = cv::FeatureDetector::create("ROB");
//          descriptor = cv::DescriptorExtractor::create("ORB");



            //建立RGB彩色图
            cv::Mat rgb_prev_Mat(view->rgb_prev->noDims.y,view->rgb_prev->noDims.x,CV_8UC3);
            ITMUChar4Image_to_Mat(view->rgb_prev,rgb_prev_Mat);//格式转换：ITMUChar4Image--->cv:Mat(int rows, int cols, int type);
            cv::Mat rgb_curr_Mat(view->rgb->noDims.y,view->rgb->noDims.x,CV_8UC3);
            ITMUChar4Image_to_Mat(view->rgb,rgb_curr_Mat);
            // 显示转换结果rgb_prev_Mat;
            cv::namedWindow("rgb_prev", cv::WINDOW_AUTOSIZE);
            cv::imshow("rgb_prev", rgb_prev_Mat);
//            cv::waitKey(0);
            cv::destroyWindow("rgb_prev");



            // 提取关键点
            vector< cv::KeyPoint > kp_pre, kp_curr;
            detector->detect( rgb_prev_Mat, kp_pre );
            detector->detect( rgb_curr_Mat, kp_curr );
            // 计算描述子
            cv::Mat desp_pre, desp_curr;
            descriptor->compute( rgb_prev_Mat, kp_pre, desp_pre );
            descriptor->compute( rgb_curr_Mat, kp_curr, desp_curr );
            // 匹配描述子
            vector< cv::DMatch > matches;
            cv::FlannBasedMatcher matcher;
            // https://stackoverflow.com/questions/11565255/opencv-flann-with-orb-descriptors?answertab=votes#tab-top
            if(desp_pre.type()!=CV_32F) {
                desp_pre.convertTo(desp_pre, CV_32F);
            }
            if(desp_curr.type()!=CV_32F) {
                desp_curr.convertTo(desp_curr, CV_32F);
            }
            if(desp_pre.empty()||desp_curr.empty())
            {
                std::cout<<"descriptor empty"<<std::endl;
            } else
            {
                matcher.match( desp_pre, desp_curr, matches );
            }
            cout<<"Find total "<<matches.size()<<" matches."<<endl;
            // 可视化：显示匹配的特征
            cv::Mat imgMatches;
            cv::drawMatches( rgb_prev_Mat, kp_pre,rgb_curr_Mat,kp_curr, matches, imgMatches );
            cv::imshow( "matches", imgMatches );
            cv::imwrite( "../../matches.png", imgMatches );
//            cv::waitKey( 0 );
            // 筛选匹配，把距离太大的去掉:这里使用的准则是去掉大于四倍(1.5)最小距离的匹配
            vector< cv::DMatch > goodMatches;
            double minDis = 9999;
            for ( size_t i=0; i<matches.size(); i++ )
            {
                if ( matches[i].distance < minDis )
                    minDis = matches[i].distance;
            }
            for ( size_t i=0; i<matches.size(); i++ )
            {
                if (matches[i].distance < 4*minDis)
                    goodMatches.push_back( matches[i] );
            }
            cout<<"good matches="<<goodMatches.size()<<endl;
            // 可视化：显示筛选后的匹配结果。
            cv::drawMatches( rgb_prev_Mat, kp_pre,rgb_curr_Mat,kp_curr, goodMatches, imgMatches );
            cv::imshow( "goodmatches", imgMatches );
            cv::imwrite( "../../goodmatches.png", imgMatches );
//            cv::waitKey( 0 );



            // RANSAC滤波
            RANSAC_long(kp_pre,kp_curr,goodMatches,rgb_prev_Mat,rgb_curr_Mat);

            // 建立当前帧的深度图（格式cv::mat)
            cv::Mat depth_pre_Mat(view->depth->noDims.y,view->depth->noDims.x,CV_32FC1);
            ITMFloatImage_to_Mat(view->depth,depth_pre_Mat);


            // 将当前帧和前一帧彩色图中提取出的且是goodmatch特征点对齐到深度图(即用深度图的内参，彩色图的内参，深度图彩色图相对位姿，计算出新的uv）
            rgb_align_to_depth(view,goodMatches,kp_pre,kp_curr);



            // pnp准备——建立相机矩阵
            float fx=view->calib.intrinsics_d.projectionParamsSimple.fx;
            float fy=view->calib.intrinsics_d.projectionParamsSimple.fy;
            float cx=view->calib.intrinsics_d.projectionParamsSimple.px;
            float cy=view->calib.intrinsics_d.projectionParamsSimple.py;
            double camera_matrix_data[3][3] = {
                    {fx,  0,   cx},
                    {0,   fy,  cy},
                    {0,   0,   1}
            };
            cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );




            // pnp准备---建立当前帧gooodmatch关键点(3D形式，即为当前帧相机坐标系下的3D坐标)，前一帧goodmathch关键点(2D形式,即为像素坐标)
            vector<cv::Point3f> pts_obj; // 存放当前帧goodmatch特征点的3D坐标
            vector<cv::Point2f> pts_img;// 存放前一帧goodmathc特征点的2D坐标
            for (size_t i=0; i<goodMatches.size(); i++)
            {
                // 向pts_obj里添加点
                cv::Point2f p = kp_curr[goodMatches[i].trainIdx].pt;// query 是第一个, train 是第二个
                ushort d = depth_pre_Mat.ptr<float>((int)p.y)[(int)p.x];// 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
                if (d == 0) continue; // d==0的点不参与pnp优化。
                cv::Point3f p_xyz;
                p_xyz.z=d/1000;
                p_xyz.x=((p.x-cx)/fx)*p_xyz.z;
                p_xyz.y=((p.y-cy)/fy)*p_xyz.z;
                pts_obj.push_back( p_xyz );// 将(u,v,d)转成(x,y,z),并添加到pts_obj中.
                // 向pts_img里添加点
                pts_img.push_back( cv::Point2f( kp_pre[goodMatches[i].queryIdx].pt ) );
            }



//            显示pts_obj
//            pcl::PointCloud<pcl::PointXYZ>::Ptr pts_obj_pcl(new pcl::PointCloud<pcl::PointXYZ>);
//            pcl::PCDWriter writer;
//            int index_pcl=0;
//            for(auto point:pts_obj)
//            {
//                pts_obj_pcl->points[index_pcl].x=point.x;
//                pts_obj_pcl->points[index_pcl].y=point.y;
//                pts_obj_pcl->points[index_pcl].z=point.z;
//                index_pcl++;
//            }
//           writer.write<pcl::PointXYZ> ("../../../pts_obj_pcl", *pts_obj_pcl, false);



            // pnp求解：必须要确定清楚优化的是哪一帧的位姿。
            cv::Mat rvec, tvec, inliersPNP;
            cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliersPNP);



            // pnp结果
            cout<<"inliers: "<<inliersPNP.rows<<endl;
            cout<<"R="<<rvec<<endl;
            cout<<"t="<<tvec<<endl;



            // 修改trackingState SetFrom
            changeTrackingState(trackingState,rvec,tvec);
        }







		void Track(ITMTrackingState *trackingState, const ITMView *view)
		{
			std::cout<<"view state(noReSet)"<<view->rgb_prev->NoReSet<<std::endl;
			if(view->rgb_prev->NoReSet==0)
            {
			    VO_initialize(trackingState,view);
            }

		    tracker->TrackCamera(trackingState, view);
		}

		template <typename TSurfel>
		void Prepare(ITMTrackingState *trackingState, const ITMSurfelScene<TSurfel> *scene, const ITMView *view,
			const ITMSurfelVisualisationEngine<TSurfel> *visualisationEngine, ITMSurfelRenderState *renderState)
		{
			if (!tracker->requiresPointCloudRendering())
				return;

			//render for tracking
			bool requiresColourRendering = tracker->requiresColourRendering();
			bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;

			if(requiresColourRendering)
			{
				// TODO: This should be implemented at some point.
				throw std::runtime_error("The surfel engine doesn't yet support colour trackers");
			}
			else
			{
				const bool useRadii = true;
				visualisationEngine->FindSurface(scene, trackingState->pose_d, &view->calib.intrinsics_d, useRadii, USR_FAUTEDEMIEUX, renderState);
				trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

				if(requiresFullRendering)
				{
					visualisationEngine->CreateICPMaps(scene, renderState, trackingState);
					trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
					if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
					else trackingState->age_pointCloud = 0;
				}
				else
				{
					trackingState->age_pointCloud++;
				}
			}
		}

		template <typename TVoxel, typename TIndex>
		void Prepare(ITMTrackingState *trackingState, const ITMScene<TVoxel,TIndex> *scene, const ITMView *view,
			const ITMVisualisationEngine<TVoxel,TIndex> *visualisationEngine, ITMRenderState *renderState)
		{
			if (!tracker->requiresPointCloudRendering())
				return;

			//render for tracking
			bool requiresColourRendering = tracker->requiresColourRendering();
			bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;

			if (requiresColourRendering)
			{
				ORUtils::SE3Pose pose_rgb(view->calib.trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
				visualisationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib.intrinsics_rgb), renderState);
				visualisationEngine->CreatePointCloud(scene, view, trackingState, renderState, settings->skipPoints);
				trackingState->age_pointCloud = 0;
			}
			else
			{
				visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib.intrinsics_d), renderState);

				if (requiresFullRendering)
				{
					visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState);
					trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
					if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
					else trackingState->age_pointCloud = 0;
				}
				else
				{
					visualisationEngine->ForwardRender(scene, view, trackingState, renderState);
					trackingState->age_pointCloud++;
				}
			}
		}

		ITMTrackingController(ITMTracker *tracker, const ITMLibSettings *settings)
		{
			this->tracker = tracker;
			this->settings = settings;
		}

		const Vector2i& GetTrackedImageSize(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d) const
		{
			return tracker->requiresColourRendering() ? imgSize_rgb : imgSize_d;
		}

		// Suppress the default copy constructor and assignment operator
		ITMTrackingController(const ITMTrackingController&);
		ITMTrackingController& operator=(const ITMTrackingController&);
	};












}
