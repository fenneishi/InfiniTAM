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
#include <cmath>
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

// GMS
#include "gms_matcher.h"

// sift

#include <opencv2/xfeatures2d.hpp>

// InfiniTAM 内部头文件
#include "../Utils/ITMImageTypes.h"
#include "../../ORUtils/MemoryDeviceType.h" // 调用T GetElement(int n, MemoryDeviceType memoryType)接口所需。
#include "../Utils/ITMMath.h"
#include "../../ORUtils/SE3Pose.h"


//------------------------------------------------------------------------------------------------------------------------------------------long


class  QiLong
{
public:
    //-----------------------------------------------------------------------------------tool function---------------------------------------------------------
    cv::Mat DrawInlier(cv::Mat &src1, cv::Mat &src2, std::vector<KeyPoint> &kpt1, std::vector<KeyPoint> &kpt2, std::vector<DMatch> &inlier, int type) {
        const int height = std::max(src1.rows, src2.rows);
        const int width = src1.cols + src2.cols;
        cv::Mat output(height, width, CV_8UC3, Scalar(0, 0, 0));
        src1.copyTo(output(Rect(0, 0, src1.cols, src1.rows)));
        src2.copyTo(output(Rect(src1.cols, 0, src2.cols, src2.rows)));

        if (type == 1)
        {
            for (size_t i = 0; i < inlier.size(); i++)
            {
                cv::Point2f left = kpt1[inlier[i].queryIdx].pt;
                cv::Point2f right = (kpt2[inlier[i].trainIdx].pt + Point2f((float)src1.cols, 0.f));
                cv::line(output, left, right, Scalar(0, 255, 255));
            }
        }
        else if (type == 2)
        {
            for (size_t i = 0; i < inlier.size(); i++)
            {
                cv::Point2f left = kpt1[inlier[i].queryIdx].pt;
                cv::Point2f right = (kpt2[inlier[i].trainIdx].pt + Point2f((float)src1.cols, 0.f));
                cv::line(output, left, right, Scalar(255, 0, 0));
            }

            for (size_t i = 0; i < inlier.size(); i++)
            {
                cv::Point2f left = kpt1[inlier[i].queryIdx].pt;
                cv::Point2f right = (kpt2[inlier[i].trainIdx].pt + Point2f((float)src1.cols, 0.f));
                cv::circle(output, left, 1, Scalar(0, 255, 255), 2);
                cv::circle(output, right, 1, Scalar(0, 255, 0), 2);
            }
        }

        return output;
    }


    int align_pixel(
                int &u_new,int &v_new,int u_old,int v_old,float d,
                const Eigen::Matrix3d &camera_matrix_rgb,
                const Eigen::Matrix3d &camera_matrix_depth,
                const Eigen::Matrix4d &depth_to_rgb)
    {

        // --------------------check--------------------
        //camera_matrix_rgb
        if
        (!(                         camera_matrix_rgb(0,1)==0&&
        camera_matrix_rgb(1,0)==0&&
        camera_matrix_rgb(2,0)==0&& camera_matrix_rgb(2,1)==0&& camera_matrix_rgb(2,2)==1
        ))
        {
            std::cout<<"camera_matrix_rgb is wrong"<<std::endl;
            return -1;
        }
        //camera_matrix_depth
        if
        (!(                           camera_matrix_depth(0,1)==0&&
        camera_matrix_depth(1,0)==0&&
        camera_matrix_depth(2,0)==0&& camera_matrix_depth(2,1)==0&&   camera_matrix_depth(2,2)==1
        ))
        {
        std::cout<<"camera_matrix_depth is wrong"<<std::endl;
        return -1;
        }
        //depth_to_rgb
        if
        (!( depth_to_rgb(3,0)==0 &&
            depth_to_rgb(3,1)==0 &&
            depth_to_rgb(3,2)==0 &&
            depth_to_rgb(3,3)==1 ) )
        {
        std::cout<<"depth_to_rgb is wrong"<<std::endl;
        return -1;
        }


        // --------------------transform--------------------
        // depth coordinate system
        Eigen::Vector3d point;
        point<<u_old,v_old,1;
//        Eigen:Vector3d point3D; // 用来检测 point=camera_matrix_depth.inverse()*point;是否正确
//        point3D[0]=(point[0]-camera_matrix_depth(0,2))/camera_matrix_depth(0,0)*d;
//        point3D[1]=(point[1]-camera_matrix_depth(1,2))/camera_matrix_depth(1,1)*d;
//        point3D[2]=d;
        point=camera_matrix_depth.inverse()*point;
        point=d*point;
        Eigen::Vector4d point4D;
        point4D<<point[0],point[1],point[2],1;
        // rgb coordinate system
        point4D=depth_to_rgb*point4D;
        point<<point4D[0],point4D[1],point4D[2];
        point<<point[0]/point[2],point[1]/point[2],1;
        point=camera_matrix_rgb*point;


        // --------------------result--------------------
        u_new=point[0];
        v_new=point[1];

        return 0;
    }



    int align_depthImage(const ITMLib::ITMView *view,cv::Mat &depth_Mat)
    {
//        // -----------------------------------------qilong:test----------------------------------------------------------------------------
//        for (int r = 0; r < view->depth->noDims.y; r++)
//        {
//            for (int c = 0; c < view->depth->noDims.x; c++) {
//                // d
//                auto d=view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU);
//                std::cout<<view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU)<<",";
//            }
//            std::cout<<std::endl;
//        }
//        std::cout<<"-----------------------------------------qilong:test8----------------------------------------------------------------------------"<<std::endl;
//        // -----------------------------------------qilong:test----------------------------------------------------------------------------


        // 彩色相机内参<Eigen::Matrix3d>
        float fx=view->calib.intrinsics_rgb.projectionParamsSimple.fx;
        float fy=view->calib.intrinsics_rgb.projectionParamsSimple.fy;
        float cx=view->calib.intrinsics_rgb.projectionParamsSimple.px;
        float cy=view->calib.intrinsics_rgb.projectionParamsSimple.py;
        Eigen::Matrix3d  camera_matrix_rgb;
        camera_matrix_rgb <<
                fx,   0,  cx,
                0,   fy,  cy,
                0,   0,   1 ;
//        std::cout<<"camera_matrix_rgb:"<<std::endl<<camera_matrix_rgb<<std::endl;


        // 深度相机内参<Eigen::Matrix3d>
        fx=view->calib.intrinsics_d.projectionParamsSimple.fx;
        fy=view->calib.intrinsics_d.projectionParamsSimple.fy;
        cx=view->calib.intrinsics_d.projectionParamsSimple.px;
        cy=view->calib.intrinsics_d.projectionParamsSimple.py;
        Eigen::Matrix3d camera_matrix_depth;
        camera_matrix_depth<<
               fx,   0,  cx,
                0,  fy,  cy,
                0,   0,   1;
//        std::cout<<"camera_matrix_depth:"<<std::endl<<camera_matrix_depth<<std::endl;


        //  外参：rgb_to_depth<Eigen::Matrix4d>
        Matrix4f calib=view->calib.trafo_rgb_to_depth.calib; //Matrix4f是infiniTAM自己定义的类型
        Eigen::Matrix4d  rgb_to_depth;
        rgb_to_depth<<
                calib.m00,calib.m10,calib.m20,calib.m30,//第1行
                calib.m01,calib.m11,calib.m21,calib.m31,//第2行
                calib.m02,calib.m12,calib.m22,calib.m32,//第3行
                calib.m03,calib.m13,calib.m23,calib.m33;//第4行
//                std::cout<<"rgb_to_depth:"<<std::endl<<rgb_to_depth<<std::endl;

        //  外参：depth_to_rgb<Eigen::Matrix4d>
        Matrix4f calib_inv=view->calib.trafo_rgb_to_depth.calib_inv; //Matrix4f是infiniTAM自己定义的类型
        Eigen::Matrix4d  depth_to_rgb;
        depth_to_rgb<<
                calib_inv.m00,calib_inv.m10,calib_inv.m20,calib_inv.m30,//第1行
                calib_inv.m01,calib_inv.m11,calib_inv.m21,calib_inv.m31,//第2行
                calib_inv.m02,calib_inv.m12,calib_inv.m22,calib_inv.m32,//第3行
                calib_inv.m03,calib_inv.m13,calib_inv.m23,calib_inv.m33;//第4行
//                std::cout<<"depth_to_rgb:"<<std::endl<<depth_to_rgb<<std::endl;

        // depth_Mat初始化
        cv::Mat testChar;
        cv::Mat testfloat;
        for (int r = 0; r < view->depth->noDims.y; r++)
        {
            for (int c = 0; c <view->depth->noDims.x; c++) {
                depth_Mat.ptr<double>(r)[c] = (double)0;
            }
        }

//        // -----------------------------------------qilong:test----------------------------------------------------------------------------
//        for (int r = 0; r < view->depth->noDims.y; r++)
//        {
//            for (int c = 0; c < view->depth->noDims.x; c++) {
//                // d
//                auto d=view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU);
//                std::cout<<view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU)<<",";
//            }
//            std::cout<<std::endl;
//        }
//        std::cout<<"-----------------------------------------qilong:test9----------------------------------------------------------------------------"<<std::endl;
//        // -----------------------------------------qilong:test----------------------------------------------------------------------------

//        std::cout<<"float:"<<sizeof(float)<<std::endl;
//        std::cout<<"double:"<<sizeof(double)<<std::endl;
        // depth_Mat赋值
        for (int r = 0; r < view->depth->noDims.y; r++)
        {
            for (int c = 0; c < view->depth->noDims.x; c++)
            {
                // d
                auto d=view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU);
//                std::cout<<view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU)<<",";
                // aligen(对于测不到的点，d=-1)
                if(d>0)
                {
                    // [u_old,v_old]
                    int u_old=c;
                    int v_old=r;
                    // [u_new,v_new]
                    int u_new=u_old;
                    int v_new=v_old;

//                    align_pixel(u_new,v_new,u_old,v_old,d,camera_matrix_rgb,camera_matrix_depth,rgb_to_depth);
                    align_pixel(u_new,v_new,u_old,v_old,d,camera_matrix_rgb,camera_matrix_depth,depth_to_rgb);
                    // undata new depth_Mat
                    if( 0<=u_new&&u_new<640 && 0<=v_new&&v_new<480 )
                    {
                        depth_Mat.ptr<double>(v_new)[u_new]=(double)d;
//                        std::cout<<"("<<v_new<<","<<u_new<<","<<d<<")";
                    }
                }
            }
        }

        for (int r = 0; r < view->depth->noDims.y; r++)
        {
            for (int c = 0; c <view->depth->noDims.x; c++) {
//                std::cout<<depth_Mat.ptr<double>(r)[c]<<",";
            }
        }
        return 0;
    }



    int  ITMUChar4Image_to_CVMat(const ITMUChar4Image *rgb,cv::Mat &rgb_Mat)
    {
        for (int r = 0; r < rgb->noDims.y; r++)
        {
            for (int c=0; c < rgb->noDims.x; c++)
            {
                rgb_Mat.ptr<uchar>(r)[c*3+2]=rgb->GetElement( r*(rgb->noDims.x)+c , MEMORYDEVICE_CPU ).r;
                rgb_Mat.ptr<uchar>(r)[c*3+1]=rgb->GetElement( r*(rgb->noDims.x)+c , MEMORYDEVICE_CPU ).g;
                rgb_Mat.ptr<uchar>(r)[c*3]  =rgb->GetElement( r*(rgb->noDims.x)+c , MEMORYDEVICE_CPU ).b;
            }
       }
       return 0;
    }

    //-------------------------------------------------------------------------------------step function-----------------------------------------------------------

    // step_1
    int feature_matching_ORB(
            const ITMLib::ITMView *view,
            vector< cv::KeyPoint > &kps_pre,
            vector< cv::KeyPoint > &kps_curr,
            vector< cv::DMatch > &matches)
    {
        // 建立特征提取器与描述子提取器(ORB)
        cv::Ptr<cv::FeatureDetector> detector;
        cv::Ptr<cv::DescriptorExtractor> descriptor;
        detector = cv::ORB::create();
        descriptor = cv::ORB::create();
        //建立RGB彩色图<cv::Mat>
        int width=view->calib.intrinsics_rgb.imgSize.width;
        int height=view->calib.intrinsics_rgb.imgSize.height;
        cv::Mat rgb_prev_Mat(height,width,CV_8UC3);
        ITMUChar4Image_to_CVMat(view->rgb_prev,rgb_prev_Mat);
        cv::Mat rgb_curr_Mat(height,width,CV_8UC3);
        ITMUChar4Image_to_CVMat(view->rgb,rgb_curr_Mat);
        // 提取关键点
        detector->detect( rgb_prev_Mat, kps_pre );
        detector->detect( rgb_curr_Mat, kps_curr );
        // 计算描述子
        cv::Mat desps_pre, desps_curr;
        descriptor->compute( rgb_prev_Mat, kps_pre, desps_pre );
        descriptor->compute( rgb_curr_Mat, kps_curr, desps_curr );
        // 匹配描述子
        cv::FlannBasedMatcher matcher;
        // https://stackoverflow.com/questions/11565255/opencv-flann-with-orb-descriptors?answertab=votes#tab-top
        if(desps_pre.type()!=CV_32F) {
            desps_pre.convertTo(desps_pre, CV_32F);
        }
        if(desps_curr.type()!=CV_32F) {
            desps_curr.convertTo(desps_curr, CV_32F);
        }
        if(desps_pre.empty()||desps_curr.empty())
        {
            std::cout<<"descriptor empty"<<std::endl;
        } else
        {
            matcher.match( desps_pre, desps_curr, matches );
        }
        cout<<"Find total "<<matches.size()<<" matches."<<endl;
        // 可视化：显示匹配的特征
        cv::Mat imgMatches;
        cv::drawMatches( rgb_prev_Mat, kps_pre,rgb_curr_Mat,kps_curr, matches, imgMatches );
        cv::imshow( "matches", imgMatches );
        cv::imwrite( "../../matches.png", imgMatches );
        return 0;
    }

    int feature_matching_GMS_ORB(
            const ITMLib::ITMView *view,
            vector< cv::KeyPoint > &kps_pre,
            vector< cv::KeyPoint > &kps_curr,
            vector< cv::DMatch > &matches_gms)
    {

        // 建立特征提取器与描述子提取器(ORB)
        Ptr<cv::ORB> orb = cv::ORB::create(100000);
        orb->setFastThreshold(0);
        //建立RGB彩色图<cv::Mat>
        int width=view->calib.intrinsics_rgb.imgSize.width;
        int height=view->calib.intrinsics_rgb.imgSize.height;
        cv::Mat rgb_prev_Mat(height,width,CV_8UC3);
        ITMUChar4Image_to_CVMat(view->rgb_prev,rgb_prev_Mat);
        cv::Mat rgb_curr_Mat(height,width,CV_8UC3);
        ITMUChar4Image_to_CVMat(view->rgb,rgb_curr_Mat);
        // 提取关键点并计算描述子
        cv::Mat desps_pre, desps_curr;
        orb->detectAndCompute(rgb_prev_Mat, Mat(), kps_pre, desps_pre);
        orb->detectAndCompute(rgb_curr_Mat, Mat(), kps_curr, desps_curr);
        // 初步匹配
        // 暴力匹配，汉明距离
        vector<cv::DMatch> matches_all;
        BFMatcher matcher(NORM_HAMMING);
        matcher.match(desps_pre,desps_curr, matches_all);
        // GMS filter
        std::vector<bool> vbInliers;
        gms_matcher gms(kps_pre, rgb_prev_Mat.size(), kps_curr,rgb_curr_Mat.size(),matches_all);
        cout << "GMS::Get total " << gms.GetInlierMask(vbInliers, false, false) << " matches." << endl;
//         collect matches
        for (size_t i = 0; i < vbInliers.size(); ++i)
        {
            if (vbInliers[i] == true)
            {
                matches_gms.push_back(matches_all[i]);
            }
        }
        // draw matching
        cv::Mat show = DrawInlier(rgb_prev_Mat, rgb_curr_Mat, kps_pre, kps_curr, matches_gms, 1);
        cv::imshow("show", show);
        cv::waitKey();

        return 0;
    }


    int feature_matching_GMS_SIFT(
            const ITMLib::ITMView *view,
            vector< cv::KeyPoint > &kps_pre,
            vector< cv::KeyPoint > &kps_curr,
            vector< cv::DMatch > &matches_all)
    {

        // 建立特征提取器与描述子提取器(sift)
        Ptr<cv::Feature2D> sift = cv::xfeatures2d::SURF::create();
        //建立RGB彩色图<cv::Mat>
        int width=view->calib.intrinsics_rgb.imgSize.width;
        int height=view->calib.intrinsics_rgb.imgSize.height;
        cv::Mat rgb_prev_Mat(height,width,CV_8UC3);
        ITMUChar4Image_to_CVMat(view->rgb_prev,rgb_prev_Mat);
        cv::Mat rgb_curr_Mat(height,width,CV_8UC3);
        ITMUChar4Image_to_CVMat(view->rgb,rgb_curr_Mat);
        // 提取关键点并计算描述子
        cv::Mat desps_pre, desps_curr;
        sift->detect(rgb_prev_Mat, kps_pre);
        sift->detect(rgb_curr_Mat, kps_curr);
        sift->compute(rgb_prev_Mat, kps_pre, desps_pre);
        sift->compute(rgb_curr_Mat, kps_curr, desps_curr);
        // 初步匹配
        // 暴力匹配，汉明距离
//        vector<cv::DMatch> matches_all;
//        BFMatcher matcher(NORM_HAMMING);
//        BFMatcher matcher;
        cv::FlannBasedMatcher matcher;
        if(desps_pre.type()!=CV_32F) {
            desps_pre.convertTo(desps_pre, CV_32F);
        }
        if(desps_curr.type()!=CV_32F) {
            desps_curr.convertTo(desps_curr, CV_32F);
        }
        if(desps_pre.empty()||desps_curr.empty())
        {
            std::cout<<"descriptor empty"<<std::endl;
        } else
        {
            matcher.match(desps_pre,desps_curr, matches_all);
        }
        cout<<"Find total "<<matches_all.size()<<" matches."<<endl;
        // 可视化：显示匹配的特征
        cv::Mat imgMatches;
        cv::drawMatches( rgb_prev_Mat, kps_pre,rgb_curr_Mat,kps_curr, matches_all, imgMatches );
        cv::imshow( "matches", imgMatches );
        cv::imwrite( "../../matches.png", imgMatches );

//        // GMS filter
//        std::vector<bool> vbInliers;
//        gms_matcher gms(kps_pre, rgb_prev_Mat.size(), kps_curr,rgb_curr_Mat.size(),matches_all);
//        cout << "GMS::Get total " << gms.GetInlierMask(vbInliers, false, false) << " matches." << endl;
////         collect matches
//        for (size_t i = 0; i < vbInliers.size(); ++i)
//        {
//            if (vbInliers[i] == true)
//            {
//                matches_gms.push_back(matches_all[i]);
//            }
//        }
//        cout<<"Find total "<<matches_all.size()<<" matches."<<endl;
//        // draw matching
//        cv::Mat show = DrawInlier(rgb_prev_Mat, rgb_curr_Mat, kps_pre, kps_curr, matches_gms, 1);
//        cv::imshow("show", show);
//        cv::waitKey();

        return 0;
    }




    // step_2
    int pnp(
            const ITMLib::ITMView *view,
            const vector< cv::KeyPoint > &kps_pre,
            const vector< cv::KeyPoint > &kps_curr,
            const vector< cv::DMatch > &matches,
            ORUtils::SE3Pose &T_CurrToPre )
    {

//        // -----------------------------------------qilong:test----------------------------------------------------------------------------
//        for (int r = 0; r < view->depth->noDims.y; r++)
//        {
//            for (int c = 0; c < view->depth->noDims.x; c++) {
//                // d
//                auto d=view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU);
//                std::cout<<view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU)<<",";
//            }
//            std::cout<<std::endl;
//        }
//        std::cout<<"-----------------------------------------qilong:test6----------------------------------------------------------------------------"<<std::endl;
//        // -----------------------------------------qilong:test----------------------------------------------------------------------------
        // image size
        // orb
        int width=view->calib.intrinsics_rgb.imgSize.width;
        int height=view->calib.intrinsics_rgb.imgSize.height;
        // depth
        int width_d=view->calib.intrinsics_d.imgSize.width;
        int height_d=view->calib.intrinsics_d.imgSize.height;



        // ---------------------------------------------pnp准备:建立相机矩阵---------------------------------------------
        float fx=view->calib.intrinsics_rgb.projectionParamsSimple.fx;
        float fy=view->calib.intrinsics_rgb.projectionParamsSimple.fy;
        float cx=view->calib.intrinsics_rgb.projectionParamsSimple.px;
        float cy=view->calib.intrinsics_rgb.projectionParamsSimple.py;
        double camera_matrix_data[3][3] = {
                {fx,  0,   cx},
                {0,   fy,  cy},
                {0,   0,   1}
        };
        cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );

//        // -----------------------------------------qilong:test----------------------------------------------------------------------------
//        for (int r = 0; r < view->depth->noDims.y; r++)
//        {
//            for (int c = 0; c < view->depth->noDims.x; c++) {
//                // d
//                auto d=view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU);
//                std::cout<<view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU)<<",";
//            }
//            std::cout<<std::endl;
//        }
//        std::cout<<"-----------------------------------------qilong:test7----------------------------------------------------------------------------"<<std::endl;
//        // -----------------------------------------qilong:test----------------------------------------------------------------------------
        // ---------------------------------------------pnp准备:对齐到rgb的深度图(仅当前帧)-----------------------------------
        cv::Mat depth_curr_Mat_align(height_d,width_d,CV_64FC1);
        align_depthImage(view,depth_curr_Mat_align);



        // ---------------------------------------------pnp准备:建立3Dpoints,2Dpoints--------------------------------------
        // creat
        vector<cv::Point2f> pts_img;// pre image 2Dpoints
        vector<cv::Point3f> pts_obj; // curr image 3Dpoints
        // fill pts_img and pts_obj
        for (size_t i=0; i<matches.size(); i++)
        {
            // extract
            cv::Point2f p_pre  = kps_pre[matches[i].queryIdx].pt;
            cv::Point2f p_curr = kps_curr[matches[i].trainIdx].pt;


            // get depth and check
            double d=depth_curr_Mat_align.ptr<double>((int)p_curr.y)[(int)p_curr.x];
//            std::cout<<"("<<p_curr.y<<","<<p_curr.x<<","<<d<<")";
            if (d == 0) continue; // d==0的点不参与pnp优化。

            // fill pts_obj
            cv::Point3f p_xyz;
            p_xyz.z=d;//注意，这里的d不是从原始图片获取的，所以应该单位应该换算成m了
            p_xyz.x=((p_curr.x-cx)/fx)*p_xyz.z;
            p_xyz.y=((p_curr.y-cy)/fy)*p_xyz.z;
            pts_obj.push_back( p_xyz );// 将(u,v,d)转成(x,y,z),并添加到pts_obj中.

            // fill pts_img
            pts_img.push_back( cv::Point2f(p_pre) );
        }


        // ---------------------------------------------pnp求解--------------------------------------
        cv::Mat rvec, tvec, inliersPNP;
        cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliersPNP);
//        cv::solvePnP( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec);
//        cout<<"PnP_result::inliers: "<<inliersPNP.rows<<endl;
//        cout<<"PnP_result::R="<<rvec<<endl;//注意这个R是李代数上面的
//        cout<<"PnP_result::t="<<tvec<<endl;


        // ---------------------------------------------pnp结果处理--------------------------------------
        // set T_PreToCurr;
        ORUtils::SE3Pose T_PreToCurr(
                (float)tvec.ptr<double>(0)[0],(float)tvec.ptr<double>(0)[1],(float)tvec.ptr<double>(0)[2],
                (float)rvec.ptr<double>(0)[0],(float)rvec.ptr<double>(0)[1],(float)rvec.ptr<double>(0)[2]
                );
        T_PreToCurr.Coerce();
        // set T_CurrToPre;
        T_CurrToPre.SetInvM(T_PreToCurr.GetM());
        T_CurrToPre.Coerce();

        // print transform matrix
        Matrix4f T=T_CurrToPre.GetM();
        std::cout<<"VO(curr_to_pre) result::T"<<std::endl;
        std::cout<<T;
        // print translation length
        float length=std::pow(T.m30*T.m30+T.m31*T.m31+T.m32*T.m32,0.5);
        std::cout<<"VO(curr_to_pre) result::translation::"<<length<<std::endl;
        // print Rotation angle
        const float* params;
        params=T_CurrToPre.GetParams();
        float angle=std::pow(params[3]*params[3]+params[4]*params[4]+params[5]*params[5],0.5);
        std::cout<<"VO(curr_to_pre) result::Rotation angle(radian)::"<<angle<<std::endl;
        std::cout<<"VO(curr_to_pre) result::Rotation angle(degree)::"<<angle/3.1415926*180<<std::endl;

        return 0;
    }





    // step_3
    int change_trackingState(
                            const ORUtils::SE3Pose &T_CurrToPre,
                            ITMLib::ITMTrackingState *trackingState
                            )
    {
        // 当前帧位姿 李代数上
        ORUtils::SE3Pose pose_curr(T_CurrToPre);// 先用T_CurrTopre进行初始化
        pose_curr.MultiplyWith(trackingState->pose_d);// 再右乘旧的位姿(即前一帧的位姿）

        // 更新当前帧位姿
        trackingState->pose_d->SetFrom(  &pose_curr  );
        return 0;
    }


    //-------------------------------------------------------------------------------------core function-----------------------------------------------------------
    int VO_initialize(const ITMLib::ITMView *view,ITMLib::ITMTrackingState *trackingState)
    {
        // data
        vector< cv::KeyPoint > kps_pre;
        vector< cv::KeyPoint > kps_curr;
        vector< cv::DMatch > matches;
        ORUtils::SE3Pose T_CurrToPre;


//        // -----------------------------------------qilong:test----------------------------------------------------------------------------
//        for (int r = 0; r < view->depth->noDims.y; r++)
//        {
//            for (int c = 0; c < view->depth->noDims.x; c++) {
//                // d
//                auto d=view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU);
//                std::cout<<view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU)<<",";
//            }
//            std::cout<<std::endl;
//        }
//        std::cout<<"-----------------------------------------qilong:test5----------------------------------------------------------------------------"<<std::endl;
//        // -----------------------------------------qilong:test----------------------------------------------------------------------------


        // solve
        if(  0!=feature_matching_GMS_SIFT(view,kps_pre,kps_curr,matches) )
        {
            std::cout<<"feature_matching is fail"<<std::endl;
            return -1;
        }
        if( 0!=pnp(view,kps_pre,kps_curr,matches,T_CurrToPre) )
        {
            std::cout<<"pnp is fail"<<std::endl;
            return -1;
        }
        if( 0!=change_trackingState(T_CurrToPre,trackingState) )
        {
            std::cout<<"change_trackingState is fail"<<std::endl;
            return -1;
        }

        return 0;
    }

};




















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







		void Track(ITMTrackingState *trackingState, const ITMView *view)
		{
			static int long_count=0;
            std::cout<<"--------------------------------process:"<<long_count<<"-------------------------------------"<<endl;
//            std::cout<<"view state(noReSet)"<<view->rgb_prev->NoReSet<<std::endl;



//            // -----------------------------------------qilong:test----------------------------------------------------------------------------
//            for (int r = 0; r < view->depth->noDims.y; r++)
//            {
//                for (int c = 0; c < view->depth->noDims.x; c++) {
//                    // d
//                    auto d=view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU);
//                    std::cout<<view->depth->GetElement(r * (view->depth->noDims.x) + c, MEMORYDEVICE_CPU)<<",";
//                }
//                std::cout<<std::endl;
//            }
//            std::cout<<"-----------------------------------------qilong:test4----------------------------------------------------------------------------"<<std::endl;
//            // -----------------------------------------qilong:test----------------------------------------------------------------------------





            if(view->rgb_prev->NoReSet==false)
            {
                QiLong().VO_initialize(view,trackingState);
            }
		    tracker->TrackCamera(trackingState, view);
            long_count++;
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
//          detector = cv::FeatureDetector::create("ROB");
//          descriptor = cv::DescriptorExtractor::create("ORB");

//    int RANSAC_long(vector< cv::KeyPoint > &kp_pre,vector< cv::KeyPoint > &kp_cur,vector< cv::DMatch > &goodMatches,const cv::Mat &rgb_prev_Mat,const cv::Mat &rgb_curr_Mat)
//    {
//
//        vector<cv::DMatch> m_Matches;
//        m_Matches = goodMatches;
//        int ptCount = goodMatches.size();
//        if (ptCount < 100)
//        {
//            cout << "Don't find enough match points" << endl;
//            return 0;
//        }
//
//        //坐标转换为float类型
//        vector <cv::KeyPoint> RAN_KP1, RAN_KP2;
//        //size_t是标准C库中定义的，应为unsigned int，在64位系统中为long unsigned int,在C++中为了适应不同的平台，增加可移植性。
//        for (size_t i = 0; i < m_Matches.size(); i++)
//        {
//            RAN_KP1.push_back(kp_pre[goodMatches[i].queryIdx]);
//            RAN_KP2.push_back(kp_cur[goodMatches[i].trainIdx]);
//            //RAN_KP1是要存储img01中能与img02匹配的点
//            //goodMatches存储了这些匹配点对的img01和img02的索引值
//        }
//        //坐标变换
//        vector <cv::Point2f> p01, p02;
//        for (size_t i = 0; i < m_Matches.size(); i++)
//        {
//            p01.push_back(RAN_KP1[i].pt);
//            p02.push_back(RAN_KP2[i].pt);
//        }
//        /*vector <Point2f> img1_corners(4);
//        img1_corners[0] = Point(0,0);
//        img1_corners[1] = Point(img_1.cols,0);
//        img1_corners[2] = Point(img_1.cols, img_1.rows);
//        img1_corners[3] = Point(0, img_1.rows);
//        vector <Point2f> img2_corners(4);*/
//        ////求转换矩阵
//        //Mat m_homography;
//        //vector<uchar> m;
//        //m_homography = findHomography(p01, p02, RANSAC);//寻找匹配图像
//        //求基础矩阵 Fundamental,3*3的基础矩阵
//        vector<uchar> RansacStatus;
//        cv::Mat Fundamental = findFundamentalMat(p01, p02, RansacStatus, cv::FM_RANSAC);
//        //重新定义关键点RR_KP和RR_matches来存储新的关键点和基础矩阵，通过RansacStatus来删除误匹配点
////            vector <cv::KeyPoint> RR_KP1, RR_KP2;
//        kp_pre.clear();
//        kp_cur.clear();
////            vector <cv::DMatch> RR_matches;
//        goodMatches.clear();
//        int index = 0;
//        for (size_t i = 0; i < m_Matches.size(); i++)
//        {
//            if (RansacStatus[i] != 0)
//            {
//                kp_pre.push_back(RAN_KP1[i]);
//                kp_cur.push_back(RAN_KP2[i]);
//                m_Matches[i].queryIdx = index;
//                m_Matches[i].trainIdx = index;
//                goodMatches.push_back(m_Matches[i]);
//                index++;
//            }
//        }
//        cout << "RANSAC后匹配点数" <<goodMatches.size()<< endl;
//        cv::Mat img_RR_matches;
//        drawMatches(rgb_prev_Mat, kp_pre, rgb_curr_Mat,kp_cur, goodMatches, img_RR_matches);
//        imshow("After RANSAC",img_RR_matches);
//        cv::imwrite( "../../goodmatche_RANSAC.png", img_RR_matches);
//        //等待任意按键按下
////            cv::waitKey(0);
//
//    }




}
