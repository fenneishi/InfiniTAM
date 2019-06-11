// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Camera/ITMCalibIO.h"
#include "../../Utils/ITMImageTypes.h"

namespace ITMLib
{
	/** \brief
	    Represents a single "view", i.e. RGB and depth images along
	    with all intrinsic and relative calibration information
	    代表单个"观察"，即"RGB+depth+相机内参+深度图和彩色图对齐内参"组成一个"view"
	    我感觉就是整个当前帧了
	*/
	class ITMView
	{
	public:
		/// Intrinsic calibration information for the view. 内参
		const ITMRGBDCalib calib;

		/// RGB colour image for the current frame.  彩色图
		ITMUChar4Image *rgb; 

		/// RGB colour image for the previous frame. 前一帧的彩色图
		ITMUChar4Image *rgb_prev; 

		/// Float valued depth image, if available according to @ref inputImageType.浮点类型的深度图，根据inputImageType来确定是否可用
		ITMFloatImage *depth;

		/// surface normal of depth image 深度图的表面法向量图
		// allocated when needed  需要时再分配
		ITMFloat4Image *depthNormal;

		/// uncertainty (std) in each pixel of depth value based on sensor noise model 每个像素的深度值不确定度（根据相机的noise model确定）
		/// allocated when needed 需要时再分配
		ITMFloatImage *depthUncertainty;

		// confidence based on distance from center  depth置信度（根据距离光心得距离）
		ITMFloatImage *depthConfidence;

		ITMView(const ITMRGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
		: calib(calibration)
		{
			this->rgb = new ITMUChar4Image(imgSize_rgb, true, useGPU);
			this->rgb_prev = NULL; // 这里默认是null，要注意。
			this->depth = new ITMFloatImage(imgSize_d, true, useGPU);
			this->depthNormal = NULL;
			this->depthUncertainty = NULL;
			this->depthConfidence = new ITMFloatImage(imgSize_d, true, useGPU); // 按理说，这里不应该是距离越远，置信度越低吗？怎么直接用的深度图
		}

		virtual ~ITMView(void)
		{
			delete rgb;
			delete rgb_prev;

			delete depth;
			delete depthConfidence;

			delete depthNormal;
			delete depthUncertainty;
		}

		// Suppress the default copy constructor and assignment operator
		ITMView(const ITMView&);
		ITMView& operator=(const ITMView&);
	};
}
