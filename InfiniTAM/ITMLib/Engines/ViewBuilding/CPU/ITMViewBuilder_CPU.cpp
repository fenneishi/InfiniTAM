// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CPU.h"

#include "../Shared/ITMViewBuilder_Shared.h"
#include "../../../../ORUtils/MetalContext.h"

using namespace ITMLib;
using namespace ORUtils;

ITMViewBuilder_CPU::ITMViewBuilder_CPU(const ITMRGBDCalib& calib):ITMViewBuilder(calib) { }
ITMViewBuilder_CPU::~ITMViewBuilder_CPU(void) { }

//rgbImage彩色图
//rawDepthImage未加工的深度图
//useBilateralFilter 使用useBilateralFilter滤波
//modelSensorNoise是否有模型传感器噪音
//storePreviousImage是否存储前一帧
void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage)
{
    // 建立view对象(并及时填充空的depthNormal，depthUncertainty，因为view的其他成员都会在构造时候自动建立
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, false);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormal = new ITMFloat4Image(rawDepthImage->noDims, true, false);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(rawDepthImage->noDims, true, false);
		}
	}

	// 临时的view
	ITMView *view = *view_ptr;

	// 如果要存储前一帧，就给这个临时的view添加上前一帧
	if (storePreviousImage)
	{
		if (!view->rgb_prev) view->rgb_prev = new ITMUChar4Image(rgbImage->noDims, true, false);
		else view->rgb_prev->SetFrom(view->rgb, MemoryBlock<Vector4u>::CPU_TO_CPU);
	}


    //	用rbgImage去建立view->rbg对象
	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
	//  用rawDepthImage(未加工的深度图像)去建立shortImage
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CPU);


//	注意，现在有三股势力:
//	1是view_ptr，是传参进来的核心要建立的view.
//	2是本地的shortImage，floatImage，用于存放未加工的rawDepthImage和某种加工过后的DepthImage，是过渡性角色。
//	3是新建的局部view，指向和形参view_ptr一致，所以对*view修改，就是对**view_ptr进行修改。


	switch (view->calib.disparityCalib.GetType()) //
	{
	case ITMDisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib.intrinsics_d), view->calib.disparityCalib.GetParams());
		break;
	case ITMDisparityCalib::TRAFO_AFFINE:
		this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib.disparityCalib.GetParams());
		break;
	default:
		break; //如果不是视差图（双目视差）就不用接受上述洗礼
	}

	if (useBilateralFilter)
	{
		//5 steps of bilateral filtering
		this->DepthFiltering(this->floatImage, view->depth);
		this->DepthFiltering(view->depth, this->floatImage);
		this->DepthFiltering(this->floatImage, view->depth);
		this->DepthFiltering(view->depth, this->floatImage);
		this->DepthFiltering(this->floatImage, view->depth);
		view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CPU_TO_CPU);//经受bilateral滤波洗礼之后，就可以做view->depth的SetFrom了
	}

    //	感觉这里有个2个bug,1是如果useBilateralFilter==false,那么就没有set"view->depth"了，2是难道modelSensorNoise==false,就不设置法向量了吗？

    //	如果要考虑深度传感器噪音的话，就设置一下view->depthNormal,view->depthUncertainty
	if (modelSensorNoise)
	{
		this->ComputeNormalAndWeights(view->depthNormal, view->depthUncertainty, view->depth, view->calib.intrinsics_d.projectionParamsSimple.all);
	}
}

//这个是有IMU的情况
void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise, bool storePreviousImage)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, false);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(depthImage->noDims, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(depthImage->noDims, true, false);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormal = new ITMFloat4Image(depthImage->noDims, true, false);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(depthImage->noDims, true, false);
		}
	}

	ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage, useBilateralFilter, modelSensorNoise, storePreviousImage);
}


//视差图-->深度图
void ITMViewBuilder_CPU::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	Vector2f disparityCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

//视差图-->深度图
void ITMViewBuilder_CPU::ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const Vector2f depthCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}


//深度图的BilateralFilter
void ITMViewBuilder_CPU::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgSize = image_in->noDims;

	image_out->Clear();

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
		filterDepth(imout, imin, x, y, imgSize);
}


void ITMViewBuilder_CPU::ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic)
{
	Vector2i imgDims = depth_in->noDims;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
		computeNormalAndWeight(depthData_in, normalData_out, sigmaZData_out, x, y, imgDims, intrinsic);
}

