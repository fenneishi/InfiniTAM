// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/Misc/ITMIMUMeasurement.h"
#include "../Trackers/Interface/ITMTracker.h"
#include "../Utils/ITMLibSettings.h"

/** \mainpage
    This is the API reference documentation for InfiniTAM. For a general
    overview additional documentation can be found in the included Technical
    Report.

    For use of ITMLib in your own project, the class
    @ref ITMLib::Engine::ITMMainEngine should be the main interface and entry
    point to the library.
*/

namespace ITMLib
{
	/** \brief
	    Main engine, that instantiates all the other engines and
	    provides a simplified interface to them.

	    This class is the main entry point to the ITMLib library
	    and basically performs the whole KinectFusion algorithm.
	    主入口并且基本呈现了整个KinectFusion算法流程
	    It stores the latest image internally, as well as the 3D
	    world model and additionally it keeps track of the camera
	    pose.
	    内部存储：最新一帧，世界3D模型，相机的实时位姿

	    The intended use is as follows:目标用法
	    -# Create an ITMMainEngine specifying the internal settings,
	       camera parameters and image sizes
	       构造函数：在internal settings(相机内参，图像大小)指导下创建一个ITMMainEngine对象
	    -# Get the pointer to the internally stored images with
	       @ref GetView() and write new image information to that
	       memory
	       GetView()：获得图像的指针，有了这个指针 ，你可以把新的图像信息更新到这个指针位置，当然也可以获取图像信息。
	    -# Call the method @ref ProcessFrame() to track the camera
	       and integrate the new information into the world model
	       ProcessFrame()：track+将新的一帧信息融入到世界模型中
	    -# Optionally access the rendered reconstruction or another
	       image for visualisation using @ref GetImage()
	       GetImage()：世界模型中获取"渲染过得重结果建"或"其他可供可视化使用的image"
	    -# Iterate the above three steps for each image in the
	       sequence
	       对于序列中的每一张图片重复上述三部，形成迭代。

	    To access the internal information, look at the member
	    variables @ref trackingState and @ref scene.
	*/
	class ITMMainEngine
	{
	public:
		enum GetImageType
		{
			InfiniTAM_IMAGE_ORIGINAL_RGB,
			InfiniTAM_IMAGE_ORIGINAL_DEPTH,
			InfiniTAM_IMAGE_SCENERAYCAST,
			InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,
			InfiniTAM_IMAGE_COLOUR_FROM_NORMAL,
			InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE,
			InfiniTAM_IMAGE_FREECAMERA_SHADED,
			InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME,
			InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL,
			InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE,
			InfiniTAM_IMAGE_UNKNOWN
		};

		/// Gives access to the current input frame
		virtual ITMView* GetView(void) = 0;

		/// Gives access to the current camera pose and additional tracking information
		virtual ITMTrackingState* GetTrackingState(void) = 0;

		/// Process a frame with rgb and depth images and optionally a corresponding imu measurement
        virtual ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement = NULL) = 0;

		/// Get a result image as output
		virtual Vector2i GetImageSize(void) const = 0;

		virtual void GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose = NULL, ITMIntrinsics *intrinsics = NULL) = 0;

		/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
		virtual void SaveSceneToMesh(const char *fileName) { };

		/// save and load the full scene and relocaliser (if any) to/from file
		virtual void SaveToFile() { };
		virtual void LoadFromFile() { };

		virtual ~ITMMainEngine() {}
	};
}
