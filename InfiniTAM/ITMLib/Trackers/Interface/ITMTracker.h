// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Objects/Tracking/ITMTrackingState.h"
#include "../../Objects/Views/ITMView.h"

namespace ITMLib
{
	/** \brief
	    Basic interface to any sort of trackers that will align an
	    incoming view with an existing scene.
	    基础接口：用来对齐新帧和已经场景
	*/
	class ITMTracker
	{
	public:
		/** Gets whether the tracker can keep tracking or not.
		    Can be used to signal e.g. the end of a sequence
		    of file-based poses, or the failure of an IMU.
		    获取tracker是否能够继续追踪
		    可以用来给"结束帧，位姿计算失败，因为IMU的失败"做signal。
		*/
		virtual bool CanKeepTracking() const { return true; }

		/** Localize a View in the given scene. The result is
		    currently stored as an attribute in trackingState.
		    定位给定场景的位置。返回值存储在trackingState的属性中
		*/
		virtual void TrackCamera(ITMTrackingState *trackingState, const ITMView *view) = 0;

		/** Updates the initial pose of the depth camera in the scene.
		    This can be used to make the scene up vector correspond
		    to the real world's up direction.
		    更新场景中深度相机的初始位姿。
		    可以用来一致化场景的up vector与真实世界的up direction.
		*/
		virtual void UpdateInitialPose(ITMTrackingState *trackingState) {}

		virtual bool requiresColourRendering() const = 0;
		virtual bool requiresDepthReliability() const = 0;
		virtual bool requiresPointCloudRendering() const = 0;

		virtual ~ITMTracker(void) {}
	};
}







