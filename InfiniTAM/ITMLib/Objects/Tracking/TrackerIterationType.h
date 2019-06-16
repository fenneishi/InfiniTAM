// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

namespace ITMLib
{
	/// The tracker iteration type used to define the tracking iteration regime
	// 用于定于track的迭代机制，猜测含义是：只迭代旋转矩阵，还是只有TRANSLATION，还是both,我觉得一般都是both
	enum TrackerIterationType
	{
		TRACKER_ITERATION_ROTATION,
		TRACKER_ITERATION_TRANSLATION,
		TRACKER_ITERATION_BOTH,
		TRACKER_ITERATION_NONE
	};
}
