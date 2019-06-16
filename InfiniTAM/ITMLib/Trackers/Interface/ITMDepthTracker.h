// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"
#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../../Objects/Tracking/ITMImageHierarchy.h"
#include "../../Objects/Tracking/ITMTemplatedHierarchyLevel.h"
#include "../../Objects/Tracking/ITMSceneHierarchyLevel.h"
#include "../../Objects/Tracking/TrackerIterationType.h"

#include "../../../ORUtils/HomkerMap.h"
#include "../../../ORUtils/SVMClassifier.h"

namespace ITMLib
{
	/** Base class for engine performing ICP based depth tracking.
	    A typical example would be the original KinectFusion
	    tracking algorithm.
	*/
	class ITMDepthTracker : public ITMTracker
	{
	private:
		const ITMLowLevelEngine *lowLevelEngine;
		ITMImageHierarchy<ITMSceneHierarchyLevel> *sceneHierarchy; // 场景金字塔
		ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> > *viewHierarchy; // 新帧金字塔

		ITMTrackingState *trackingState; const ITMView *view; // tracking状态（主要有位姿），view（新的一帧）

		int *noIterationsPerLevel; // 金字塔每个层级的迭代次数？

		float terminationThreshold; // 终止迭代的阈值？

		void PrepareForEvaluation(); // 为tracking质量评估做准备，还是为迭代过程做准备？
		void SetEvaluationParams(int levelId); // 设置评估参数，关键这个里的这个Evaluation到底指的是什么？我觉得应该是迭代过程

		void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const; // 计算△
		void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const; // 用△去更新位姿
		bool HasConverged(float *step) const; // 判断是否收敛

		void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);// 设置评估数据（迭代的基础数据）

		void UpdatePoseQuality(int noValidPoints_old, float *hessian_good, float f_old); // 更新位姿质量的几个评估参数（内点的比例，汉森矩阵，残差和）

		ORUtils::HomkerMap *map;
		ORUtils::SVMClassifier *svmClassifier;
		Vector4f mu, sigma;
	protected:
		float *distThresh;

		int levelId; // 当前tracking所处的金字塔的层级ID
		TrackerIterationType iterationType; //迭代类型

		Matrix4f scenePose; // 场景位姿
		ITMSceneHierarchyLevel *sceneHierarchyLevel;  // 场景的金字塔序列
		ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel;  // 新帧的金字塔序列

		virtual int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0; // 暂时不知道是干嘛的，但一定是设备特定层的

	public:
		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view); // 核心函数

		// 一直不知道下面几个函数是干什么用的
		bool requiresColourRendering() const { return false; }
		bool requiresDepthReliability() const { return false; }
		bool requiresPointCloudRendering() const { return true; }

		// 设置等级？金字塔的等级？ 迭代数量（粗糙版） 迭代数量（fine版）距离阈值（粗糙版)  距离阈值（fine版）
		void SetupLevels(int numIterCoarse, int numIterFine, float distThreshCoarse, float distThreshFine);

		// 构造函数
		ITMDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
			float terminationThreshold, float failureDetectorThreshold, 
			const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
		virtual ~ITMDepthTracker(void);
	};
}
