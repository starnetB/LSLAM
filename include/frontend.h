#pragma once

#ifndef FRONTEND_H
#define FORNTEND_H


#include <opencv2/features2d.hpp>
#include <common_include.h>
#include <frame.h>
#include <map.h>

namespace lslam{
class Backend;
class Viewer;

enum class FrontendStatus{ INITING,TRACKING_GOOD,TRACKING_BAD,LOST};

/**
 * 前端
 * 估计当前帧Pose，在满足关键帧条件时向地图加入关键帧并触发优化
 */
class Frontend{
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   typedef std::shared_ptr<Frontend> Ptr;

   Frontend();

   ///外部接口，添加一个帧并计算其定位结果
   bool AddFrame(Frame::Ptr frame);

   ///Set 函数
   void SetMap(Map::Ptr map){map_=map;}
   void SetBackend(std::shared_ptr<Backend> backend){backend_=backend;}
   void SetViewer(std::shared_ptr<Viewer> viewer){viewer_=viewer;}

   FrontendStatus GetStatus() const {return status_;}

   void SetCameras(Camera::Ptr left,Camera::Ptr right){
       camera_left_=left;
       camera_right_=right;
   }

private:
    /**
     * Track in normal mode
     * @return true if success
     */
    bool Track();

    /**
     * Reset when lost
     * @return true if success
     */
    bool Reset();

    /**
     * Track with last frame
     * @return num of tracked points
     */
    int TrackLastFrame();

    /**
     * estimate current frame's pose
     * @return num of inliers
     */
    int EstimateCurrentPose();

    /**
     * set current frame as a keyframe and insert it into backend
     * @return true if success
     */
    bool InsertKeyframe();

    /**
     * Try init the frontend with stereo images saved in current_frame_
     * @return true if success
     */
    bool StereoInit();

    /**
     * Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     * @return
     */
    int DetectFeatures();

    /**
     * Find the corresponding features in right image of current_frame_
     * @return num of features found
     */
    int FindFeaturesInRight();

    /**
     * Build the initial map with single image
     * @return true if succeed
     */
    bool BuildInitMap();

    /**
     * Triangulate the 2D points in current frame
     * @return num of triangulated points
     */
    int TriangulateNewPoints();

    /**
     * Set the features in keyframe as new observation of the map points
     */
    void SetObservationsForKeyFrame();

    // data
    FrontendStatus status_=FrontendStatus::INITING;

    Frame::Ptr current_frame_=nullptr;   //当前帧
    Frame::Ptr last_frame=nullptr;       //上一帧
    Camera::Ptr camera_left_=nullptr;     //右侧相机
    Camera::Ptr camera_right_=nullptr;    //左侧相机

    Map::Ptr map_=nullptr;
    std::shared_ptr<Backend> backend_=nullptr;
    std::shared_ptr<Viewer> viewer_=nullptr;

    SE3 relative_motion_; //当前帧与上一帧的相对运动，用于估计当前帧pose初值

    int tracking_inliers_=0;  //inliers,used for testint new keyframes

    //params
    int num_features_=200;
    int num_features_init_ =100;
    int num_features_tracking_=50;
    int num_features_tracking_bad_=20;
    int num_features_needed_for_keyframe_=80;


    //utilities
    cv::Ptr<cv::GFTTDetector> gftt_; //feature detector in opencv
};
}    //namespace lsalm
#endif    //FRONTEND_H

// 1. 前端本身有初始化，正常追踪，追踪丢失三种状态
// 2. 在初始化状态中，根据左右目之间的光流匹配，寻找可以三角化的地图点，成功时建立初始地图
// 3. 追踪阶段中，前端计算上一帧的特征点到当前帧的光流，根据光流结果计算图像位置。该计算只使用左目地图
// 4. 如果追踪到的点较少，就办定当前帧为关键帧，对于关键帧，做以下几件事：
     // * 提取新的特征点
     // * 找到这些点在右图的对应点，用三角化建立新的路标点
     // * 将新的关键帧和路标点加入地图，并出发一次后端优化
// 5. 如果追踪丢失，就重置前端系统，重新初始化