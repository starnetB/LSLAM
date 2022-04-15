#ifndef SLAM_VIEW_H
#define SLAM_VIEW_H

#include <thread>
#include <pangolin/pangolin.h>
#include <common_include.h>
#include <frame.h>
#include <map.h>

namespace lslam{

/**
 * 可视化
 */
class Viewer{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map) {map_=map;}

    void Close();

    //增加一个的当前帧
    void AddCurrentFrame(Frame::Ptr current_frame);

    //更新
    void UpdateMap();

private:
    void ThreadLoop();

    void DrawFrame(Frame::Ptr frame,const float* color);

    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    /// plot the features in current frame into an image
    /// 在一张图上画出特征点
    cv::Mat PlotFrameImage();

    Frame::Ptr current_frame_=nullptr;
    Map::Ptr map_=nullptr;

    std::thread viewer_thread_;
    bool viewer_running_=true;

    std::unordered_map<unsigned long,Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long,MapPoint::Ptr> active_landmarks_;

    bool map_updated_=false;
    std::mutex viewer_data_mutex_;
};
}  //namespace myslam
#endif