#pragma once
#ifndef MAP_H
#define MAP_H

#include<common_include.h>
#include<frame.h>
#include<mappoint.h>

namespace  lslam
{
/**
 * @brief 地图
 * 和地图的交互：前端调用InsertKeyframe和InsertMapPoint插入新帧和地图点，后端维护地图的结构，判定outlier/剔除等等
*/
class Map{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map(){}

    ///增加一个关键帧
    void InsertKeyFrame(Frame::Ptr frame);

    ///增加一个地图顶点
    void InsertMapPoint(MapPoint::Ptr map_point);

    ///所有地图点
    LandmarksType GerAllMapPoints(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }

    /// 获取所有关键帧
    KeyframesType GetAllFrames(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    ///获取激活地图点
    LandmarksType GetActiveMapPoints(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    ///获取激活关键帧
    KeyframesType GetActiveKeyFrames(){
        std::unique_lock<std::mutex> lck(data_mutex_);
        return activate_keyframes_;
    }

    ///清理map中观测数量为零的点
    void CleanMap();

private:
    //将旧的关键帧置为不活跃状态
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;                    // all landmarks
    LandmarksType active_landmarks_;            // active landmarks
    KeyframesType keyframes_;                   // all key-frames
    KeyframesType activate_keyframes_;            // active all key-frames

    Frame::Ptr current_frame=nullptr;

    //settings 
    int num_activate_keyframes_=7;            //激活的关键帧

    //namespace lslam
};
} // namespace  lslam
#endif
// Map 中包含以下内容
    // 所有路标
    // 激活路标
    // 所有frame
    // 激活frame
    // 当前帧

//这里的激活就是删除旧的帧，插入新的帧，保持7个激活的帧
//这里激活点也是，最初的激活点，是有初始图来选择