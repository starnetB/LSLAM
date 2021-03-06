#include "map.h"
#include "feature.h"

namespace lslam{
void Map::InsertKeyFrame(Frame::Ptr frame){
    current_frame_=frame;
    // <unsigned long,Frame::Ptr> 
    // usigned long id
    // Frame::Ptr frame
    // if no frame in keyframes_,insert frame
    if(activate_keyframes_.find(frame->keyframe_id_) == activate_keyframes_.end()){
    //if(keyframes_.find(frame->keyframe_id_) == keyframes_.end()){
    //    keyframes_.insert(make_pair(frame->keyframe_id_,frame));
        activate_keyframes_.insert(make_pair(frame->keyframe_id_,frame));
    }else{
        //keyframes_[frame->keyframe_id_]=frame;
        activate_keyframes_[frame->keyframe_id_]=frame;
    }

    if(static_cast<int>(activate_keyframes_.size())> num_activate_keyframes_){
        //将激活的旧的关键帧删除
        RemoveOldKeyframe();
    }
}
void Map::InsertMapPoint(MapPoint::Ptr map_point){
    if(active_landmarks_.find(map_point->id_)==active_landmarks_.end()){
    //if(landmarks_.find(map_point->id_)==landmarks_.end()){
        //非激活点是可以不要的，因为根本没有用到
        //landmarks_.insert(make_pair(map_point->id_,map_point));
        active_landmarks_.insert(make_pair(map_point->id_,map_point));
    }else{
        //landmarks_[map_point->id_]=map_point;
        active_landmarks_[map_point->id_]=map_point;
    }
}

void Map::RemoveOldKeyframe(){
    if(current_frame_==nullptr) return;
    //寻找与当前帧最近与最远的两个关键帧
    double max_dis=0,min_dis=9999;
    double max_kf_id=0,min_kf_id=0;
    //当前帧对于基础坐标系的位置，想对于左图？，还是stereo 
    //T world point to camera , T  should be left image?
    auto Twc=current_frame_->Pose().inverse();
    for (auto & kf:activate_keyframes_){
        if(kf.second==current_frame_) continue;
        auto dis=(kf.second->Pose()* Twc).log().norm();
        if(dis>max_dis){
            max_dis=dis;
            max_kf_id=kf.first;
        }
        if(dis<min_dis){
            min_dis=dis;
            min_kf_id=kf.first;
        }
    }

    const double min_dis_th=0.2;  //最近阈值
    Frame::Ptr frame_to_remove=nullptr;
    if(min_dis<min_dis_th){
        //如果存在很近的帧，优先删掉最近的
        //frame_to_remove=keyframes_.at(min_kf_id);
        frame_to_remove=activate_keyframes_.at(min_kf_id);
    }else{
        //删掉最远的
        //frame_to_remove=keyframes_.at(max_kf_id);
        frame_to_remove=activate_keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
    // remove keyframe and landmark observation
    activate_keyframes_.erase(frame_to_remove->keyframe_id_);
    for(auto feat:frame_to_remove->features_left_){
        auto mp=feat->map_point_.lock();
        if(mp){
            mp->RemoveObservation(feat);
        }
    }
    for(auto feat:frame_to_remove->features_right_){
        if(feat==nullptr) continue;
        auto mp=feat->map_point_.lock();
        if(mp){
            mp->RemoveObservation(feat);
        }
    }

    CleanMap();
}

//清楚 那些没有被观察到的点
void Map::CleanMap(){
    int cnt_landmark_removed=0;
    for(auto iter=active_landmarks_.begin();
        iter!=active_landmarks_.end();){
        if(iter->second->observed_times_==0){
            iter=active_landmarks_.erase(iter);
            cnt_landmark_removed++;
        }else{
            ++iter;
        }
    }
}
}  //namespace lslam