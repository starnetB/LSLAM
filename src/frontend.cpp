#include <opencv2/opencv.hpp>

#include <algorithm.h>
//#include <backend.h>
#include <config.h>
#include <feature.h>
#include <frontend.h>
//#include <g2o_types.h>
#include <map.h>
//#include <viewer.h>

namespace lslam{
Frontend::Frontend(){
    gftt_=cv::GFTTDetector::create(Config::Get<int>("num_features"),0.01,20);
    num_features_init_=Config::Get<int>("num_features_init");
    num_features_=Config::Get<int>("num_features");
}

bool Frontend::AddFrame(lslam::Frame::Ptr frame){
    current_frame_=frame;
    //判断状态量根据状态量完成不同的行为规范
    switch (status_)
    {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        //正常状态下的两种行为
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }

    last_frame_=current_frame_;
    return true;
}

bool Frontend::Track(){
    if(last_frame_){
        // relative_motion_.   point current->last
        // last_frame->pose    c->w
        // relative_motion_.   current->w
        current_frame_->SetPose(relative_motion_*last_frame_->Pose());
    }
    int num_track_last=TrackLastFrame();  //使用上一帧图，来更新这一帧图的 features_left Vector

//    tracking_inliers_=Es
}

int Frontend::TrackLastFrame(){
    // use LK flow to estimate points in the right image
    // 当前帧和上一帧的关键点
    std::vector<cv::Point2f> kps_last,kps_current;
    // 把上一帧的左图关键点给拿出来
    for(auto &kp: last_frame_->features_left_){
        // 找到对应的地图点
        if(kp->map_point_.lock()){
            //use project point
            // 如果有找到对应的地图点，那就使用重投影的方法找到像素点
            auto mp=kp->map_point_.lock();
            auto px=camera_left_->world2pixel(mp->pos_,current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0],px[1]));
        }else{
            //如果没有找到地图点，暂时假设两者的位置是一样的，
            //后面会有光流法来获取对应的位置
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }
    std::vector<uchar> status;
    Mat error;
    /*calcOpticalFlowPyrLK
    * @ status 介绍有没有错误的状态
    * @ error 输出错误的矢量; 向量的每个元素都设置为相应特征的错误，错误度量的类型可以在flags参数中设置; 如果未找到流，则未定义错误（使用status参数查找此类情况）
    * @ cv::Size(11,11) 金字塔光流度量核大小
    * @ 3 金字塔的层次，这里是指三层金字塔
    * @ cv::TermCriteria()
    *   @ cv::TermCriteria::COUNT + cv::TermCriteria::EPS 迭代终止的条件，cout+eps需要同时考虑
    *   @ 30 迭代的最大次数
    *   @ 0.01 迭代的终止的阈值
    * @ cv::OPTFLOW_USE_INITIAL_FLOW  初始化的方式这个不太明白？
    * @ minEigThreshold ：算法计算光流方程的2x2正常矩阵的最小特征值，除以窗口中的像素数;如果此值小于minEigThreshold，则过滤掉相应的功能并且不处理其流程，因此它允许删除坏点并获得性能提升。
    * */
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_,current_frame_->left_img_,kps_last,
        kps_current,status,error,cv::Size(11,11),3,
        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
                         30,0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);
    
    int num_good_pts=0;

    for(size_t i=0;i<status.size();++i){
        if(status[i]){
            cv::KeyPoint kp(kps_current[i],7);
            Feature::Ptr feature(new Feature(current_frame_,kp));
            feature->map_point_=last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}

int Frontend::EstimateCurrentPose(){
    //setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;


}
}