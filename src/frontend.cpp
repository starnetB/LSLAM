#include <opencv2/opencv.hpp>

#include <algorithm.h>
#include <backend.h>
#include <config.h>
#include <feature.h>
#include <frontend.h>
#include <g2o_types.h>
#include <map.h>
#include <viewer.h>

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
            //完成双目初始化，构建map
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
        // last_frame->pose    last->world
        // result   current->w
        // 下面的歧义在这里解决啦，每次都是用相对坐标更新就可以啦
        current_frame_->SetPose(relative_motion_*last_frame_->Pose());
    }
    int num_track_last=TrackLastFrame();  //使用上一帧图，来更新这一帧图的 features_left Vector,也就是找到对应的特征点
    //获取当前帧的位姿，current_pose->pose_,但是优化有的当前帧位置是左图指向世界坐标系 
    //返回内点的数量，将外点的mappint 为null
    
    tracking_inliers_=EstimateCurrentPose();
    
    if(tracking_inliers_>num_features_tracking_){
        //tracking good
        status_=FrontendStatus::TRACKING_GOOD;
    }else if(tracking_inliers_>num_features_tracking_bad_){
        //tracking bad
        status_=FrontendStatus::TRACKING_BAD;
    }else{
        //lost
        status_=FrontendStatus::LOST;
    }
    InsertKeyframe();
    relative_motion_=current_frame_->Pose()*last_frame_->Pose().inverse();
    if(viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

bool Frontend::InsertKeyframe(){
    
    if(tracking_inliers_>=num_features_needed_for_keyframe_){
        //still have enough features,don't insert keyframe
        LOG(INFO)<<"setframe"<<std::endl;
        return false;
    }
    //current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame();  //为左图features->mp_point 增加关键点
    DetectFeatures();  //detect new features

    //track in right image
    // 补充特征点和Map点
    FindFeaturesInRight();
    //Triangulate map points
    TriangulateNewPoints();
    
    //后端，对Map的关键帧以及mappoint进行一次更新
    backend_->UpdateMap(); 
    //在视图上更新一次地图点与current_frame
    if(viewer_) viewer_->UpdateMap();
    return true;
}
int Frontend::TriangulateNewPoints(){
    
    std::vector<SE3> poses{camera_left_->pose(),camera_right_->pose()};
    SE3 current_pose_Twc=current_frame_->Pose().inverse();
    int cnt_triangulated_pts=0;
    for(size_t i=0;i<current_frame_->features_left_.size();++i){
        // .expired判断指针是否过期，想对于lock() 一般不需要取出对象，更加节约时间
        if(current_frame_->features_left_[i]->map_point_.expired() && 
           current_frame_->features_right_[i] !=nullptr){
            //左图的特征点未关联地图点  
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld=Vec3::Zero();
        
            if(triangulation(poses,points,pworld) && pworld[2]>0){
                auto new_map_point=MapPoint::CreateNewMappoint();
                pworld=current_pose_Twc*pworld;
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                
                current_frame_->features_left_[i]->map_point_=new_map_point;
                current_frame_->features_right_[i]->map_point_=new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;
            } 
        }
    }
    LOG(INFO) <<"new landmarks:  "<<cnt_triangulated_pts;
    return cnt_triangulated_pts;
}



void Frontend::SetObservationsForKeyFrame(){
    
    for(auto &feat:current_frame_->features_left_){
        auto mp=feat->map_point_.lock();
        if(mp) mp->AddObservation(feat);
    }
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
            // * 下面有个地方出现了歧义
            auto px=camera_left_->world2pixel(mp->pos_,current_frame_->Pose());
            //这里只是初步更新，结果还是看光流
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
//更新current_frame 的Pose_,
//返回有效特征点的个数
int Frontend::EstimateCurrentPose(){
    
    //setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver=new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimzier;
    optimzier.setAlgorithm(solver);
    
    //vertex
    VertexPose *vertex_pose=new VertexPose();  //camera_vertex_pose 左图到世界坐标系
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimzier.addVertex(vertex_pose);

    //K
    Mat33 K=camera_left_->K();

    //edges
    int index=1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    
    LOG(INFO)<<"featurese_size ::  "<<current_frame_->features_left_.size();
    for(size_t i=0;i<current_frame_->features_left_.size();++i){
        auto mp=current_frame_->features_left_[i]->map_point_.lock();
        if(mp){
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge=new EdgeProjectionPoseOnly(mp->pos_,K);
            
            edge->setId(index);
            edge->setVertex(0,vertex_pose);
            // 地图点的pos_，直接带入其中，也就是左图直接指向世界坐标，current_frame就是这样，那么这里就和上面出现的歧义，后面要好好对应一下
            edge->setMeasurement(
                toVec2(current_frame_->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimzier.addEdge(edge);
            index++;
        }
    }
    /*e^T Q^-1 e*/ //信息举证表示我们对不同的误差重视程度不一样
    //estimate the Pose the determine the outliers
    //位姿是否应该是排除点
    const double chi2_th=5.991;
    int cnt_outlier=0;
    for(int iteration=0;iteration<4;++iteration){
        vertex_pose->setEstimate(current_frame_->Pose());
        optimzier.initializeOptimization();
        optimzier.optimize(10);
        cnt_outlier=0;

        //count the outliers
        //每次优化后剔除优化错误的点，误差过大的异常点
        for(size_t i=0;i<edges.size();++i){
            auto e=edges[i];
            //如果上一次优化，有些点被定义为异常点，那么我们要在计算一边
            if(features[i]->is_outlier_){
                e->computeError();
            }
            //看看阈值是否超过我们的设定值
            if(e->chi2()>chi2_th){
                //如果超过，我们就将其设定为异常点
                features[i]->is_outlier_=true;
                //下次优化忽略这个变
                e->setLevel(1);
                //每次优化点加一
                cnt_outlier++;
            }else{
                //如果不超过相应点，恢复优化
                features[i]->is_outlier_=false;
                e->setLevel(0);
            } 

            if(iteration==2){
                //迭代两次后我们不在需要鲁棒核了，
                //因为距离住足够接近了
                e->setRobustKernel(nullptr);
            }
        }
    }
    LOG(INFO)<<"outlier/Inlier in pose estimating:"<<cnt_outlier<<"/"
             <<features.size()-cnt_outlier;
    
    //pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) <<"Current Pose= \n"<<current_frame_->Pose().matrix();

    for(auto &feat:features){
        if(feat->is_outlier_){
            //释放对托管对象的引用 
            //也就是将std::weak_ptr<MapPoint> 变成null
            feat->map_point_.reset();
            feat->is_outlier_=false;  //在优化的时候我们可能不放在里面，但优化完后还是会放回里面的
        }
    }
    return features.size()-cnt_outlier;
}

//双目相机初始化：
bool Frontend::StereoInit(){
    int num_feautes_left=DetectFeatures();
    int num_coor_features=FindFeaturesInRight();
    //如果左右图中的对应点小于num_features_init_点的数量
    //返回false
    if(num_coor_features< num_features_init_){
        return false;
    }
    bool build_map_success=BuildInitMap();
    //如果构建成功
    if(build_map_success){
        //实现状态切换
        
        status_=FrontendStatus::TRACKING_GOOD;
        
        if(viewer_){
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;

    }
    return false;
}

int Frontend::DetectFeatures(){
    //和左图的那一帧一样，255作为i掩码流值

    cv::Mat mask(current_frame_->left_img_.size(),CV_8UC1,255);
    for(auto &feat:current_frame_->features_left_){
        //在特征点附近添加0,的填充
        cv::rectangle(mask,
                    feat->position_.pt-cv::Point2f(10,10),
                    feat->position_.pt+cv::Point2f(10,10),0,CV_FILLED);

    }

    std::vector<cv::KeyPoint> keypoints;
    //记住已经探测出关键角点的区域，我们就不需要角点了
    gftt_->detect(current_frame_->left_img_,keypoints,mask);
    int cnt_detected=0;
    for(auto &kp:keypoints){
        //添加角点
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_,kp)));
        cnt_detected++;
    }
    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

int Frontend::FindFeaturesInRight(){
    //use LK flow to estimate points in the rgiht image
    std::vector<cv::Point2f> kps_left,kps_right;
    for(auto &kp:current_frame_->features_left_){
        kps_left.push_back(kp->position_.pt);
        auto mp=kp->map_point_.lock();
        if(mp){
            auto px=camera_right_->world2pixel(mp->pos_,current_frame_->Pose());
            kps_right.push_back(cv::Point2f(px[0],px[1]));
        }else{
            //use same pixel in left image
            kps_right.push_back(kp->position_.pt);
        }

    }
    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_,current_frame_->right_img_,kps_left,
        kps_right,status,error,cv::Size(11,11),3,
        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,30,0.01),cv::OPTFLOW_USE_INITIAL_FLOW
    );
    int num_good_pts=0;
    for(size_t i=0;i<status.size();++i){
        if(status[i]){
            cv::KeyPoint kp(kps_right[i],7);
            Feature::Ptr feat(new Feature(current_frame_,kp));
            feat->is_on_left_image_=false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        }else{
            current_frame_->features_right_.push_back(nullptr);
        }
    }
    LOG(INFO)<<"Find " <<num_good_pts<<" in the right image";
    return num_good_pts;
}

bool Frontend::BuildInitMap(){
    std::vector<SE3> poses{camera_left_->pose(),camera_right_->pose()};
    size_t cnt_init_landmarks=0;
    for(size_t i=0;i<current_frame_->features_left_.size();++i){
        if(current_frame_->features_right_[i]==nullptr) continue;
        //create map point from triangulation
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                current_frame_->features_right_[i]->position_.pt.y))
        };

        Vec3 pworld=Vec3::Zero();

        //根据左右图的位姿，获取对应的世界坐标系下的pworld，双目相机的世界坐标系下的坐标点
        //curent_frame 通过重投影误差算出来的pose 应该是左图只向直接坐标系
        if(triangulation(poses,points,pworld)&&pworld[2]>0){
            auto new_map_point=MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            //增加两个观察点
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            current_frame_->features_left_[i]->map_point_=new_map_point;
            current_frame_->features_right_[i]->map_point_=new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);
        }
    }
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();
    LOG(INFO) << "Initial map created with " <<cnt_init_landmarks
              << " map points";
    
    return true;
}

bool Frontend::Reset(){
    LOG(INFO) <<"Reset is not implemented.  ";
    return true;
}
}