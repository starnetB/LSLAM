#include "backend.h"
#include "algorithm.h"
#include "feature.h"
#include "g2o_types.h"
#include "map.h"
#include "mappoint.h"

namespace lslam{
Backend::Backend(){
    backend_runing_.store(true);
    //启动优化线程
    backend_thread_=std::thread(std::bind(&Backend::BackendLoop,this));
}

void Backend::UpdateMap()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update.notify_one();  //进行一次线程运行
}

void Backend::Stop(){
    backend_runing_.store(false);
    map_update.notify_one();
    backend_thread_.join();  //其实就是join()方法将挂起调用线程的执行,直到被调用的对象完成它的执行。”
}
void Backend::BackendLoop(){
    //通过std::atomic来运行程序
    while(backend_runing_.load()){
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update.wait(lock);

        /// 后端仅优化激活的Frames和Landmarks
        Map::KeyframesType active_kfs=map_->GetActiveKeyFrames();
        Map::LandmarksType active_landmarks=map_->GetActiveMapPoints();
        Optimize(active_kfs,active_landmarks); //优化map中激活帧与点或地图点

    }
}

void Backend::Optimize(Map::KeyframesType &keyframes,Map::LandmarksType &landmarks){
    //setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver=new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);  

    //pose 顶点 ,使用Keyframe id
    std::map<unsigned long,VertexPose *> vertices;
    unsigned long max_kf_id=0;
    for(auto &keyframe:keyframes){
        auto kf=keyframe.second;
        VertexPose *vertex_pose=new VertexPose(); // camera vertex_pose
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose());
        optimizer.addVertex(vertex_pose);
        if (kf->keyframe_id_>max_kf_id){
            max_kf_id=kf->keyframe_id_;
        }
        vertices.insert({kf->keyframe_id_,vertex_pose});
    }

    //路标顶点，使用路标id索引  
    std::map<unsigned long,VertexXYZ *> vertices_lanmarks;

    //K和左右外参数
    Mat33 K=cam_left_->K();
    SE3 left_ext=cam_left_->pose();
    SE3 right_ext=cam_right_->pose();

    //edges
    int index=1;
    double chi2_th=5.991;  //robut_kernel 鲁棒核阈值
    std::map<EdgeProjection *,Feature::Ptr> edges_and_features;

    for(auto &landmark:landmarks){
        if(landmark.second->is_outlier_) continue;
        unsigned long landmark_id=landmark.second->id_;
        auto observations=landmark.second->GetObs();
        for(auto &obs:observations){
            if(obs.lock()==nullptr) continue;;
            auto feat=obs.lock();
            if(feat->is_outlier_||feat->frame_.lock()==nullptr) continue;

            auto frame=feat->frame_.lock();
            EdgeProjection *edge=nullptr;
            if(feat->is_on_left_image_){
                edge=new EdgeProjection(K,left_ext);
            }else{
                edge=new EdgeProjection(K,right_ext);
            }

            //如果landmars 还没有被加入优化，则新加一个点
            if(vertices_lanmarks.find(landmark_id)==vertices_lanmarks.end()){
                vertices_lanmarks.end();
                VertexXYZ *v=new VertexXYZ;
                v->setEstimate(landmark.second->Pos());
                v->setId(landmark_id+max_kf_id+1);
                v->setMarginalized(true);
                vertices_lanmarks.insert({landmark_id,v});
                optimizer.addVertex(v);
            }

            edge->setId(index);
            edge->setVertex(0,vertices.at(frame->keyframe_id_)); // pose
            edge->setVertex(1,vertices_lanmarks.at(landmark_id)); //landmark
            edge->setMeasurement(toVec2(feat->position_.pt));
            edge->setInformation(Mat22::Identity());
            auto rk=new g2o::RobustKernelHuber();
            rk->setDelta(chi2_th);
            edge->setRobustKernel(rk);
            edges_and_features.insert({edge,feat});
            optimizer.addEdge(edge);
            index++;
        }
    }

    //do optimization and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier=0,cnt_inlier=0;
    int iteration=0;
    while(iteration <5){
        cnt_outlier=0;
        cnt_inlier=0;
        //determine if we want to adjust the outlier threshold
        for(auto &ef:edges_and_features){
            if(ef.first->chi2()>chi2_th){
                cnt_outlier++;
            }else{
                cnt_inlier++;
            }
        }
        double inlier_ratio=cnt_inlier/double(cnt_inlier+cnt_inlier);
        if(inlier_ratio>0.5){
            break;
        }else{
            chi2_th*=2;
            iteration++;
        }
    }

    for(auto &ef:edges_and_features){
        if(ef.first->chi2()>chi2_th){
            ef.second->is_outlier_=true;
            //remove the observation
            ef.second->map_point_.lock()->RemoveObservation(ef.second);
        }else{
            ef.second->is_outlier_=false;
        }
    }

    LOG(INFO) <<"Outlier/Inlier in optimization:  "<<cnt_outlier<<"/ "<<cnt_inlier;

    //Set pose and landmark position
    for(auto &v:vertices){
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }

    for(auto &v:vertices_lanmarks){
        landmarks.at(v.first)->SetPos(v.second->estimate());
    }
}
}  //namespce mylsam 