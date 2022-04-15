#include "viewer.h"
#include "feature.h"
#include "frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace lslam{
Viewer::Viewer(){
    viewer_thread_=std::thread(std::bind(&Viewer::ThreadLoop,this));
}

void Viewer::Close(){
    viewer_running_=false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame){
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_=current_frame;
}

void Viewer::UpdateMap(){
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_!=nullptr);
    active_keyframes_=map_->GetActiveKeyFrames();
    active_landmarks_=map_->GetActiveMapPoints();
    map_updated_=true;
}

void Viewer::ThreadLoop(){
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("LSLAM",1024,768);

    /**
    * glEnable(GL_DEPTH_TEST)：
    *  用来开启更新深度缓冲区的功能，也就是，如果通过比较后深度值发生变化了，会进行更新深度缓冲区的操作。
    * 启动它，OpenGL就可以跟踪再Z轴上的像素，这样，它只会再那个像素前方没有东西时，才会绘画这个像素。
    * 在做绘画3D时，这个功能最好启动，视觉效果比较真实。
    * */
    glEnable(GL_DEPTH_TEST);

    /**
    * Blend 混合是将源色和目标色以某种方式混合生成特效的技术。混合常用来绘制透明或半透明的物体。
    * 在混合中起关键作用的α值实际上是将源色和目标色按给定比率进行混合，以达到不同程度的透明。
    * α值为0则完全透明，α值为1则完全不透明。混合操作只能在RGBA模式下进行，颜色索引模式下无法指定α值。
    * 物体的绘制顺序会影响到OpenGL的混合处理。*/
    glEnable(GL_BLEND);
    /**
    * 如果设置了glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    * 则表示源颜色乘以自身的alpha 值，目标颜色乘以1.0减去源颜色的alpha值，这样一来，
    * 源颜色的alpha值越大，则产生的新颜色中源颜色所占比例就越大，而目标颜色所占比例则减小。
    * 这种情况下，我们可以简单的将源颜色的alpha值理解为“不透明度”。这也是混合时最常用的方式。*/
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    pangolin::OpenGlRenderState vis_camera(
        /**
         * w、h：相机的视野宽、高
         * fu、fv、u0、v0相机的内参，对应《视觉SLAM十四讲》中内参矩阵的fx、fy、cx、cy
         * zNear、zFar：相机的最近、最远视距
         */
    
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        
        // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
        //                观测目标位置：(0, 0, 0)
        //                观测的方位向量：(0.0,-1.0, 0.0)
        // 摄像机初始时刻所处的位置 摄像机的视点位置（即摄像机的光轴朝向哪一个点）以及摄像机的本身哪一轴朝上
        // -1 就是y轴朝下
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    //Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& vis_display =
        pangolin::CreateDisplay()
        //前四个参数依次表示视图在视窗中的范围（下、上、左、右），可以采用相对坐标（0~1）以及绝对坐标（使用Attach对象）。 
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));
    
    const float blue[3]={0,0,1};
    const float green[3]={0,1,0};  

    while(!pangolin::ShouldQuit() && viewer_running_){
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if(current_frame_){
            //画出当前帧的电视机框框
            DrawFrame(current_frame_,green);  //画出当前帧框架
            FollowCurrentFrame(vis_camera);   //追踪到当前相机

            cv::Mat img=PlotFrameImage();  //画出左图特征点
            cv::imshow("image",img);
            cv::waitKey(1);
        }

        if(map_) {
            DrawMapPoints();
        }

        pangolin::FinishFrame(); //结束当前帧
        usleep(5000);
    }

    LOG(INFO) << "Stop viewer";
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera)
{
    //跟中相机
    SE3 Twc=current_frame_->Pose().inverse();   //C->W inver W->C
    pangolin::OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m,true);  //跟综到当前相机 
}

cv::Mat Viewer::PlotFrameImage(){
    cv::Mat img_out;
    cv::cvtColor(current_frame_->left_img_,img_out,CV_GRAY2BGR);
    for(size_t i=0;i<current_frame_->features_left_.size();++i){
        if(current_frame_->features_left_[i]->map_point_.lock()){
            auto feat=current_frame_->features_left_[i];
            cv::circle(img_out,feat->position_.pt,2,cv::Scalar(0,255,0),2);
        }
    }
}

void Viewer::DrawFrame(Frame::Ptr frame,const float* color){
    SE3 Twc=frame->Pose().inverse();  //W->C
    const float sz=1.0;
    const int line_width=2.0;
    const float fx=400;
    const float fy=400;
    const float cx=512;
    const float cy=384;
    const float width=1080;
    const float height=768;
    // 把当前模型保存到视图矩阵，
    // 以后之前定义的参数作为坐标元点
    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    //将当前坐标系转换的frame坐标系
    glMultMatrixf((GLfloat*)m.data());

    if(color==nullptr){
        glColor3f(1,0,0);
    }else{
        glColor3f(color[0],color[1],color[2]);
    }
    glLineWidth(line_width);
    glBegin(GL_LINES);
    //画上一个类似与电视机打框框的宏
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    //弹出位姿举证，回到之前的矩阵
    glPopMatrix();
}

void Viewer::DrawMapPoints(){
    const float red[3]={1.0,0,0};
    for(auto& kf:active_keyframes_){
        DrawFrame(kf.second,red);   //画出帧对应的相机位置
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for(auto& landmark:active_landmarks_){
        auto pos=landmark.second->Pos();
        glColor3f(red[0],red[1],red[2]);
        glVertex3d(pos[0],pos[1],pos[2]);
    }
    glEnd();
}
}  //namespace lslam