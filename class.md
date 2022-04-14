## Feature
* 2D特征点管理类
  * map_point_std::weak_ptr 指向一个MapPoint
  * is_outlier:是否是异常点
  * is_on_left_image_:bool是否在做图

## MapPoint
* 地图点，想对于世界坐标系  
  * id
  * is_outlier
  * pos_:Vec3  世界坐标系下的异常点
  * observation_list:MapPoint观测到的特征点的列表，如果没有观测点的时候，MapPoint会被移除  


## Frame
* 每一帧图片的管理器
  * id
  * keyframe_id
  * is_keyframe:bool  是否定义为关键帧
  * pose_:SE3   当前帧相对于世界坐标系的位姿，C to world
  * left_img:   左图
  * right_img:   右图
  * features_left:FeatureVector 左图特征点的列表
  * features_right:FeatureVector 有图特征点的列表

## Map
* landmarks_:mappointVector  当前地图点共有多少个路标
* active_landmarks:mappoingVector 当前Map多少个路标支持更新 ，后端
* keyFrames:Framevector  关键帧
* active_keyframes:FrameVector  激活中关键帧
* current_frame:frame
* num_active_keyframes:int 激活关键帧的数量


## Frontend
* StereoInit()
  * 初始化双目相机：通过current_frame，初始化Map，Map中地图点，以及current_frame(left,right)特征的点的数量
  * Track():通过last_frame，和current_frame中的left_features(包含光流法的追中),更新当前帧的位置，当点不够是重新通过InsertKeyFrame()补充特征点，Map等以及更新激活帧等

## g2o_type
*  g2o_type优化的相关内容
*  有些错误

## algorith.h
* 算法相关内容

## Config
* 配置读取的相关内容


## backend
* 创建线程，挂起线程，
* 对Map中的活跃的关键帧和活跃的地图点进行优化