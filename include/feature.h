#pragma once

#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include <common_include.h>

namespace lslam{
struct Frame;
struct MapPoint;

/**
 * 2D 特征点
 * 在三角化之后会被关联一个地图点
 */
struct Feature{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    // 当对象嵌套时，最好使用weak_ptr 这样可以避免循环引用
    // 简而言之，Frame 持有了Feature的shared_ptr,那么应该避免Feature
    // 再持有Frame的shared_Ptr.否则两者相互引用，将导致只能指针无法自动析构

    std::weak_ptr<Frame> frame_;          // 持有该feature的frame
    cv::KeyPoint position_;               // 2D提取位置
    std::weak_ptr<MapPoint> map_point_;   //关联地图点

    bool is_outlier_=false;               //是否是异常点
    bool is_on_left_image_=true;         //标识是否在左图,false为右图

public:
    Feature(){}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        :frame_(frame), position_(kp) {}

};
}   //namespace lslam
#endif