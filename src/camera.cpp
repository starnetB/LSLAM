#include <camera.h>

namespace lslam{

Camera::Camera(){}

//这里呢有点注意点，函数名中的camera是指single camera,应该是leftcamera
// T_C_W中的C代表的camera 是指stereo camera
Vec3 Camera::world2camera(const Vec3 &p_w,const SE3 &T_c_w){
    // pose_* H :from stereo camera to single camera   single->stereo
    // T_c_w* H :from world to stereo camera  camera->world
    return pose_ *T_c_w *p_w;
}

Vec3 Camera::camera2world(const Vec3 &p_c,const SE3 &T_c_w){
    return T_c_w.inverse()*pose_inv_*p_c;
}

Vec2 Camera::camera2pixel(const Vec3 &p_c){
    return Vec2(
        fx_*p_c(0,0)/p_c(2,0)+cx_,
        fy_*p_c(1,0)/p_c(2,0)+cy_
    );
}

Vec3 Camera::pixel2camera(const Vec2 &p_p,double depth){
    return Vec3(
        (p_p(0,0)- cx_)*depth/fx_,
        (p_p(1,0)- cy_)*depth/fy_,
        depth
    );
}

Vec2 Camera::world2pixel(const Vec3 &p_w,const SE3 &T_c_w){
    return camera2pixel(world2camera(p_w,T_c_w));
}

Vec3 Camera::pixel2world(const Vec2 &p_p,const SE3 &T_c_w,double depth){
    return camera2world(pixel2camera(p_p,depth),T_c_w);
}
}