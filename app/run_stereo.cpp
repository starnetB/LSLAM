#include <gflags/gflags.h>
#include <visual_odometry.h>

//设置输入命令行参数默认
DEFINE_string(config_file,"../config/default.yaml","config file path");

int main(int argc,char **argv){
    google::ParseCommandLineFlags(&argc,&argv,true);
    lslam::VisualOdometry::Ptr vo(new lslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init()==true);
    vo->Run();
    return 0;
}