#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
#include <opencv2/viz.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/calib3d.hpp>
using namespace std;

template<typename T>
Eigen::Matrix<T,4,4> interpolateSE3(const Eigen::Matrix<T,4,4> & source, const Eigen::Matrix<T,4,4> & target, const T &alpha){

//    if(alpha<0 || alpha>1)
//    {
//        cerr << "warning: alpha < 0 or alpha > 1" <<endl;
//    }

    Sophus::SE3<T> SE1(source);
    Sophus::SE3<T> SE2(target);

    Eigen::Matrix<T, 6, 1> se1 = SE1.log();
    Eigen::Matrix<T, 6, 1> se2 = SE2.log();

    Sophus::SE3<T> SE3t = SE1 * Sophus::SE3<T>::exp(alpha * (se2-se1));

    return SE3t.matrix();
}

cv::Affine3f toCvAffine(Eigen::Matrix4f matrix);

#define STEP 30

int main() {
    Eigen::Matrix4f source = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f target = Eigen::Matrix4f::Identity();
    target.block<3,3>(0,0) = Eigen::AngleAxisf(M_PI / 2 , Eigen::Vector3f(0,0,1)).toRotationMatrix();
    target.block<3,1>(0,3) = Eigen::Vector3f(1.0,0,0);

    //***************Start Visualizing********************************
    cv::viz::Viz3d viz_windows("Window");
    viz_windows.showWidget("Frame", cv::viz::WCoordinateSystem(0.3));
    viz_windows.showWidget("FrameFinal", cv::viz::WCoordinateSystem(0.3));

    cv::Affine3f cv_final = toCvAffine(target);
    while(!viz_windows.wasStopped()){
        for(int i = 0; i <= STEP; i++){
            float progress = (double)i * 1. / STEP;
            Eigen::Matrix4f mat = interpolateSE3(source, target, progress);
            cv::Affine3f cv_Tt = toCvAffine(mat);
            viz_windows.setWidgetPose("Frame", cv_Tt);
            viz_windows.setWidgetPose("FrameFinal", cv_final);
            viz_windows.spinOnce(100, true);
        }
    }

    return 0;
}


cv::Affine3f toCvAffine(Eigen::Matrix4f matrix){
    
    
    Eigen::Matrix3f Rfin = matrix.block<3,3>(0,0);
    cv::Vec3f cv_tini(matrix(0,3),matrix(1,3),matrix(2,3));
    
    cv::Mat cv_final_R = (cv::Mat_<float>(3,3) << Rfin(0,0), Rfin(0,1), Rfin(0,2),
            Rfin(1,0), Rfin(1,1), Rfin(1,2),
            Rfin(2,0), Rfin(2,1), Rfin(2,2));
    
    cv::Affine3f cv_final(cv_final_R, cv_tini);
    
    return cv_final;
}
