#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
#include <opencv2/viz.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/calib3d.hpp>


using namespace std;
using namespace Eigen;
using namespace cv;

#define STEP 30

int main() {
    //****************Rini -> phi -> Rfin & tini, tfin******************
    Matrix3d Rini, Rfin;
    Rini << 1,0,0,0,1,0,0,0,1;
    AngleAxisd phi(M_PI , Vector3d(1/sqrt(3),1/sqrt(3),1/sqrt(3)));
    Rfin = phi.toRotationMatrix() * Rini;
    cout << phi.toRotationMatrix() <<endl;
    cv::Vec3d cv_tini(0,0,0), cv_tfin(0.3,0.3,0.3);

    //****************Get SO3ini & vdiff & tdiff*********************
    Sophus::SO3d SO3ini(Rini), SO3fin(Rfin);
    Sophus::SO3d SO3_Rdiff = SO3ini.inverse() * SO3fin;
    Vector3d vdiff = SO3_Rdiff.log();
    cv::Vec3d cv_tdiff = cv_tfin - cv_tini;

    //***************Start Visualizing********************************
    viz::Viz3d viz_windows("Window");
    viz_windows.showWidget("Frame", cv::viz::WCoordinateSystem(0.3));

    while(!viz_windows.wasStopped()){
        for(int i = 0; i <= STEP+1; i++){
            double progress = (double)i * 1. / STEP;
            Sophus::SO3d SO3t = SO3ini * Sophus::SO3d::exp(progress * vdiff);
            // cout << "SO3(Time=" << Time << "s)= \n" << SO3t.matrix() << endl;
            Matrix3d Rt = SO3t.matrix();
            cv::Mat cv_Rt = (cv::Mat_<double>(3,3) << Rt(0,0), Rt(0,1), Rt(0,2),
                    Rt(1,0), Rt(1,1), Rt(1,2),
                    Rt(2,0), Rt(2,1), Rt(2,2));
            cv::Vec3d cv_tt = cv_tini + progress * cv_tdiff;
            cv::Affine3d cv_Tt(cv_Rt, cv_tt);
            viz_windows.setWidgetPose("Frame", cv_Tt);
            viz_windows.spinOnce(100, true);
        }
    }

    return 0;
}
