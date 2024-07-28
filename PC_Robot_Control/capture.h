#ifndef CAPTURE_H
#define CAPTURE_H

//#include <QObject>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <QThread>
#include <QDebug>
#include <QPixmap>
#include <QElapsedTimer>

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

#include <cmath>

using namespace cv;
using namespace rs2;

class capture : public QThread
{
    Q_OBJECT
public:
    explicit capture(QObject *parent = nullptr);

    int max_x_cordinate = -1, classification;

    cv::Point closestPoint,farthestPoint, centroidObj;
    bool gotClosestPoint = false;
    bool prepareGrip = false;
    int objectColor = -1;
    bool coVat;
    std::vector<cv::Point> centroiVector;

    int getObjectColor() {
        qDebug() << "object color at capture: "<< objectColor;
            return objectColor;
        }

    bool checkCameraState()
    {
        return cameraState;
    }

    double distanceGrip()
    {
        return distanceGripper;
    }

    bool changeStreamOpt(const uint8_t value)
    {
      streamOpt = value;
    }

    void startCamera()
    {
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);

        // Get intrinsics
        auto stream = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        intrin = stream.get_intrinsics();

        cameraState = true;

    }
    void stopCamera()
    {
        cameraState = false;
        cfg.disable_all_streams();
        pipe.stop();
    }

    cv::Mat getObjPos2Base(){
        qDebug() << "jump to getObjPos2Base()";
        return translation_result;
    }

    double rotationAngle()
    {
        return rtAngle;
    }

    bool objInPos() {
        getObjPos2Base();
        return objectInPos;
    }

    bool isObjIsOn(){
        return isObj;
    }
    bool resetObjInPos() {
        this->objectColor = -1;     // co sua them de chay code P003
        return objectInPos = false;
    }
    void resetFlag(){
        objectInPos = false;
        isObj       = false;
    }

    /*debug*/
    bool objinPosTest(){
       return objectInPos;
    }

    float center_pixel_distance, posObjZCoor;
    float pixel[2];
    float point_in_camera_space[3];

    void emitToPixmapCaptured();

    cv::Mat findAngleObject(const cv::Mat &frame);
    void toggleAutoRunSystem();
    void caculatePosCam2Base(float point_in_camera_space[3], cv::Mat& t_result);
    void testCode(cv::Mat& t_result);
    cv::Mat objDetect(const cv::Mat& frame, cv::Point &center);

    cv::Mat objDetect_1(const cv::Mat &frameInput, cv::Point &center);
signals:
    void newPixmapCaptured(QPixmap pixMap);

public slots:
//    void detectAndDrawArUco(cv::Mat &colorFrame);
    void detectAndDrawArUco(cv::Mat &colorFrame);
    void toggleArucoDetection();
    // QThread interface
protected:
    void run();

private slots:
    QImage cvMatToQImage(const Mat &inMat);
    QPixmap cvMatToQPixmap(const Mat &inMat);

private:
    rs2::config cfg;
    rs2::pipeline pipe;
    rs2::colorizer colorMap;
    QPixmap pixmapColor;
    bool cameraState = false;
    uint8_t streamOpt = 0;

    cv::Mat cameraMatrix;
    cv::Mat distCoeeffs;
    //------------------khởi tạo bộ lọc cho depth --------------------
    // Khởi tạo các bộ lọc
    rs2::decimation_filter decimation_filter;
    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;
    rs2::hole_filling_filter hole_filling_filter;
    rs2::colorizer color_map;
    rs2::threshold_filter threshold_filter;
    //------------------set opt for filters---------------------------


    //------------------for calib-------------
    rs2_intrinsics intrin;

    std::vector<cv::Vec3d> rvecs, tvecs;
    //----------------for detect--------------


    //----------------for detect ArUco--------
    bool detectAruco = false;
    bool autoRunSystemBool = false;

    //----------------for position caculate---

    cv::Mat R_cam2base = (cv::Mat_<double>(3, 3) <<
                                                      0.001178732033102349,  0.999855009442505, 0.01698736835187691,
                                                      0.9999213407273516,   -0.0009663500075555721, -0.0125051399705217,
                                                     -0.01248691109977005,   0.01700077234689937, -0.9997775006424154);
    cv::Mat t_cam2base = (cv::Mat_<double>(3, 1) << 276.2543668244658,
                                            -201.2058165545883,
                                            236.2664808085231);
    cv::Mat objPos2Base;  
    cv::Mat homogeneous_matrix = cv::Mat::eye(4, 4, CV_64F);
    double rtAngle;
    cv::Mat translation_result;
    double distanceGripper;
    bool objectInPos = false;
    bool isObj = false;


};

#endif // CAPTURE_H
