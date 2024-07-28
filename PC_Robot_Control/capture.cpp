#include "capture.h"

capture::capture(QObject *parent) : QThread (parent)
{
    cameraMatrix = cv::Mat::eye(3,3, CV_64F);
    distCoeeffs = cv::Mat::zeros (5,1, CV_64F);
//    coVat = false;

//-------------------------640x480--------------------------
//    cameraMatrix.at<double>(0, 0) = 383.046447753906; // Fx
//    cameraMatrix.at<double>(1, 1) = 383.046447753906; // Fy
//    cameraMatrix.at<double>(0, 2) = 323.402587890625; // PPX
//    cameraMatrix.at<double>(1, 2) = 238.035278320313; //PPY


    cameraMatrix.at<double>(0, 0) = 606.281066894531; // Fx
    cameraMatrix.at<double>(1, 1) = 606.417541503906; // Fy
    cameraMatrix.at<double>(0, 2) = 327.089965820313; // PPX
    cameraMatrix.at<double>(1, 2) = 240.459075927734; //PPY

    /*
 Intrinsic of "Color" / 640x480 / {YUYV/RGB8/BGR8/RGBA8/BGRA8/Y16}
  Width:        640
  Height:       480
  PPX:          327.089965820313
  PPY:          240.459075927734
  Fx:           606.281066894531
  Fy:           606.417541503906
  Distortion:   Inverse Brown Conrady
  Coeffs:       0       0       0       0       0
  FOV (deg):    55.65 x 43.18


 Intrinsic of "Depth" / 640x480 / {Z16}
  Width:        640
  Height:       480
  PPX:          323.402587890625
  PPY:          238.035278320313
  Fx:           383.046447753906
  Fy:           383.046447753906
  Distortion:   Brown Conrady
  Coeffs:       0       0       0       0       0
  FOV (deg):    79.75 x 64.14
*/

//------------------------- Matrix after calibration-------
//    R_cam2base = (cv::Mat_<double>(3, 3) <<
//                  0.001582608077934592, 0.9998460122431739, 0.01747704646377178,
//                   0.9999290166609106, -0.001375865721047336, -0.01183505948709409,
//                   -0.01180919096369771, 0.01749453614540405, -0.9997772172908527);

//    t_cam2base = (cv::Mat_<double>(3, 1) << 276.2106592589532,
//                                            -201.5256069075959,
//                                            237.2815315118312);

//    R_cam2base = (cv::Mat_<double>(3, 3) <<
//                  0.001178732033102349, 0.999855009442505, 0.01698736835187691,
//                   0.9999213407273516, -0.0009663500075555721, -0.0125051399705217,
//                   -0.01248691109977005, 0.01700077234689937, -0.9997775006424154);
//    t_cam2base = (cv::Mat_<double>(3, 1) << 276.2543668244658
//                                            -201.2058165545883,
//                                            236.2664808085231);

    // Khởi tạo ma trận đồng nhất 4x4

    R_cam2base.copyTo(homogeneous_matrix(cv::Rect(0, 0, 3, 3)));
    t_cam2base.copyTo(homogeneous_matrix(cv::Rect(3, 0, 1, 3)));

    /*
    Filter magnitude = sub-sampling number (ex: 2, 3, 4..)

    Filter magnitude = iterations of the filter. We recommend using 2 (default).
    Filter smooth alpha = the spatial alpha parameters (0-1)
    Filter smooth delta = the spatial delta threshold in 1/32 disparities. Default is 20, but try 8 as well.
    Filter holes fill = the spatial radius for hole filling (0=none, 1=2 pixels, 2=4 pixels, 3=8 pixels, 4=16 pixels, 5=unlimited).

    Filter smooth alpha = the temporal alpha parameters (0-1)
    Filter smooth delta = the temporal delta threshold in 1/32 disparities. Default is 20.
    Filter holes fill = the persistence value (0=no persistence, and 8=persist indefinitely)

    Filter holes fill = additional filling of holes based on choosing nearest neighbor filled with 0=Use left pixel to fill, 1=Max of nearest pixels on left, and 2=Min value of nearest pixels on left.

    rs2::decimation_filter decimation_filter;
    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;
    rs2::hole_filling_filter hole_filling_filter;
    rs2::colorizer color_map;
*/


//    decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
//    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
//    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 8);       //try recommend
//    hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, 1);



}
void capture::run()
{
    rs2::align align_to_color(RS2_STREAM_COLOR);

    decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 8);       //try recommend
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0);
    threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.0);
    threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, 1.0);
    hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, 2);

    while (cameraState)
    {
        QElapsedTimer time;
        time.start();

        rs2::frameset data = pipe.wait_for_frames();
        data = align_to_color.process(data);
        frame color = data.get_color_frame();
        frame depth = data.get_depth_frame().apply_filter(colorMap);

        //-----------------------------------apply filter cho depth --------------
//        frame depth = data.get_depth_frame();
//////////        // Áp dụng các bộ lọc
//////        depth = decimation_filter.process(depth);
//        depth = depth_to_disparity.process(depth);
////        depth = spatial_filter.process(depth);
//        depth = temporal_filter.process(depth);
//        depth = disparity_to_depth.process(depth);
//        depth = hole_filling_filter.process(depth);
////        depth = color_map.process(depth);
//        depth = depth.apply_filter(colorMap);

//        //----------------------------------------------

        auto stream = color.get_profile().as<rs2::video_stream_profile>();

        // Get depth frame
        rs2::depth_frame depth_frame = data.get_depth_frame();

        const int w_depth = depth.as<rs2::video_frame>().get_width();
        const int h_depth = depth.as<rs2::video_frame>().get_height();

        const int w_color = color.as<rs2::video_frame>().get_width();
        const int h_color = color.as<rs2::video_frame>().get_height();

        Mat depthFrame(Size(w_depth,h_depth), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        Mat colorFrame(Size(w_color,h_color), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        if (!colorFrame.empty() && !depthFrame.empty())
        {
            intrin = stream.get_intrinsics();

            if (detectAruco)
            {
                detectAndDrawArUco(colorFrame);
//                   float depth_value = depthFrame.at<float>(center.y, center.x);

                center_pixel_distance = depth_frame.get_distance( pixel[0], pixel[1]);

//                intrin.fx = 606.281066894531f; // Focal length in pixels (in x-direction)
//                intrin.fy = 606.417541503906f; // Focal length in pixels (in y-direction)
//                intrin.ppx = 327.089965820313f; // Principal point (in x-direction)
//                intrin.ppy = 240.459075927734f;
                rs2_deproject_pixel_to_point (point_in_camera_space, &intrin, pixel, center_pixel_distance);
                std::cout << "3D Point in Camera Space: (" << point_in_camera_space[0] << ", " << point_in_camera_space[1] << ", " << point_in_camera_space[2] << ")" << std::endl;
                qDebug()<< "center_pixel_distance: "<< center_pixel_distance;
            }

            if (autoRunSystemBool )
            {
//                qDebug() << "Here color detect" ;
                detectAruco = false;
//                colorFrame = findAngleObject(colorFrame);
                colorFrame = objDetect_1(colorFrame, centroidObj);

//                qDebug() << " centroidObj value at capture thread: " << centroidObj.x ;
//                    isObj = true;
                posObjZCoor = depth_frame.get_distance( centroidObj.x, centroidObj.y);
//                std::cout << "have value of pos: " <<posObjZCoor;
                float pixel[2] = { float (centroidObj.x), float (centroidObj.y)};
//                std::cout << "have value of pixel : " <<pixel[0];
//                std::cout << "have value of pixel : " <<pixel[1];
//                float point_in_camera_space[3];
                 /*Use rs2_deproject_pixel_to_point to get the 3D coordinates*/
                 rs2_deproject_pixel_to_point(point_in_camera_space, &intrin, pixel, posObjZCoor);

//                qDebug() << "point in camera space 0: " << point_in_camera_space[0];
//                qDebug() << "point in camera space 1: " << point_in_camera_space[1];
//                qDebug() << "point in camera space 2: " << point_in_camera_space[2];
                 caculatePosCam2Base(point_in_camera_space, translation_result);
            }

//           qDebug()<< int (1000/time.elapsed());  // fps value
            if (streamOpt == 0)     pixmapColor = cvMatToQPixmap(colorFrame);
            else                    pixmapColor = cvMatToQPixmap(depthFrame);


        }
//        qDebug() <<"value of streamOpt: "<< streamOpt;
        Q_EMIT newPixmapCaptured(pixmapColor);
    }
}

//--------------------------------detect ArUco-------------------------------
void capture::detectAndDrawArUco(cv::Mat &colorFrame)
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

    cv::aruco::detectMarkers(colorFrame, dictionary, markerCorners, ids, detectorParams, rejectedCandidates);

    cv::aruco::estimatePoseSingleMarkers(markerCorners, 65.0, cameraMatrix, distCoeeffs, rvecs, tvecs);
//    cv::aruco::drawAxis(colorFrame, cameraMatrix, distCoeeffs, rvecs, tvecs, 1.5);

    for(size_t i = 0; i < ids.size(); ++i) {

        cv::aruco::drawAxis(colorFrame, cameraMatrix, distCoeeffs, rvecs[i], tvecs[i], 1.5);
        // Calculate the center of the marker
        cv::Point2f center(0,0);
        for (const auto& corner : markerCorners[i]) {
            center.x += corner.x;
            center.y += corner.y;
        }
        center.x /= markerCorners[i].size();
        center.y /= markerCorners[i].size();
        cv::circle(colorFrame, center, 5, cv::Scalar(0, 255, 0), -1);

        pixel[0] = center.x;
        pixel[1] = center.y;
    }
}
void capture::toggleArucoDetection()
{
    detectAruco = !detectAruco;
    qDebug() << "value of detectArUco: "<< detectAruco;
}
//-----------------------------------------------------------------------------

//--------------------------------------over test code-------------------------
QImage capture::cvMatToQImage(const Mat &inMat){
    switch ( inMat.type() )
              {
                 // 8-bit, 4 channel
                 case CV_8UC4:
                 {
                    QImage image( inMat.data,
                                  inMat.cols, inMat.rows,
                                  static_cast<int>(inMat.step),
                                  QImage::Format_ARGB32 );

                    return image;
                 }

                 // 8-bit, 3 channel
                 case CV_8UC3:
                 {
                    QImage image( inMat.data,
                                  inMat.cols, inMat.rows,
                                  static_cast<int>(inMat.step),
                                  QImage::Format_RGB888 );

                    return image.rgbSwapped();
                 }

                 // 8-bit, 1 channel
                 case CV_8UC1:
                 {
        #if QT_VERSION >= QT_VERSION_CHECK(5, 15, 2)
                    QImage image( inMat.data,
                                  inMat.cols, inMat.rows,
                                  static_cast<int>(inMat.step),
                                  QImage::Format_Grayscale8 );
        #else
                    static QVector<QRgb>  sColorTable;

                    // only create our color table the first time
                    if ( sColorTable.isEmpty() )
                    {
                       sColorTable.resize( 256 );

                       for ( int i = 0; i < 256; ++i )
                       {
                          sColorTable[i] = qRgb( i, i, i );
                       }
                    }

                    QImage image( inMat.data,
                                  inMat.cols, inMat.rows,
                                  static_cast<int>(inMat.step),
                                  QImage::Format_Indexed8 );

                    image.setColorTable( sColorTable );
        #endif

                    return image;
                 }

                 default:
                    qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
                    break;
              }

    return QImage();
}

QPixmap capture::cvMatToQPixmap(const Mat &inMat){
    return QPixmap::fromImage(cvMatToQImage(inMat));
}
//----------------------------------------Image Processing----------------------
void capture::toggleAutoRunSystem()
{
    autoRunSystemBool = !autoRunSystemBool;
}
/*
cv::Mat capture::findAngleObject(const cv::Mat& frame)
{

//    cv::Scalar orangeLow = cv::Scalar(5,231, 130);     //6 225 128
//    cv::Scalar orangeHigh = cv::Scalar(15,255, 220 );

//    cv::Scalar greenLow = cv::Scalar(41,66,51);     //6 225 128
//    cv::Scalar greenHigh = cv::Scalar(61,231,150);

//    cv::Scalar yellowLow = cv::Scalar(21,197,134);
//    cv::Scalar yellowHigh = cv::Scalar(36,255,175);

//    cv::Scalar redLow = cv::Scalar(0,83,97);     //6 225 128
//    cv::Scalar redHigh = cv::Scalar(12,241,191);

    cv::Scalar orangeLow = cv::Scalar(6, 212, 128);
    cv::Scalar orangeHigh = cv::Scalar(17, 255, 206);

    cv::Scalar greenLow = cv::Scalar(38, 79, 61);
    cv::Scalar greenHigh = cv::Scalar(56, 231,  159);

    cv::Scalar yellowLow = cv::Scalar(20, 164, 107);
    cv::Scalar yellowHigh = cv::Scalar(31, 255, 200);

    cv::Scalar redLow = cv::Scalar(0, 163, 79);     //6 225 128
    cv::Scalar redHigh = cv::Scalar(5, 239, 185);

    cv::Mat colorFrameHSV;
    cv::cvtColor(frame, colorFrameHSV, cv::COLOR_BGR2HSV);

    cv::Mat maskAll, maskOrange, maskGreen, maskYellow, maskRed;
    cv::inRange(colorFrameHSV, orangeLow, orangeHigh, maskOrange);
    cv::inRange(colorFrameHSV, greenLow, greenHigh, maskGreen);
    cv::inRange(colorFrameHSV, yellowLow, yellowHigh, maskYellow);
    cv::inRange(colorFrameHSV, redLow, redHigh, maskRed);


    // Combine masks
    maskAll = maskOrange | maskGreen | maskYellow | maskRed;

    int kernelSize = 7;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::erode(maskAll, maskAll, kernel);
    cv::dilate(maskAll, maskAll, kernel);


    // Find contours for each mask
    std::vector<std::vector<cv::Point>> contoursOrange, contoursGreen, contoursYellow, contoursRed;
    cv::findContours(maskOrange, contoursOrange, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(maskGreen, contoursGreen, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(maskYellow, contoursYellow, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(maskRed, contoursRed, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::imshow("Show mask", maskAll);

    int max_x_cordinate = 0;
    this->objectColor=-1;

    //----------------------------------------------------------------------------------------------
        for (size_t i = 0; i < contoursOrange.size(); i++)
        {

            double area = cv::contourArea(contoursOrange[i]);
            if (area > 5000) { // Area threshold

                // Calculate centroid
                cv::Moments M = cv::moments(contoursOrange[i]);
                cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);

                // Draw marker at the centroid position
                cv::circle(frame, centroid, 5, cv::Scalar(255, 0, 0), -1);
                // Vẽ đường bao contour của vật
                cv::drawContours(frame, contoursOrange, static_cast<int>(i), cv::Scalar(0, 168, 200), 2);
                // Calculate bounding rectangle
                cv::Rect boundingRect = cv::boundingRect(contoursOrange[i]);

//-----------------------------Tìm điểm trên contour có khoảng cách ngắn nhất từ tâm--------------
        // Tìm contour mà closestPoint thuộc vào

                double minDist = std::numeric_limits<double>::max(); // Khởi tạo khoảng cách tối thiểu là giá trị lớn nhất
                cv::Point closestPoint_temp;
                for (const auto& point : contoursOrange[i]) {
                    double dist = cv::norm(point - centroid); // Tính khoảng cách từ tâm đến điểm trên contour
                    if (dist < minDist) {
                        minDist = dist;
                        closestPoint_temp = point;
                    }
                }

                 // Tính toán điểm đối xứng qua tâm
//                cv::Point symmetricPoint(2 * centroid.x - closestPoint_temp.x, 2 * centroid.y - closestPoint_temp.y);

                 // Vẽ đường thẳng từ tâm đến điểm trên contour có khoảng cách ngắn nhất
//                 cv::line(frame, closestPoint_temp, centroid, cv::Scalar(0, 168, 200), 2);

                 // Calculate slope and y-intercept of the line passing through closestPoint and centroid
                 double slope;
                 double y_intercept;
                 cv::Point farthest_point;
                 if (closestPoint_temp.x != centroid.x) {
                     // Tính slope như bình thường
                     slope = static_cast<double>(centroid.y - closestPoint_temp.y) / (centroid.x - closestPoint_temp.x);
                     // Tính y_intercept
                     y_intercept = closestPoint_temp.y - slope * closestPoint_temp.x;

                     // Find the point on the contour that has a y-intercept close to the line and the maximum distance from closestPoint
                     double min_dist = 10; // Set a minimum distance threshold
                     double max_dist = 0;

                     for (const auto& point : contoursOrange[i]) {
                         double y_intercept_diff = point.y - slope * point.x;
                         if (std::abs(y_intercept_diff - y_intercept) < 15) { // Allow for a tolerance of 5 pixels
                             double dist = cv::norm(point - closestPoint_temp);
                             if (dist > min_dist && dist > max_dist) {
                                 max_dist = dist;
                                 farthest_point = point;
                             }
                         }
                     }
                 } else {
                     // Trường hợp closestPoint và centroid thẳng hàng theo hướng dọc
                     slope = std::numeric_limits<double>::infinity(); // Đặt slope bằng vô cực
                     // Tính y_intercept
                     y_intercept = closestPoint_temp.y; // Đặt y_intercept là closestPoint.x

                     double max_dist = 0;

                     for (const auto& point : contoursOrange[i]) {
                         // Kiểm tra xem điểm có trên đường thẳng không (đường thẳng là song song với trục y, nên chỉ cần kiểm tra giá trị x)
                         if (std::abs(point.x - closestPoint_temp.x) < 5) { // Allow for a tolerance of 5 pixels
                             // Tính khoảng cách từ điểm đến centroid
                             double dist = cv::norm(point - centroid);
                             // Kiểm tra nếu dist lớn hơn max_dist, nếu có cập nhật farthest_point và max_dist
                             if (dist > max_dist) {
                                 max_dist = dist;
                                 farthest_point = point;
                             }
                         }
                     }
                 }

                 // Check if farthest_point is not equal to (0, 0)
                 if (farthest_point != cv::Point(0, 0)) {
                     // Draw the farthest_point
                     cv::circle(frame, farthest_point, 5, cv::Scalar(0, 255, 0), -1);
                     cv::line(frame, closestPoint_temp, farthest_point, cv::Scalar(0, 168, 200), 2);
//                     std::cout << "Fartherst Point before save: (" << farthestPoint.x << ", " << farthestPoint.y << ")" << std::endl;
                 }
         //---------------

         // Draw bounding rectangle
                cv::rectangle(frame, boundingRect, cv::Scalar(0, 168, 200), 2);

                std::cout << "Orange object detected" << std::endl;
                objectColor = 3;
                 // Replace 100 with your specified y-coordinate
                if (centroid.x == 250) {
                    closestPoint = closestPoint_temp;
                    farthestPoint = farthest_point;

                    cv::Point point1_line2(109, 364);
                    cv::Point point2_line2(323, 364);

                    centroidObj = centroid;
                    distanceGripper = minDist;   // write a function to get a distance real from Intel Realsense
                    std::cout << "Closest Point: (" << closestPoint.x << ", " << closestPoint.y << ")" << std::endl;
                    std::cout << "Fartherst Point: (" << farthestPoint.x << ", " << farthestPoint.y << ")" << std::endl;
                    std::cout << "Centroid of Object: (" << centroidObj.x << ", " << centroidObj.y << ")" << std::endl;
                    std::cout << "distanceGripper of Object: "<< distanceGripper << std::endl;

                    // Check if both points of each line are in the third or fourth quadrant
                    // Check if points are in the fourth quadrant
                    bool isQuadrant3or4 = (closestPoint.y < centroid.y && closestPoint.x < centroid.x) ||
                                            (farthestPoint.y > centroid.y && farthestPoint.x > centroid.x);

                    // Check if points are in the third quadrant
                    bool isQuadrant3or4_line2 = (closestPoint.y > centroid.y && closestPoint.x < centroid.x) ||
                                           (farthestPoint.y < centroid.y && farthestPoint.x > centroid.x);

                    if (isQuadrant3or4 || isQuadrant3or4_line2) {
                        // Calculate the slopes of both lines
                        double slope_line1 = static_cast<double>(closestPoint.y - farthestPoint.y) / (closestPoint.x - farthestPoint.x);
                        double slope_line2 = static_cast<double>(point2_line2.y - point1_line2.y) / (point2_line2.x - point1_line2.x);

                        // Calculate the angle between the two lines
                        double angle_rad = std::atan(std::abs((slope_line2 - slope_line1) / (1 + slope_line1 * slope_line2)));

//                        if (isQuadrant3or4) {
//                            angle_rad = CV_PI - angle_rad; // Reflecting third quadrant angle
//                        } else if (isQuadrant3or4_line2) {
//                            angle_rad = -angle_rad; // Reflecting fourth quadrant angle
//                        }
                        // Convert radians to degrees
                        double angle_deg = angle_rad * (180 / CV_PI);
//                        // Adjust the angle to be positive and within the range [0, 180]
                        if (angle_deg < 0) {
                            angle_deg += 180.0;
                        }

                        rtAngle = angle_deg;

                        // Print the angle
                        std::cout << "Angle between the two lines: " << angle_deg << " degrees" << std::endl;

                        prepareGrip = true;
                        qDebug() << "Value of prepareGrip:" << prepareGrip;
                    }
                }
            }
        }


    //-----------------------------------------------------------------------------------------
        for (size_t i = 0; i < contoursGreen.size(); i++)
        {
            double area = cv::contourArea(contoursGreen[i]);
            if (area > 5000) { // Area threshold

                // Calculate centroid
                cv::Moments M = cv::moments(contoursGreen[i]);
                cv::Point centroid_Green(M.m10 / M.m00, M.m01 / M.m00);

                // Draw marker at the centroid position
                cv::circle(frame, centroid_Green, 5, cv::Scalar(255, 0, 0), -1);
                // Vẽ đường bao contour của vật
                cv::drawContours(frame, contoursGreen, static_cast<int>(i), cv::Scalar(0, 168, 200), 2);
                // Calculate bounding rectangle
                cv::Rect boundingRect = cv::boundingRect(contoursGreen[i]);

     //-----------------------------Tìm điểm trên contour có khoảng cách ngắn nhất từ tâm--------------
                 double minDist = std::numeric_limits<double>::max(); // Khởi tạo khoảng cách tối thiểu là giá trị lớn nhất
                 cv::Point closestPoint_temp;
                 for (const auto& point : contoursGreen[i]) {
                     double dist = cv::norm(point - centroid_Green); // Tính khoảng cách từ tâm đến điểm trên contour
                     if (dist < minDist) {
                         minDist = dist;
                         closestPoint_temp = point;
                     }
                 }

                 // Vẽ đường thẳng từ tâm đến điểm trên contour có khoảng cách ngắn nhất
//                 cv::line(frame, centroid, closestPoint, cv::Scalar(0, 168, 200), 2);

                        //----------
                 // Calculate slope and y-intercept of the line passing through closestPoint and centroid
                 double slope;
                 double y_intercept;
                  cv::Point farthest_point;
                 if (closestPoint_temp.x != centroid_Green.x) {
                     // Tính slope như bình thường
                     slope = static_cast<double>(centroid_Green.y - closestPoint_temp.y) / (centroid_Green.x - closestPoint_temp.x);
                     // Tính y_intercept
                     y_intercept = closestPoint_temp.y - slope * closestPoint_temp.x;

                     // Find the point on the contour that has a y-intercept close to the line and the maximum distance from closestPoint
                     double min_dist = 10; // Set a minimum distance threshold
                     double max_dist = 0;

                     for (const auto& point : contoursGreen[i]) {
                         double y_intercept_diff = point.y - slope * point.x;
                         if (std::abs(y_intercept_diff - y_intercept) < 15) { // Allow for a tolerance of 5 pixels
                             double dist = cv::norm(point - closestPoint_temp);
                             if (dist > min_dist && dist > max_dist) {
                                 max_dist = dist;
                                 farthest_point = point;
                             }
                         }
                     }
                 } else {
                     // Trường hợp closestPoint và centroid thẳng hàng theo hướng dọc
                     slope = std::numeric_limits<double>::infinity(); // Đặt slope bằng vô cực
                     // Tính y_intercept
                     y_intercept = closestPoint_temp.y; // Đặt y_intercept là closestPoint.x

                     double max_dist = 0;

                     for (const auto& point : contoursGreen[i]) {
                         // Kiểm tra xem điểm có trên đường thẳng không (đường thẳng là song song với trục y, nên chỉ cần kiểm tra giá trị x)
                         if (std::abs(point.x - closestPoint_temp.x) < 5) { // Allow for a tolerance of 5 pixels
                             // Tính khoảng cách từ điểm đến centroid
                             double dist = cv::norm(point - centroid_Green);
                             // Kiểm tra nếu dist lớn hơn max_dist, nếu có cập nhật farthest_point và max_dist
                             if (dist > max_dist) {
                                 max_dist = dist;
                                 farthest_point = point;
                             }
                         }
                     }
                 }

                 // Check if farthest_point is not equal to (0, 0)
                 if (farthest_point != cv::Point(0, 0)) {
                     // Draw the farthest_point
                     cv::circle(frame, farthest_point, 5, cv::Scalar(0, 255, 0), -1);
                     cv::line(frame, closestPoint_temp, farthest_point, cv::Scalar(0, 168, 200), 2);
                 }
      //---------------------------------------------------------------------------------------------------

                // Draw bounding rectangle
//                cv::rectangle(frame, boundingRect, cv::Scalar(0, 168, 200), 2);

                std::cout << "Green object detected" << std::endl;
                // Set the object color variable
                objectColor = 2;

                if (centroid_Green.x == 250) {
                    qDebug() << "Here test data of Green Object";
                    closestPoint = closestPoint_temp;
                    farthestPoint = farthest_point;

                    cv::Point point1_line2(109, 364);
                    cv::Point point2_line2(323, 364);

                    centroidObj = centroid_Green;
                    distanceGripper = minDist;   // write a function to get a distance real from Intel Realsense
                    std::cout << "Closest Point: (" << closestPoint.x << ", " << closestPoint.y << ")" << std::endl;
                    std::cout << "Fartherst Point: (" << farthestPoint.x << ", " << farthestPoint.y << ")" << std::endl;
                    std::cout << "Centroid of Object: (" << centroidObj.x << ", " << centroidObj.y << ")" << std::endl;
                    std::cout << "distanceGripper of Object: "<< distanceGripper << std::endl;

                    // Check if both points of each line are in the third or fourth quadrant
                    // Check if points are in the fourth quadrant
                    bool isQuadrant3or4 = (closestPoint.y < centroid_Green.y && closestPoint.x < centroid_Green.x) ||
                                            (farthestPoint.y > centroid_Green.y && farthestPoint.x > centroid_Green.x);

                    // Check if points are in the third quadrant
                    bool isQuadrant3or4_line2 = (closestPoint.y > centroid_Green.y && closestPoint.x < centroid_Green.x) ||
                                           (farthestPoint.y < centroid_Green.y && farthestPoint.x > centroid_Green.x);

                    if (isQuadrant3or4 || isQuadrant3or4_line2) {
                        // Calculate the slopes of both lines
                        double slope_line1 = static_cast<double>(closestPoint.y - farthestPoint.y) / (closestPoint.x - farthestPoint.x);
                        double slope_line2 = static_cast<double>(point2_line2.y - point1_line2.y) / (point2_line2.x - point1_line2.x);

                        // Calculate the angle between the two lines
                        double angle_rad = std::atan(std::abs((slope_line2 - slope_line1) / (1 + slope_line1 * slope_line2)));

//                        if (isQuadrant3or4) {
//                            angle_rad = CV_PI - angle_rad; // Reflecting third quadrant angle
//                        } else if (isQuadrant3or4_line2) {
//                            angle_rad = -angle_rad; // Reflecting fourth quadrant angle
//                        }
                        // Convert radians to degrees
                        double angle_deg = angle_rad * (180 / CV_PI);
//                        // Adjust the angle to be positive and within the range [0, 180]
                        if (angle_deg < 0) {
                            angle_deg += 180.0;
                        }

                        rtAngle = angle_deg;
                        // Print the angle
                        std::cout << "Angle between the two lines: " << angle_deg << " degrees" << std::endl;

                        prepareGrip = true;
                        qDebug() << "Value of prepareGrip:" << prepareGrip;
                    }
                }
            }
        }



    //----------------------------------------------------------------------------------------------
        for (size_t i = 0; i < contoursYellow.size(); i++)
        {
            double area = cv::contourArea(contoursYellow[i]);
            if (area > 4500) { // Area threshold
                // Calculate centroid
                cv::Moments M = cv::moments(contoursYellow[i]);
                cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);

                // Draw marker at the centroid position
                cv::circle(frame, centroid, 5, cv::Scalar(255, 0, 0), -1);
                // Vẽ đường bao contour của vật
                cv::drawContours(frame, contoursYellow, static_cast<int>(i), cv::Scalar(0, 168, 200), 2);
                // Calculate bounding rectangle
                cv::Rect boundingRect = cv::boundingRect(contoursYellow[i]);

                //-----------------------------Tìm điểm trên contour có khoảng cách ngắn nhất từ tâm--------------
                 double minDist = std::numeric_limits<double>::max(); // Khởi tạo khoảng cách tối thiểu là giá trị lớn nhất
                 cv::Point closestPoint_temp;
                 for (const auto& point : contoursYellow[i]) {
                     double dist = cv::norm(point - centroid); // Tính khoảng cách từ tâm đến điểm trên contour
                     if (dist < minDist) {
                         minDist = dist;
                         closestPoint_temp = point;
                     }
                 }
//                 // Vẽ đường thẳng từ tâm đến điểm trên contour có khoảng cách ngắn nhất
//                 cv::line(frame, centroid, closestPoint, cv::Scalar(0, 168, 200), 2);

                                 //----------
                 // Calculate slope and y-intercept of the line passing through closestPoint and centroid
                 double slope;
                 double y_intercept;
                 cv::Point farthest_point;
                 if (closestPoint_temp.x != centroid.x) {
                     // Tính slope như bình thường
                     slope = static_cast<double>(centroid.y - closestPoint_temp.y) / (centroid.x - closestPoint_temp.x);
                     // Tính y_intercept
                     y_intercept = closestPoint_temp.y - slope * closestPoint_temp.x;

                     // Find the point on the contour that has a y-intercept close to the line and the maximum distance from closestPoint
                     double min_dist = 10; // Set a minimum distance threshold
                     double max_dist = 0;

                     for (const auto& point : contoursYellow[i]) {
                         double y_intercept_diff = point.y - slope * point.x;
                         if (std::abs(y_intercept_diff - y_intercept) < 15) { // Allow for a tolerance of 5 pixels
                             double dist = cv::norm(point - closestPoint_temp);
                             if (dist > min_dist && dist > max_dist) {
                                 max_dist = dist;
                                 farthest_point = point;
                             }
                         }
                     }
                 } else {
                     // Trường hợp closestPoint và centroid thẳng hàng theo hướng dọc
                     slope = std::numeric_limits<double>::infinity(); // Đặt slope bằng vô cực
                     // Tính y_intercept
                     y_intercept = closestPoint_temp.y; // Đặt y_intercept là closestPoint.x

                     double max_dist = 0;

                     for (const auto& point : contoursYellow[i]) {
                         // Kiểm tra xem điểm có trên đường thẳng không (đường thẳng là song song với trục y, nên chỉ cần kiểm tra giá trị x)
                         if (std::abs(point.x - closestPoint_temp.x) < 5) { // Allow for a tolerance of 5 pixels
                             // Tính khoảng cách từ điểm đến centroid
                             double dist = cv::norm(point - centroid);
                             // Kiểm tra nếu dist lớn hơn max_dist, nếu có cập nhật farthest_point và max_dist
                             if (dist > max_dist) {
                                 max_dist = dist;
                                 farthest_point = point;
                             }
                         }
                     }
                 }

                  // Check if farthest_point is not equal to (0, 0)
                  if (farthest_point != cv::Point(0, 0)) {
                      // Draw the farthest_point
                      cv::circle(frame, farthest_point, 5, cv::Scalar(0, 255, 0), -1);
                      cv::line(frame, closestPoint_temp, farthest_point, cv::Scalar(0, 168, 200), 2);
                  }
             //------------------
                // Draw bounding rectangle
//                cv::rectangle(frame, boundingRect, cv::Scalar(0, 168, 200), 2);

                std::cout << "Yellow object detected" << std::endl;

                // Set the object color variable
                objectColor = 1;
                if (centroid.x == 250) {
                    closestPoint = closestPoint_temp;
                    farthestPoint = farthest_point;

                    cv::Point point1_line2(109, 364);
                    cv::Point point2_line2(323, 364);

                    centroidObj = centroid;
                    distanceGripper = minDist;   // write a function to get a distance real from Intel Realsense
                    std::cout << "Closest Point: (" << closestPoint.x << ", " << closestPoint.y << ")" << std::endl;
                    std::cout << "Fartherst Point: (" << farthestPoint.x << ", " << farthestPoint.y << ")" << std::endl;
                    std::cout << "Centroid of Object: (" << centroidObj.x << ", " << centroidObj.y << ")" << std::endl;
                    std::cout << "distanceGripper of Object: "<< distanceGripper << std::endl;

                    // Check if both points of each line are in the third or fourth quadrant
                    // Check if points are in the fourth quadrant
                    bool isQuadrant3or4 = (closestPoint.y < centroid.y && closestPoint.x < centroid.x) ||
                                            (farthestPoint.y > centroid.y && farthestPoint.x > centroid.x);

                    // Check if points are in the third quadrant
                    bool isQuadrant3or4_line2 = (closestPoint.y > centroid.y && closestPoint.x < centroid.x) ||
                                           (farthestPoint.y < centroid.y && farthestPoint.x > centroid.x);

                    if (isQuadrant3or4 || isQuadrant3or4_line2) {
                        // Calculate the slopes of both lines
                        double slope_line1 = static_cast<double>(closestPoint.y - farthestPoint.y) / (closestPoint.x - farthestPoint.x);
                        double slope_line2 = static_cast<double>(point2_line2.y - point1_line2.y) / (point2_line2.x - point1_line2.x);

                        // Calculate the angle between the two lines
                        double angle_rad = std::atan(std::abs((slope_line2 - slope_line1) / (1 + slope_line1 * slope_line2)));

//                        if (isQuadrant3or4) {
//                            angle_rad = CV_PI - angle_rad; // Reflecting third quadrant angle
//                        } else if (isQuadrant3or4_line2) {
//                            angle_rad = -angle_rad; // Reflecting fourth quadrant angle
//                        }
                        // Convert radians to degrees
                        double angle_deg = angle_rad * (180 / CV_PI);
//                        // Adjust the angle to be positive and within the range [0, 180]
                        if (angle_deg < 0) {
                            angle_deg += 180.0;
                        }

                        rtAngle = angle_deg;
                        // Print the angle
                        std::cout << "Angle between the two lines: " << angle_deg << " degrees" << std::endl;

                        prepareGrip = true;
                        qDebug() << "Value of prepareGrip:" << prepareGrip;
                    }
                }

            }
        }

    //---------------------------------------------------------------------------------------------

    for (size_t i = 0; i < contoursRed.size(); i++)
        {
            double area = cv::contourArea(contoursRed[i]);

            if (area > 5000) { // Area threshold
                // Calculate centroid
                cv::Moments M = cv::moments(contoursRed[i]);
                cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);

                // Draw marker at the centroid position
                cv::circle(frame, centroid, 5, cv::Scalar(255, 0, 0), -1);
                cv::drawContours(frame, contoursRed, static_cast<int>(i), cv::Scalar(0, 168, 200), 2);
                // Calculate bounding rectangle
                cv::Rect boundingRect = cv::boundingRect(contoursRed[i]);

                //-----------------------------Tìm điểm trên contour có khoảng cách ngắn nhất từ tâm--------------
                 double minDist = std::numeric_limits<double>::max(); // Khởi tạo khoảng cách tối thiểu là giá trị lớn nhất
                 cv::Point closestPoint_temp;
                 for (const auto& point : contoursRed[i]) {
                     double dist = cv::norm(point - centroid); // Tính khoảng cách từ tâm đến điểm trên contour
                     if (dist < minDist) {
                         minDist = dist;
                         closestPoint_temp = point;
                     }
                 }

                 // Vẽ đường thẳng từ tâm đến điểm trên contour có khoảng cách ngắn nhất
                 cv::line(frame, centroid, closestPoint_temp, cv::Scalar(0, 168, 200), 2);

                 //----------
                 // Calculate slope and y-intercept of the line passing through closestPoint and centroid
                 double slope;
                 double y_intercept;
                 cv::Point farthest_point;
                 if (closestPoint_temp.x != centroid.x) {
                     // Tính slope như bình thường
                     slope = static_cast<double>(centroid.y - closestPoint_temp.y) / (centroid.x - closestPoint_temp.x);
                     // Tính y_intercept
                     y_intercept = closestPoint_temp.y - slope * closestPoint_temp.x;

                     // Find the point on the contour that has a y-intercept close to the line and the maximum distance from closestPoint
                     double min_dist = 10; // Set a minimum distance threshold
                     double max_dist = 0;

                     for (const auto& point : contoursRed[i]) {
                         double y_intercept_diff = point.y - slope * point.x;
                         if (std::abs(y_intercept_diff - y_intercept) < 15) { // Allow for a tolerance of 5 pixels
                             double dist = cv::norm(point - closestPoint_temp);
                             if (dist > min_dist && dist > max_dist) {
                                 max_dist = dist;
                                 farthest_point = point;
                             }
                         }
                     }
                 } else {
                     // Trường hợp closestPoint và centroid thẳng hàng theo hướng dọc
                     slope = std::numeric_limits<double>::infinity(); // Đặt slope bằng vô cực
                     // Tính y_intercept
                     y_intercept = closestPoint_temp.y; // Đặt y_intercept là closestPoint.x

                     double max_dist = 0;

                     for (const auto& point : contoursRed[i]) {
                         // Kiểm tra xem điểm có trên đường thẳng không (đường thẳng là song song với trục y, nên chỉ cần kiểm tra giá trị x)
                         if (std::abs(point.x - closestPoint_temp.x) < 5) { // Allow for a tolerance of 5 pixels
                             // Tính khoảng cách từ điểm đến centroid
                             double dist = cv::norm(point - centroid);
                             // Kiểm tra nếu dist lớn hơn max_dist, nếu có cập nhật farthest_point và max_dist
                             if (dist > max_dist) {
                                 max_dist = dist;
                                 farthest_point = point;
                             }
                         }
                     }
                 }

                  // Check if farthest_point is not equal to (0, 0)
                  if (farthest_point != cv::Point(0, 0)) {
                      // Draw the farthest_point
                      cv::circle(frame, farthest_point, 5, cv::Scalar(0, 255, 0), -1);
                      cv::line(frame, closestPoint_temp, farthest_point, cv::Scalar(0, 168, 200), 2);
                  }
             //---------------------------------------------------------------------
                cv::rectangle(frame, boundingRect, cv::Scalar(0, 168, 200), 2);
                std::cout << "Red object detected" << std::endl;


                // Set the object color variable
                objectColor = 0;
                if (centroid.x == 250) {
                    closestPoint = closestPoint_temp;
                    farthestPoint = farthest_point;

                    cv::Point point1_line2(109, 364);
                    cv::Point point2_line2(323, 364);

                    centroidObj = centroid;
                    distanceGripper = minDist;   // write a function to get a distance real from Intel Realsense
                    std::cout << "Closest Point: (" << closestPoint.x << ", " << closestPoint.y << ")" << std::endl;
                    std::cout << "Fartherst Point: (" << farthestPoint.x << ", " << farthestPoint.y << ")" << std::endl;
                    std::cout << "Centroid of Object: (" << centroidObj.x << ", " << centroidObj.y << ")" << std::endl;
                    std::cout << "distanceGripper of Object: "<< distanceGripper << std::endl;

                    // Check if both points of each line are in the third or fourth quadrant
                    // Check if points are in the fourth quadrant
                    bool isQuadrant3or4 = (closestPoint.y < centroid.y && closestPoint.x < centroid.x) ||
                                            (farthestPoint.y > centroid.y && farthestPoint.x > centroid.x);

                    // Check if points are in the third quadrant
                    bool isQuadrant3or4_line2 = (closestPoint.y > centroid.y && closestPoint.x < centroid.x) ||
                                           (farthestPoint.y < centroid.y && farthestPoint.x > centroid.x);

                    if (isQuadrant3or4 || isQuadrant3or4_line2) {
                        // Calculate the slopes of both lines
                        double slope_line1 = static_cast<double>(closestPoint.y - farthestPoint.y) / (closestPoint.x - farthestPoint.x);
                        double slope_line2 = static_cast<double>(point2_line2.y - point1_line2.y) / (point2_line2.x - point1_line2.x);

                        // Calculate the angle between the two lines
                        double angle_rad = std::atan(std::abs((slope_line2 - slope_line1) / (1 + slope_line1 * slope_line2)));

//                        if (isQuadrant3or4) {
//                            angle_rad = CV_PI - angle_rad; // Reflecting third quadrant angle
//                        } else if (isQuadrant3or4_line2) {
//                            angle_rad = -angle_rad; // Reflecting fourth quadrant angle
//                        }
                        // Convert radians to degrees
                        double angle_deg = angle_rad * (180 / CV_PI);
//                        // Adjust the angle to be positive and within the range [0, 180]
                        if (angle_deg < 0) {
                            angle_deg += 180.0;
                        }

                        // Print the angle
                        rtAngle = angle_deg;
                        std::cout << "Angle between the two lines: " << angle_deg << " degrees" << std::endl;

                        prepareGrip = true;
                        qDebug() << "Value of prepareGrip:" << prepareGrip;
                    }
                }
            }
        }

//    std::cout << "Object color at videoProc: " << objectColor << std::endl;
    return frame;
}
*/
void capture::caculatePosCam2Base(float point_in_camera_space[3], cv::Mat& t_result)
{
        cv::Mat pos_homogeneous             = (cv::Mat_<double>(4, 1) <<
                                                 point_in_camera_space[0]*1000,
                                                 point_in_camera_space[1]*1000,
                                                 point_in_camera_space[2]*1000,
                                                 1);

        cv::Mat result = homogeneous_matrix * pos_homogeneous;

        // Trích xuất phần dịch chuyển t từ result
        t_result = result(cv::Rect(0, 0, 1, 3));    //lay 3 vi tri x,y,z tu t_result
        // Hiển thị kết quả

//        std::cout << "Translation Result (x, y, z): "
//                  << t_result.at<double>(0, 0) << ", "
//                  << t_result.at<double>(1, 0) << ", "
//                  << t_result.at<double>(2, 0) << std::endl;
}


//--------------------------------------------Code test ------------------------------------------
void capture::testCode(cv::Mat& t_result){
    cv::Mat Test = (cv::Mat_<double>(3, 1) << -0.0284299*1000, -0.0233816*1000, 0.281*1000);

    cv::Mat Test_homogeneous    = (cv::Mat_<double>(4, 1) <<
                                                             Test.at<double>(0, 0),
                                                             Test.at<double>(1, 0),
                                                             Test.at<double>(2, 0),
                                                             1);

    cv::Mat result = homogeneous_matrix * Test_homogeneous;

        // Trích xuất phần dịch chuyển t từ result
    t_result = result(cv::Rect(0, 0, 1, 3));    //lay 3 vi tri x,y,z tu t_result

        // Hiển thị kết quả
//        std::cout << "Helo world";
//        qDebug() <<"helo world qdebug";
//        // Print or debug output
//        std::cout << " t_cam2base matrix:\n" <<  t_cam2base << std::endl;
//        std::cout << "Test matrix:\n" << Test << std::endl;
//        std::cout << "Homogeneous matrix:\n" << homogeneous_matrix << std::endl;
//        std::cout << "Test_homogeneous matrix:\n" << Test_homogeneous << std::endl;
//        std::cout << "Result matrix:\n" << result << std::endl;
    std::cout << "Translation Result (x, y, z): "
              << t_result.at<double>(0, 0) << ", "
              << t_result.at<double>(1, 0) << ", "
              << t_result.at<double>(2, 0) << std::endl;
    //----------------------------------------------

}

//---------------------------------------Rewrite the code DetectObj------------------------------
cv::Mat capture::objDetect(const cv::Mat& frame, cv::Point &center)
{

    std::vector<std::pair<cv::Scalar, cv::Scalar>> colorRanges = {

        {cv::Scalar(9, 201, 150),   cv::Scalar(23, 255, 255)},   // Orange
        {cv::Scalar(6, 197, 131),   cv::Scalar(27, 255, 255)},   // Orange
//        {cv::Scalar(10, 178, 111),   cv::Scalar(27, 255, 255)},   // Orange
//        {cv::Scalar(38, 79, 61), cv::Scalar(56, 231, 159)},    // Green
        {cv::Scalar(42, 98, 106), cv::Scalar(68, 170, 204)},    // Green
        {cv::Scalar(42, 90, 125), cv::Scalar(68, 180, 191)},    // Green

        {cv::Scalar(20, 129, 100), cv::Scalar(31, 255, 211)},  // Yellow

        {cv::Scalar(0, 102, 100), cv::Scalar(10, 255, 219)},   // Red (lower range)
        {cv::Scalar(160, 102, 100), cv::Scalar(179, 255, 219)} // Red (upper range)
    };

    std::vector<std::string> colorNames = {"Orange", "Orange", "Green", "Green", "Yellow", "Red", "Red"};


    cv::Mat colorFrameHSV;
    cv::cvtColor(frame, colorFrameHSV, cv::COLOR_BGR2HSV);

    cv::Mat maskAll = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<cv::Mat> masks;

    for (const auto& range : colorRanges)
    {
        cv::Mat mask;
        cv::inRange(colorFrameHSV, range.first, range.second, mask);
        maskAll |= mask;
        masks.push_back(mask);
    }

    int kernelSize = 7;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::erode(maskAll, maskAll, kernel);
    cv::dilate(maskAll, maskAll, kernel);

    std::vector<std::vector<cv::Point>> contoursAll;
    cv::findContours(maskAll, contoursAll, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//    qDebug() << "contours size " << contoursAll.size();
    if(contoursAll.size() == 1)
    {
        coVat = false;
    }
    else {
        coVat = true;
    }

    for (size_t j = 0; j < masks.size(); ++j)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(masks[j], contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours)
        {
            double area = cv::contourArea(contour);
//            std::cout << "contour Area: " << area <<std::endl;
            if (area > 5000)
            {                                   //co bao co vat the
                cv::Moments M = cv::moments(contour);
                cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);
//                qDebug() << " centroid value at objectDetect: " << centroid.x ;
//                cv::circle(frame, centroid, 5, cv::Scalar(255, 0, 0), -1);
                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 168, 200), 2);
                cv::Rect boundingRect = cv::boundingRect(contour);

                double minDist = std::numeric_limits<double>::max();
                cv::Point closestPoint_temp;
                for (const auto& point : contour)
                {
                    double dist = cv::norm(point - centroid);
                    if (dist < minDist)
                    {
                        minDist = dist;
                        closestPoint_temp = point;
                    }
                }

                double slope, y_intercept;
                cv::Point farthest_point;
                if (closestPoint_temp.x != centroid.x)
                {
                    slope = static_cast<double>(centroid.y - closestPoint_temp.y) / (centroid.x - closestPoint_temp.x);
                    y_intercept = closestPoint_temp.y - slope * closestPoint_temp.x;

                    double min_dist = 10;
                    double max_dist = 0;

                    for (const auto& point : contour)
                    {
                        double y_intercept_diff = point.y - slope * point.x;
                        if (std::abs(y_intercept_diff - y_intercept) < 15)
                        {
                            double dist = cv::norm(point - closestPoint_temp);
                            if (dist > min_dist && dist > max_dist)
                            {
                                max_dist = dist;
                                farthest_point = point;
                            }
                        }
                    }
                }
                else
                {
                    slope = std::numeric_limits<double>::infinity();
                    y_intercept = closestPoint_temp.y;

                    double max_dist = 0;

                    for (const auto& point : contour)
                    {
                        if (std::abs(point.x - closestPoint_temp.x) < 5)
                        {
                            double dist = cv::norm(point - centroid);
                            if (dist > max_dist)
                            {
                                max_dist = dist;
                                farthest_point = point;
                            }
                        }
                    }
                }

                if (farthest_point != cv::Point(0, 0))
                {
                    cv::circle(frame, farthest_point, 5, cv::Scalar(0, 255, 0), -1);
                    cv::line(frame, closestPoint_temp, farthest_point, cv::Scalar(0, 168, 200), 2);
                }

                cv::rectangle(frame, boundingRect, cv::Scalar(0, 168, 200), 2);

//                std::cout << colorNames[j] << " object detected" << std::endl;
//                centroidObj = centroid;
                center = centroid;
                centroiVector.push_back(centroid);
                qDebug() << " centroid value: " << centroid.x ;
//                qDebug() << " center value: "   << center.x ;
                //-----------------------------------------------------vị trí pixel lấy thông tin obj để chuẩn bị gắp vật---------------------//
//                if ( centroid.x > 320 && centroid.x < 326 )
//                {
//                    isObj = true;
//                }

                if (colorNames[j] == "Orange")
                {
                    this->objectColor = 0;
//                    std::cout << "Detected an Orange object." << std::endl;
                }
                else if (colorNames[j] == "Green")
                {
                    this->objectColor = 1;
//                    std::cout << "Detected a Green object." << std::endl;
                }
                else if (colorNames[j] == "Yellow")
                {
                    this->objectColor = 2;
//                    std::cout << "Detected a Yellow object." << std::endl;
                }
                else if (colorNames[j] == "Red")
                {
                    this->objectColor = 3;
//                    std::cout << "Detected a Red object." << std::endl;
                }
                if (centroid.x > 326 && centroid.x < 347)
                {
//                    std::cout << " centroid value at pixel 250: " << centroid.x << std::endl;
//                    objectInPos = true;

                    closestPoint = closestPoint_temp;
                    farthestPoint = farthest_point;

                    cv::Point point1_line2(109, 364);
                    cv::Point point2_line2(323, 364);

//                    centroidObj = centroid;
                    distanceGripper = minDist;

//                    std::cout << "Closest Point: (" << closestPoint.x << ", " << closestPoint.y << ")" << std::endl;
//                    std::cout << "Farthest Point: (" << farthestPoint.x << ", " << farthestPoint.y << ")" << std::endl;
//                    std::cout << "Centroid of Object: (" << centroidObj.x << ", " << centroidObj.y << ")" << std::endl;
//                    std::cout << "Distance to Gripper: " << distanceGripper << std::endl;

                    bool isQuadrant3or4 = (closestPoint.y < centroid.y && closestPoint.x < centroid.x) ||
                                          (farthestPoint.y > centroid.y && farthestPoint.x > centroid.x);

                    bool isQuadrant3or4_line2 = (closestPoint.y > centroid.y && closestPoint.x < centroid.x) ||
                                                (farthestPoint.y < centroid.y && farthestPoint.x > centroid.x);

                    if (isQuadrant3or4 || isQuadrant3or4_line2)
                    {
                        double slope_line1 = static_cast<double>(closestPoint.y - farthestPoint.y) / (closestPoint.x - farthestPoint.x);
                        double slope_line2 = static_cast<double>(point2_line2.y - point1_line2.y) / (point2_line2.x - point1_line2.x);

                        double angle_rad = std::atan(std::abs((slope_line2 - slope_line1) / (1 + slope_line1 * slope_line2)));

                        double angle_deg = angle_rad * (180 / CV_PI);
//                        if (angle_deg < 0)
//                        {
//                            angle_deg += 180.0;
//                        }
                        // Xử lý góc âm nếu cần thiết

//                        if (slope_line2 < slope_line1) {
//                            angle_deg = -angle_deg;  // Đảo ngược dấu góc nếu đường thẳng thứ hai nằm bên trái của đường thẳng đầu tiên
//                        }

                        rtAngle = angle_deg;
                        qDebug() << "Angle between the two lines: " << rtAngle;
                        std::cout << "Angle between the two lines: " << angle_deg << " degrees" << std::endl;

                        prepareGrip = true;
//                        qDebug() << "Value of prepareGrip:" << prepareGrip;
                    }

                }
            }
        }
    }

    return frame;
}


cv::Mat capture::objDetect_1(const cv::Mat& frameInput, cv::Point &center) {
    // Define the ROI (Region of Interest)
    cv::Rect roiRect(300, 0, 340, 480); // x, y, width, height
    cv::Mat frame = frameInput.clone(); // Clone the input frame to keep the original size

    // Create a mask of zeros (same size as the original frame)
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);

    // Set the ROI part of the mask to 255 (indicating the area to be processed)
    mask(roiRect).setTo(255);

    // Convert the frame to HSV
    cv::Mat colorFrameHSV;
    cv::cvtColor(frame, colorFrameHSV, cv::COLOR_BGR2HSV);

    // Create a final mask that combines all color ranges
    cv::Mat maskAll = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<cv::Mat> masks;
    std::vector<std::pair<cv::Scalar, cv::Scalar>> colorRanges = {
        {cv::Scalar(6, 212, 128),   cv::Scalar(17, 255, 206)},   // Orange
        {cv::Scalar(38, 79, 61),    cv::Scalar(56, 231, 159)},   // Green
        {cv::Scalar(20, 129, 100),  cv::Scalar(31, 255, 211)},   // Yellow
        {cv::Scalar(0, 102, 100),   cv::Scalar(10, 255, 219)},   // Red (lower range)
        {cv::Scalar(160, 102, 100), cv::Scalar(179, 255, 219)}   // Red (upper range)
    };
    std::vector<std::string> colorNames = {"Orange", "Green", "Yellow", "Red", "Red"};

    for (const auto& range : colorRanges) {
        cv::Mat maskColor;
        cv::inRange(colorFrameHSV, range.first, range.second, maskColor);
        maskAll |= maskColor;
        masks.push_back(maskColor);
    }

    // Apply the mask to restrict processing to the ROI
    cv::Mat maskedAll;
    maskAll.copyTo(maskedAll, mask);

    // Apply morphological operations to clean the mask
    int kernelSize = 7;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    cv::erode(maskedAll, maskedAll, kernel);
    cv::dilate(maskedAll, maskedAll, kernel);

    // Find contours on the restricted area
//    std::vector<std::vector<cv::Point>> contoursAll;
//    cv::findContours(maskedAll, contoursAll, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t j = 0; j < masks.size(); ++j) {
        cv::Mat maskedColor;
        masks[j].copyTo(maskedColor, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(maskedColor, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 5000) {
                cv::Moments M = cv::moments(contour);
                cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);

                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 168, 200), 2);
                cv::Rect boundingRect = cv::boundingRect(contour);

                double minDist = std::numeric_limits<double>::max();
                cv::Point closestPoint_temp;
                for (const auto& point : contour) {
                    double dist = cv::norm(point - centroid);
                    if (dist < minDist) {
                        minDist = dist;
                        closestPoint_temp = point;
                    }
                }

                double slope, y_intercept;
                cv::Point farthest_point;
                if (closestPoint_temp.x != centroid.x) {
                    slope = static_cast<double>(centroid.y - closestPoint_temp.y) / (centroid.x - closestPoint_temp.x);
                    y_intercept = closestPoint_temp.y - slope * closestPoint_temp.x;

                    double max_dist = 0;
                    for (const auto& point : contour) {
                        double y_intercept_diff = point.y - slope * point.x;
                        if (std::abs(y_intercept_diff - y_intercept) < 15)
                        {
                            double dist = cv::norm(point - closestPoint_temp);
                            if (dist > max_dist)
                            {
                                max_dist = dist;
                                farthest_point = point;
                            }
                        }
                    }
                } else {
                    slope = std::numeric_limits<double>::infinity();
                    y_intercept = closestPoint_temp.y;

                    double max_dist = 0;
                    for (const auto& point : contour) {
                        if (std::abs(point.x - closestPoint_temp.x) < 5)
                        {
                            double dist = cv::norm(point - centroid);
                            if (dist > max_dist)
                            {
                                max_dist = dist;
                                farthest_point = point;
                            }
                        }
                    }
                }

                if (farthest_point != cv::Point(0, 0))
                {
                    cv::circle(frame, farthest_point, 5, cv::Scalar(0, 255, 0), -1);
//                    cv::line(frame, closestPoint_temp, farthest_point, cv::Scalar(0, 168, 200), 2);
                }

                cv::rectangle(frame, boundingRect, cv::Scalar(0, 168, 200), 2);

                center = centroid;
                if (colorNames[j] == "Orange") {
                    this->objectColor = 0;
                } else if (colorNames[j] == "Green") {
                    this->objectColor = 1;
                } else if (colorNames[j] == "Yellow") {
                    this->objectColor = 2;
                } else if (colorNames[j] == "Red") {
                    this->objectColor = 3;
                }

                if (centroid.x > 220 && centroid.x < 640) {
                    closestPoint = closestPoint_temp;
                    farthestPoint = farthest_point;
                    distanceGripper = minDist;

                    cv::Point point1_line2(109, 364);
                    cv::Point point2_line2(323, 364);

                    bool isQuadrant3or4 = (closestPoint.y < centroid.y && closestPoint.x < centroid.x) ||
                                          (farthestPoint.y > centroid.y && farthestPoint.x > centroid.x);

                    bool isQuadrant3or4_line2 = (closestPoint.y > centroid.y && closestPoint.x < centroid.x) ||
                                                (farthestPoint.y < centroid.y && farthestPoint.x > centroid.x);

                    if (isQuadrant3or4 || isQuadrant3or4_line2) {
                        double slope_line1 = static_cast<double>(closestPoint.y - farthestPoint.y) / (closestPoint.x - farthestPoint.x);
                        double slope_line2 = static_cast<double>(point2_line2.y - point1_line2.y) / (point2_line2.x - point1_line2.x);

                        double angle_rad = std::atan(std::abs((slope_line2 - slope_line1) / (1 + slope_line1 * slope_line2)));
                        double angle_deg =  (angle_rad * (180 / CV_PI) ) + 15.00;
                        if (slope_line2 < slope_line1) {
                            angle_deg = -angle_deg;
                        }
                        rtAngle = angle_deg;
                        prepareGrip = true;
                    }
                }
            }
        }
    }

    return frame;
}

















