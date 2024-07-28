#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "iostream"
#include "queue"
#include <cmath>

#include <QCoreApplication>
#include <QEventLoop>
#include <QTime>
#include <QPixmap>
#include <QThread>
#include <QTimer>

#include <QMessageBox>
#include <QSerialPortInfo>

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/ccalib.hpp"

#include "udp.h"
#include "yrc1000micro_com.h"
#include "yrc1000micro_command.h"
#include "capture.h"
#include "serialport.h"
#include "caculatematrix.h"


#include <vector>
#include <algorithm>

#define windowSize 10



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void init_window();
    QVector<double> UpdatePos();
    double speedAverageValue = 0.0;

    void updateLabel(int value);
    void connectSliderData();

    void on_btnOpenPort_clicked();
    void loadPorts();
    void readData(QByteArray data);

    //-----------------------
    bool takeParamCamera = false;
    //-----------------------
    void captureThePictureAndData(int imageIndex);

    double calculateMovingAverage();
    void handleReceivedData(double newData);
    void grippObj();
    void openGripper();
    void delay(int n);
    void captureObjectImage();
    int findMaxX(const std::vector<cv::Point> &points);
private slots:
    void display(QPixmap pixmap);
    void autoRunSystem();

    void on_btn_connect_clicked();

    void on_btn_home_clicked();

    void on_btn_servo_clicked();

    void on_btn_set_pos_clicked();

    void on_movi_btn_clicked();

    void on_movj_btn_clicked();

    void on_movl_btn_clicked();

    void on_btn_get_pos_clicked();

    void on_btn_add_X_clicked();

    void on_btn_add_Y_clicked();

    void on_btn_add_Z_clicked();

    void on_btn_sub_X_clicked();

    void on_btn_sub_Y_clicked();

    void on_btn_sub_Z_clicked();

    void on_btn_add_Roll_clicked();

    void on_btn_add_Ptch_clicked();

    void on_btn_add_Yaw_clicked();

    void on_btn_sub_Roll_clicked();

    void on_btn_sub_Ptch_clicked();

    void on_btn_sub_Yaw_clicked();

    void on_load_Job_butt_clicked();

    void on_start_Job_butt_clicked();

    void on_openCam_but_clicked();

    void on_btnSend_clicked();

    void on_btn_open_port_clicked();

    void on_capture_btn_clicked();

    void on_detect_btn_clicked();

    void on_btn_cal_clicked();

    void on_savepicObject_btn_clicked();

    void on_btn_auto_clicked();

    void on_test_fnc_btn_clicked();

    void on_reset_Sys_butt_clicked();

    void on_selShowImage_activated(int index);

private:
    Ui::MainWindow *ui;
    YRC1000micro_com *yrc1000micro_com;
    capture *videoCapture;
    serialPort _port;
    caculateMatrix *maTrix;

    QPixmap getPixmap;

    bool runSystem = false;
    QTimer *runTimer;
    QVector<double> robot_Pos;

    std::queue<double> receivedDataQueue;

    bool done = false;

    int maxX;
    uint8_t orangeCount = 0;
    uint8_t greenCount = 0;
    uint8_t yellowCount = 0;
    uint8_t redCount = 0;

//    bool stop = false;      // biến bool để tắt while khi tắt GUI bằng dấu X mà không toogle AutoRun

};
#endif // MAINWINDOW_H
