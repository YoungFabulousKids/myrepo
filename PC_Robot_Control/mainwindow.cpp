#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    init_window();
    yrc1000micro_com = new YRC1000micro_com(this);
    videoCapture = new capture(this);
    maTrix = new caculateMatrix(this);

    runTimer = new QTimer(this);
    runTimer->setInterval(800);

    loadPorts();
    connect(&_port, &serialPort::dataReceive, this, &MainWindow::readData);
    connect(runTimer, &QTimer::timeout, this, &MainWindow::autoRunSystem);
    connectSliderData();
}

MainWindow::~MainWindow()
{
//    stop = true;
    delete ui;
    delete yrc1000micro_com;
    delete videoCapture;
    delete &_port;
    delete maTrix;
    delete runTimer;

}
void MainWindow::init_window()
{
    QImage image;
    image.load("../Image/No_signal.png");
    ui->Image->setPixmap(QPixmap::fromImage(image));
    QIcon icon;
    icon.addFile(QString::fromUtf8("../Image/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
    MainWindow::setWindowIcon(icon);
}
//--------------------------------------------Connection Group------------------------------------------------------------------

void MainWindow::on_btn_connect_clicked()
{
    if (ui->btn_connect->text() == "Connect"){
        QHostAddress udp_address;
        quint16 udp_port;
        QString ip_string = ui->txt_ip->text();
        QStringList ip_list = ip_string.split(".");
        quint32 ip_int32 = (ip_list.at(0).toUInt() << 24) | (ip_list.at(1).toUInt() << 16) |
                             (ip_list.at(2).toUInt() << 8) | ip_list.at(3).toUInt();

        udp_address.setAddress(ip_int32);
        udp_port = ui->txt_port->text().toUShort();

        yrc1000micro_com->YRC1000microSetConnection(udp_address,udp_port);
        ui->btn_connect->setText("Disconnect");
        //qDebug() << ip_string;
    }
    else if(ui->btn_connect->text() == "Disconnect"){
        ui->btn_connect->setText("Connect");
        yrc1000micro_com->YRC1000microDisConnect();
        //qDebug() << ui->txt_port->text();
    }
}
//--------------------------------------------Basic Control Group-----------------------------------------------------------------

void MainWindow::on_btn_servo_clicked()
{
    if(ui->btn_servo->text() == "Servo On"){
        yrc1000micro_com->YRC1000microOnServo();
//        yrc1000micro_com->YRC1000microDataCallback();
        ui->btn_servo->setText("Servo Off");
    }
    else if(ui->btn_servo->text() == "Servo Off"){
        yrc1000micro_com->YRC1000microOffServo();
        ui->btn_servo->setText("Servo On");
    }
}

void MainWindow::on_btn_home_clicked()
{
    //    yrc1000micro_com->YRC1000microHomePos();
        // or use the second way
        double set_speed = 50;
        QVector<double> set_position;

        set_position.append(185);
        set_position.append(0);
        set_position.append(25);
        set_position.append(180);
        set_position.append(0);
        set_position.append(0);

        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);

}
//--------------------------------------------Set Position Group------------------------------------------------------------------

void MainWindow::on_btn_set_pos_clicked()
{
    double set_speed = ui->txt_setSpeed->text().toDouble();
    QVector<double> set_position;

    set_position.append(ui->txt_setX->text().toDouble());
    set_position.append(ui->txt_setY->text().toDouble());
    set_position.append(ui->txt_setZ->text().toDouble());
    set_position.append(ui->txt_setRoll->text().toDouble());
    set_position.append(ui->txt_setPitch->text().toDouble());
    set_position.append(ui->txt_setYaw->text().toDouble());

    if (ui->movj_btn->isChecked())
    {

        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                    CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
    }
    else if(ui->movi_btn->isChecked())
    {
//        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_ABSOLUTE,
//                                                    CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                    CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
    }
    else if(ui->movl_btn->isChecked())
    {
//        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,
//                                                    CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                    CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
    }

}

void MainWindow::on_movi_btn_clicked()
{
    ui->movj_btn->setChecked(false);
    ui->movl_btn->setChecked(false);
}

void MainWindow::on_movj_btn_clicked()
{
    ui->movi_btn->setChecked(false);
    ui->movl_btn->setChecked(false);
}

void MainWindow::on_movl_btn_clicked()
{
    ui->movj_btn->setChecked(false);
    ui->movi_btn->setChecked(false);
}
//--------------------------------------------Get Position Group------------------------------------------------------------------

QVector<double> MainWindow::UpdatePos()
{
    qDebug() << "Test UpdatePos";
    yrc1000micro_com->YRC1000ReadPosition();
    yrc1000micro_com->YRC1000microDataCallback();
    robot_Pos = yrc1000micro_com->updateRobotPosition();

    qDebug() << "here robot pos: " << robot_Pos;
    return robot_Pos;
}
void MainWindow::on_btn_get_pos_clicked()
{
    UpdatePos();

    qDebug() << "test data of postion: robot_Pos" << robot_Pos;

//    robot_Pos= {124,124,578,64,158,45748};
    ui->txt_get_PosX    ->setText(QString::number(robot_Pos.at(0)));
    ui->txt_get_PosY    ->setText(QString::number(robot_Pos.at(1)));
    ui->txt_get_PosZ    ->setText(QString::number(robot_Pos.at(2)));
    ui->txt_get_Roll    ->setText(QString::number(robot_Pos.at(3)));
    ui->txt_get_Pitch   ->setText(QString::number(robot_Pos.at(4)));
    ui->txt_get_Yaw     ->setText(QString::number(robot_Pos.at(5)));
}
//--------------------------------------------Incremental Position Group------------------------------------------------------------------
void MainWindow::updateLabel(int value) {

    QSlider *slider = qobject_cast<QSlider*>(sender());
    if (slider) {
        QString text = QString::number(value);
        // Determine which label corresponds to the slider and update its text
        if (slider == ui->horizontalSlider)
            ui->delta_dis_value->setText(text);
        else if (slider == ui->horizontalSlider_2)
            ui->delta_deg_value->setText(text);
        else if (slider == ui->horizontalSlider_4)
            ui->speed_dis_value->setText(text);
        else if (slider == ui->horizontalSlider_3)
            ui->speed_deg_value->setText(text);
    }
}

void MainWindow::connectSliderData(){
    connect(ui->horizontalSlider, &QSlider::valueChanged, this, &MainWindow::updateLabel);
    connect(ui->horizontalSlider_2, &QSlider::valueChanged, this, &MainWindow::updateLabel);
    connect(ui->horizontalSlider_4, &QSlider::valueChanged, this, &MainWindow::updateLabel);
    connect(ui->horizontalSlider_3, &QSlider::valueChanged, this, &MainWindow::updateLabel);

}

void MainWindow::on_btn_add_X_clicked()
{
    UpdatePos();

//    double posX     = robot_Pos.at(0);
//    double posY     = robot_Pos.at(1);
//    double posZ     = robot_Pos.at(2);
//    double posRoll  = robot_Pos.at(3);
//    double posPitch = robot_Pos.at(4);
//    double posYaw   = robot_Pos.at(5);

    double deltaPos = ui->delta_dis_value->text().toDouble();
    double deltaSpeed = ui->speed_dis_value->text().toDouble();
    qDebug()<< "pos : "<< deltaPos;

    double set_speed = deltaSpeed;

//    double set_speed = ui->txt_setSpeed->text().toDouble();
    QVector<double> set_position = robot_Pos;

        set_position[0] += deltaPos;

    qDebug()<< "new pos : "<< set_position;

    yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
    on_btn_get_pos_clicked();

}

void MainWindow::on_btn_add_Y_clicked()
{
    UpdatePos();

    double deltaPos = ui->delta_dis_value->text().toDouble();
    double deltaSpeed = ui->speed_dis_value->text().toDouble();
    qDebug()<< "pos : "<< deltaPos;

    double set_speed = deltaSpeed;

//    double set_speed = ui->txt_setSpeed->text().toDouble();
    QVector<double> set_position = robot_Pos;

        set_position[1] += deltaPos;

    qDebug()<< "new pos : "<< set_position;

        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
        on_btn_get_pos_clicked();
}

void MainWindow::on_btn_add_Z_clicked()
{
    UpdatePos();

    double deltaPos = ui->delta_dis_value->text().toDouble();
    double deltaSpeed = ui->speed_dis_value->text().toDouble();
    qDebug()<< "pos : "<< deltaPos;

    double set_speed = deltaSpeed;

//    double set_speed = ui->txt_setSpeed->text().toDouble();
    QVector<double> set_position = robot_Pos;

        set_position[2] += deltaPos;

    qDebug()<< "new pos : "<< set_position;

        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
        on_btn_get_pos_clicked();
}

void MainWindow::on_btn_sub_X_clicked()
{
    UpdatePos();

//    double posX     = robot_Pos.at(0);
//    double posY     = robot_Pos.at(1);
//    double posZ     = robot_Pos.at(2);
//    double posRoll  = robot_Pos.at(3);
//    double posPitch = robot_Pos.at(4);
//    double posYaw   = robot_Pos.at(5);

    double deltaPos = ui->delta_dis_value->text().toDouble();
    double deltaSpeed = ui->speed_dis_value->text().toDouble();
    qDebug()<< "pos : "<< deltaPos;

    double set_speed = deltaSpeed;

//    double set_speed = ui->txt_setSpeed->text().toDouble();
    QVector<double> set_position = robot_Pos;

        set_position[0] -= deltaPos;

    qDebug()<< "new pos : "<< set_position;

        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
        on_btn_get_pos_clicked();
}

void MainWindow::on_btn_sub_Y_clicked()
{
    UpdatePos();

    double deltaPos = ui->delta_dis_value->text().toDouble();
    double deltaSpeed = ui->speed_dis_value->text().toDouble();
    qDebug()<< "pos : "<< deltaPos;

//    double set_speed = ui->txt_setSpeed->text().toDouble();
    double set_speed = deltaSpeed;
    QVector<double> set_position = robot_Pos;

        set_position[1] -= deltaPos;

    qDebug()<< "new pos : "<< set_position;

        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
    on_btn_get_pos_clicked();
}

void MainWindow::on_btn_sub_Z_clicked()
{
    UpdatePos();

    double deltaPos = ui->delta_dis_value->text().toDouble();
    double deltaSpeed = ui->speed_dis_value->text().toDouble();
    qDebug()<< "pos : "<< deltaPos;

    double set_speed = deltaSpeed;

//    double set_speed = ui->txt_setSpeed->text().toDouble();
    QVector<double> set_position = robot_Pos;

        set_position[2] -= deltaPos;

    qDebug()<< "new pos : "<< set_position;

        yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
    on_btn_get_pos_clicked();
}

void MainWindow::on_btn_add_Roll_clicked()
{
    UpdatePos();

    double deltaAngle = ui->delta_deg_value->text().toDouble();
    double deltaSpeedAngle = ui->speed_deg_value->text().toDouble();

    qDebug()<< "pos : "<< deltaAngle;
    double set_speed = deltaSpeedAngle;
    QVector<double> set_position= robot_Pos;

    set_position[3] += deltaAngle;

    qDebug()<< "new pos : "<< set_position;

    yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);

    on_btn_get_pos_clicked();
}

void MainWindow::on_btn_add_Ptch_clicked()
{
    UpdatePos();

    double deltaAngle = ui->delta_deg_value->text().toDouble();
    double deltaSpeedAngle = ui->speed_deg_value->text().toDouble();

    qDebug()<< "pos : "<< deltaAngle;

    double set_speed = deltaSpeedAngle;
    QVector<double> set_position= robot_Pos;

    set_position[4] += deltaAngle;

    qDebug()<< "new pos : "<< set_position;

    yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);

    on_btn_get_pos_clicked();
}

void MainWindow::on_btn_add_Yaw_clicked()
{
    UpdatePos();

//    double posX     = robot_Pos.at(0);
//    double posY     = robot_Pos.at(1);
//    double posZ     = robot_Pos.at(2);
//    double posRoll  = robot_Pos.at(3);
//    double posPitch = robot_Pos.at(4);
//    double posYaw   = robot_Pos.at(5);

//    double deltaPos = ui->delta_dis_value->text().toDouble();
//    double deltaSpeed = ui->speed_dis_value->text().toDouble();

    double deltaAngle = ui->delta_deg_value->text().toDouble();

    double deltaSpeedAngle = ui->speed_deg_value->text().toDouble();

    qDebug()<< "pos : "<< deltaAngle;

//    double set_speed = ui->txt_setSpeed->text().toDouble();
    double set_speed = deltaSpeedAngle;
    QVector<double> set_position= robot_Pos;

        set_position[5] += deltaAngle;

    qDebug()<< "new pos : "<< set_position;

    yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);

    on_btn_get_pos_clicked();
}

void MainWindow::on_btn_sub_Roll_clicked()
{
    UpdatePos();

    double deltaAngle = ui->delta_deg_value->text().toDouble();
    double deltaSpeedAngle = ui->speed_deg_value->text().toDouble();

    qDebug()<< "pos : "<< deltaAngle;

    double set_speed = deltaSpeedAngle;
    QVector<double> set_position= robot_Pos;

    set_position[3] -= deltaAngle;

    qDebug()<< "new pos : "<< set_position;

    yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
}

void MainWindow::on_btn_sub_Ptch_clicked()
{
    UpdatePos();

    double deltaAngle = ui->delta_deg_value->text().toDouble();
    double deltaSpeedAngle = ui->speed_deg_value->text().toDouble();

    qDebug()<< "pos : "<< deltaAngle;

    double set_speed = deltaSpeedAngle;
    QVector<double> set_position= robot_Pos;

    set_position[4] -= deltaAngle;

    qDebug()<< "new pos : "<< set_position;

    yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
    on_btn_get_pos_clicked();
}

void MainWindow::on_btn_sub_Yaw_clicked()
{
    UpdatePos();

//    double posX     = robot_Pos.at(0);
//    double posY     = robot_Pos.at(1);
//    double posZ     = robot_Pos.at(2);
//    double posRoll  = robot_Pos.at(3);
//    double posPitch = robot_Pos.at(4);
//    double posYaw   = robot_Pos.at(5);

//    double deltaPos = ui->delta_dis_value->text().toDouble();
//    double deltaSpeed = ui->speed_dis_value->text().toDouble();

    double deltaAngle = ui->delta_deg_value->text().toDouble();
    double deltaSpeedAngle = ui->speed_deg_value->text().toDouble();

    qDebug()<< "pos : "<< deltaAngle;

//    double set_speed = ui->txt_setSpeed->text().toDouble();
    double set_speed = deltaSpeedAngle;
    QVector<double> set_position= robot_Pos;

    set_position[5] -= deltaAngle;

    qDebug()<< "new pos : "<< set_position;

    yrc1000micro_com->YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                        CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
    on_btn_get_pos_clicked();
}

//--------------------------------------------------------Job Select Function-------------------------------------------------------------------------------------------
void MainWindow::on_load_Job_butt_clicked()
{
    if( (ui->btn_connect->text() == "Disconnect") &&(ui->btn_servo->text() == "Servo Off"))
    {
        QString job = ui->jobFile_text->toPlainText();
        char jobName[12];
        strcpy(jobName, job.toStdString().c_str());
        qDebug() << "jobName: " << jobName;
        yrc1000micro_com->YRC1000microLoadJob(jobName);
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check Connect UDP or Servo State again");
    }
}

void MainWindow::on_start_Job_butt_clicked()
{
    if( (ui->btn_connect->text() == "Disconnect") &&(ui->btn_servo->text() == "Servo Off"))
    {
        yrc1000micro_com->YRC1000microStartJob();
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check Connect UDP or Servo State again");
    }
}
//--------------------------------------------------------Serial Port Function-------------------------------------------------------------------------------------------
void MainWindow::on_btn_open_port_clicked()
{
    qDebug() << "Here open port";
    if (ui -> btn_open_port->text() == "Open")
    {
        auto isConnected = _port.connectPort(ui->cmbPorts->currentText());

        if (!isConnected){
            QMessageBox::critical(this, "Error", "There is a problem connection");
        }
        ui->btn_open_port->setText("Close");
        ui->btn_open_port->setStyleSheet("QPushButton {color: red;}");
    }
    else/* (ui -> btn_open_port->text() == "Close Serial Port")*/
    {
        _port.closePort();
        ui->btn_open_port->setText("Open");
        ui->btn_open_port->setStyleSheet("QPushButton {color: green;}");
    }

}
void MainWindow::loadPorts()
{
    foreach(auto &port, QSerialPortInfo::availablePorts()){
        qDebug()<< port.portName();
        ui->cmbPorts->addItem(port.portName());
    }
}
void MainWindow::readData(QByteArray data)
{
//    ui->lstMessage->addItem(QString(data));
//    // Chuyển đổi dữ liệu sang kiểu số (double)
    QString dataString = QString::fromUtf8(data);
//    qDebug() << "gia tri nhan ve tu uart: "<< dataString;
    double receivedValue = data.toDouble();
//    std::cout << "van toc bang chuyen la: "<< receivedValue << "\n";
    // Gọi hàm để xử lý dữ liệu và tính giá trị trung bình di động
    handleReceivedData(receivedValue);
    // Sau khi xử lý dữ liệu, cập nhật giá trị trung bình di động để sử dụng ở đâu đó trong mã của bạn
    speedAverageValue = calculateMovingAverage();
//    qDebug() << "van toc trung binh la: " << speedAverageValue;
}

void MainWindow::on_btnSend_clicked()
{
    qDebug() << "Herre ";
    // Lấy giá trị từ QLineEdit
    QString message = ui->lnMessage->text();
    // Chuyển đổi giá trị sang chuỗi UTF-8
    QByteArray byteArray = message.toUtf8();
    byteArray = byteArray.leftJustified(6, '0', true);
//    const char *data = byteArray.constData();
    _port.writeData(byteArray);

    qDebug() << "Data Transmit: "<< byteArray;
}
//--------------------------------------------------------Camera Function-------------------------------------------------------------------------------------------

void MainWindow::display(QPixmap pixmap)
{

    if (videoCapture->checkCameraState() == false)
    {
        QImage image;
        image.load("No_signal.png");
        ui->Image->setPixmap(QPixmap::fromImage(image));
    }
    else
    {
        ui->Image->setPixmap(pixmap);
    }
}

void MainWindow::on_selShowImage_activated(int index)
{
   videoCapture->changeStreamOpt(index);
}


void MainWindow::on_openCam_but_clicked()
{
    std::cout << "Here open Camera " << std::endl;

    if (videoCapture->checkCameraState() == false)
    {
        connect(videoCapture, &capture::newPixmapCaptured, this, &MainWindow::display);
        ui->openCam_but->setText("Close Camera");
        videoCapture->startCamera();
        videoCapture->start(QThread::HighPriority);
    }
    else
    {
        disconnect(videoCapture, &capture::newPixmapCaptured, this, &MainWindow::display);
        videoCapture->stopCamera();
        QImage image;
        image.load("No_signal.png");
        ui->openCam_but->setText("Open Camera");
        ui->Image->setPixmap(QPixmap::fromImage(image));
    }
}

void MainWindow::on_capture_btn_clicked()
{
    static int imageIndex = 100;
    if (ui->detect_btn->text() == "On Detecting")
    {
        videoCapture->toggleArucoDetection(); // turn off detect ArUco
        captureThePictureAndData(imageIndex);
        videoCapture->toggleArucoDetection(); // turn on detect ArUco again
    }
    captureThePictureAndData(imageIndex);
    imageIndex++;
    qDebug() << "Index Image: " << imageIndex;
}

void MainWindow::captureThePictureAndData(int imageIndex)
{

    QString path_picture_Data = "D:/Tai_lieu_do_an/Picture_Aruco_Hand_For_Calib/PC_Robot_Control/Image_Data";
    QString path_data_robot = "D:/Tai_lieu_do_an/Picture_Aruco_Hand_For_Calib/PC_Robot_Control/Position";

    // Retrieve the current pixmap from the QLabel
    const QPixmap *currentPixmap = ui->Image->pixmap();

    // Convert the pixmap to a QImage
    QImage currentImage = currentPixmap->toImage();

    // Check if the image is valid
    if (!currentImage.isNull())
    {
        QString directory_pic = path_picture_Data; // Specify your directory here
        QString fileName_pic = QString("%1/image_(%2).jpg").arg(directory_pic).arg(imageIndex);

        // Save the QImage to a file
        if (currentImage.save(fileName_pic))
        {
//            QMessageBox::information(this, tr("Success"), tr("Image saved successfully."));

        }
        else
        {
            QMessageBox::warning(this, tr("Error"), tr("Failed to save image."));
        }
    }
    else
    {
        QMessageBox::warning(this, tr("Error"), tr("No image to save."));
    }

    //----------------------------------save data into .txt file-------

    // Generate the filename with the index
    QString directory_text = path_data_robot; // Specify your directory here
    QString fileName_text = QString("%1/position_(%2).txt").arg(directory_text).arg(imageIndex);

    // Open the file for writing
    QFile file(fileName_text);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        // Write your data to the file
        QTextStream out(&file);

        out << videoCapture->pixel[0]<< endl;
        out << videoCapture->pixel[1]<< endl;

//        out << videoCapture->point_in_camera_space[0]<< endl;
//        out << videoCapture->point_in_camera_space[1]<< endl;
//        out << videoCapture->point_in_camera_space[2]<< endl;

        out << ui->txt_get_PosX->text().toStdString().c_str() << endl;
        out << ui->txt_get_PosY->text().toStdString().c_str() << endl;
        out << ui->txt_get_PosZ->text().toStdString().c_str() << endl;
        out << ui->txt_get_Roll->text().toStdString().c_str() << endl;
        out << ui->txt_get_Pitch->text().toStdString().c_str() << endl;
        out << ui->txt_get_Yaw->text().toStdString().c_str() << endl;
        // Close the file
        file.close();

        QMessageBox::information(this, tr("Success"), tr("Data saved successfully."));
    }
    else
    {
        QMessageBox::warning(this, tr("Error"), tr("Failed to open file for writing."));
    }
}

void MainWindow::on_detect_btn_clicked()
{
    if(ui->detect_btn->text() == "Detect ArUco")
    {
        videoCapture->toggleArucoDetection();
        ui->detect_btn->setText("On Detecting");
    }else if (ui->detect_btn->text() == "On Detecting")
    {
        videoCapture->toggleArucoDetection();
        ui->detect_btn->setText("Detect ArUco");
    }
}

void MainWindow::on_btn_cal_clicked()
{
    int numberOfSample = ui->numberSampleCalib->text().toInt();
    maTrix->getMatrixFromCam2Base(numberOfSample);
}

void MainWindow::on_savepicObject_btn_clicked()
{
    if(ui->savepicObject_btn->text() == "Capture Object")
    {
        ui->savepicObject_btn->setText("Capturing");
        captureObjectImage();
    }
    else if(ui->savepicObject_btn->text() == "Capturing")
    {
        ui->savepicObject_btn->setText("Capture Object");
        captureObjectImage();
    }

}
void MainWindow::captureObjectImage()
{
    while(ui->savepicObject_btn->text() == "Capturing")
    {
        static int imageIndex = 0;
        // Retrieve the current pixmap from the QLabel
        const QPixmap *currentPixmap = ui->Image->pixmap();

        // Convert the pixmap to a QImage
        QImage currentImage = currentPixmap->toImage();

        // Check if the image is valid
        if (!currentImage.isNull())
        {
            QString directory_pic = "D:/Tai_lieu_do_an/Picture_Aruco_Hand_For_Calib/PC_Robot_Control/Image_Object"; // Specify your directory here
            QString fileName_pic = QString("%1/image_(%2).jpg").arg(directory_pic).arg(imageIndex);

            // Save the QImage to a file
            if (currentImage.save(fileName_pic))
            {
    //            QMessageBox::information(this, tr("Success"), tr("Image saved successfully."));
                imageIndex++;

            }
            else
            {
                QMessageBox::warning(this, tr("Error"), tr("Failed to save image."));
            }
        }
        else
        {
            QMessageBox::warning(this, tr("Error"), tr("No image to save."));
        }
        delay(1000);
    }
}

//-------------------------------------------Moving Average Speed--------------------------------------------------------------------------------------------------------
// Hàm tính giá trị trung bình di động của dữ liệu nhận được từ cổng serial
double MainWindow::calculateMovingAverage() {
    if (receivedDataQueue.empty()) {
        return 0; // Trả về 0 nếu không có dữ liệu trong hàng đợi
    }

    double sum = 0;
    int count = 0;
    std::queue<double> temp(receivedDataQueue); // Tạo một bản sao của hàng đợi để không làm thay đổi dữ liệu gốc

    while (!temp.empty()) {
        sum += temp.front();
        temp.pop();
        count++;
    }

    return sum / count;
}

// Hàm để xử lý dữ liệu nhận được từ cổng serial
void MainWindow::handleReceivedData(double newData) {
    receivedDataQueue.push(newData); // Thêm dữ liệu mới vào hàng đợi

    // Kiểm tra nếu kích thước của hàng đợi lớn hơn kích thước cửa sổ trung bình di động
    // Nếu có, loại bỏ phần tử cũ nhất
    if (receivedDataQueue.size() > windowSize) {
        receivedDataQueue.pop();
    }

    // Tính giá trị trung bình di động của dữ liệu trong hàng đợi
    double speedAverage = calculateMovingAverage();

    // Sử dụng giá trị trung bình di động ở đây
//     Ví dụ: qDebug() << "Moving Average: " << movingAverage;
}

//------------------------------------------Auto Run---------------------------------------------------------------------------------------------------------------------
void MainWindow::on_btn_auto_clicked()
{
    if( (ui->btn_connect->text() == "Disconnect") && (ui->btn_servo->text() == "Servo Off"))
    {
        if(runSystem)
        {
            runTimer->stop();
            runSystem = false;
            ui->btn_auto->setText("Auto Run");
            ui->btn_auto->setStyleSheet("QPushButton {color: green;}");
            videoCapture->toggleAutoRunSystem();
            openGripper();

        }
        else
        {
            openGripper();
            runTimer->start();
            runSystem = true;
            ui->btn_auto->setText("Stop Running");
            ui->btn_auto->setStyleSheet("QPushButton {color: red;}");
            videoCapture->toggleAutoRunSystem();

        }
    }
    else
    {
        QMessageBox::information(this,"Warning!!!","Check Connect UDP or Servo State again");
    }


}

/*
 * chỉnh sửa lại độ cao Z, nên calib cho thấp xuống thêm.
 * kiểm soát lại chu trình gắp vật
 * kiểm tra góc xoay Yaw khi gắp vật thể, chưa quá hoàn chỉnh
 * lập trình đọc tốc độ -> gắp vật
*/


//void MainWindow::autoRunSystem()            // code nay dang chay duoc ne nhung co loi
//{

//    qDebug() << "Calling autoRunSystem" ;
////    openGripper();
//    yrc1000micro_com->YRC1000microGetByte(07);      //byte đại diện robto đã đến home hay chưa?
//    delay(50);
//    if( yrc1000micro_com->byte_val == 1 && videoCapture->objInPos() == true || ui->txt_get_PosX->text() == "1" )
//    {
////        qDebug() << "Robot da den vi tri home";
//                                    // den vi tri home va mo tay gap
//        int centroidObjAtX = videoCapture->centroidObj.x;
//        qDebug() << "value of centroid at main: " << centroidObjAtX;
//        /* cho co vat the di vao vung detect */
//        if( /*videoCapture -> isObjIsOn() == 1*/  centroidObjAtX > 320 /*&& centroidObjAtX < 347 */  || ui->txt_get_PosY->text() == "1" )
//        {
////            qDebug() << "Da co vat the trong pham vi detect" ;
//            yrc1000micro_com->YRC1000microWriteByte(8, 1);     // Move Robot den vi tri cho P001
//            yrc1000micro_com->YRC1000microGetByte(9);          // kiem tra xem Robot da den P001 chua?
//            delay(20);                                          // delay de cho phan hoi tu host
//            if(yrc1000micro_com->byte_val == 1 || ui->txt_get_PosZ->text() == "1" )
//            {
////                qDebug() << "Robot da den vi tri cho P001" ;
////                   qDebug() << "gia tri objInPos: "<<videoCapture->objInPos();
//                if(  centroidObjAtX > 326 && centroidObjAtX < 600  || ui->txt_get_Roll->text() == "1" )
//                {
////                    qDebug() << "Lay thong tin cua vat can gap";
//                    /*-------------------------------- Get the position from objPos2Base ---------------*/
////                    qDebug() << "vat vo vi tri capture";
//                    QVector<int32_t> posPick, posPlace0, posMiddle;
//                    cv::Mat objPos2Base;
////                    videoCapture->testCode(objPos2Base);                               // lấy vị trí gắp vật
////                    qDebug() << "jump out testCode()";
//                    delay(300);
//                    objPos2Base = videoCapture->getObjPos2Base();       //lấy vị trí gắp vật

//                    std::cout << "ket qua cua objPos2Base (x, y, z): "
//                              << objPos2Base.at<double>(0, 0) << ", "
//                              << objPos2Base.at<double>(1, 0) << ", "
//                              << objPos2Base.at<double>(2, 0) << std::endl;
//                    qDebug() << "jump to Here 2";
//                    // Convert cv::Mat to QVector<double>
//                    posPick.resize(objPos2Base.rows);
//                    for (int i = 0; i < objPos2Base.rows; ++i) {
//                        posPick[i] = static_cast<int32_t>(std::round(objPos2Base.at<double>(i, 0)*1000));
//                    }

////                    qDebug() << "gan gia tri cho vi tri gap: " << posPick;
//                   /*------------------------------- Đã có thông tin vị trí gắp vật -------------------------------*/

////                    qDebug() << "gia tri tai vi tri gap posPick truoc khi config: " << posPick;

//                    /*------------------------------ thông tin config them cho vị trí gắp vật----------------------*/

////                    int32_t yValues = static_cast<int32_t>(std::round(speedAverageValue*1000));
//                    int32_t yValues = 10*1000;
//                    int32_t zValue = 30*1000;
////                    qDebug() << "gia trị them vao Y la: " << yValues;
//                    //------------------------------

//                    double_t yawValue = videoCapture->rotationAngle() * -1;        //chi có 1 biến thay đổi là x thôi, y có thể fix cứng được
////                   double yawValue = 12.0614;

//                    posPick[1] = posPick[1] + yValues*1.5;  // gia tri van toc van * thoi gian gap sau x giây
//                    posPick[2] -= zValue;
//                    posPick.push_back(180*10000);
//                    posPick.push_back(0);
//                    posPick.push_back(static_cast<int32_t>(std::round(yawValue*10000)));                    // bổ sung góc xoay Yaw gắp vật

////                    posMiddle = posPick;
////                    posMiddle[1] = posMiddle[1] + 20*1000; -> lôi ở đây
////                    qDebug() << "gia tri tai vi tri gap posPick: " << posPick;
//                    if(posPick[2] < -102000 ) posPick[2] = -102*1000;
//                    yrc1000micro_com->YRC1000microWriteVarPosition(002, posPick);  // ghi giá trị thanh ghi vị vị trí gắp vật
////                    yrc1000micro_com->YRC1000microWriteVarPosition(005, posMiddle);
//                    delay(30);
//                    yrc1000micro_com->YRC1000microWriteByte(10,1);               // Bật cờ cho Robot di chuyển đến vị trí gắp vật
////                    qDebug() << "Robot can di chuyen den vi tri gap vat P002";
//                    delay(20);
//                    yrc1000micro_com->YRC1000microGetByte(11);                  // kiểm tra xem robot đã đên vị trí P002 để gắp vật chưa?
//                    delay(50);

////                    qDebug() << "da chay toi day roi";
//                    if( yrc1000micro_com->byte_val == 1  || ui->txt_get_Pitch->text() == "1" )        // robot dang cho gap o vi tri tinh toan truoc
//                    {

////                        qDebug() << "Robot da di chuyen den vi tri gap vat P002";
////                        qDebug() << "Da co thong tin gap vat, gap ne";
//                        delay(100);
//                        grippObj();

//                        delay(100);
////                        qDebug() << "Da gap duoc vat the, chay den vi tri tha";
//                        int objColor = videoCapture->getObjectColor();                  // lấy thông tin màu sắc của vật
//                            //       int objColor = ui->testColor->text().toInt();
//                        qDebug() << "object color: "<< videoCapture->getObjectColor();

////                        qDebug() << "Initial size of posPlace0:" << posPlace0.size();
//                        posPlace0.clear();
//                        switch (objColor)               // dựa vào thông tin màu sắc để di chuyển
//                            {
//                                case 0:         //Orange object, move to vi tri thả
//                                {
//                            //                vị trí thả
//                                    posPlace0.push_back(-81*1000);
//                                    posPlace0.push_back(-216*1000);
//                                    posPlace0.push_back(-60*1000);
//                                    posPlace0.push_back(-180*10000);
//                                    posPlace0.push_back(0*10000);
//                                    posPlace0.push_back(-110*10000);
//                                    qDebug() << "Orange object detected";
//                                }
//                                    break;
//                                case 1:         //Green object
//                                {
//                                    posPlace0.push_back(38*1000);
//                                    posPlace0.push_back(-309.361*1000);
//                                    posPlace0.push_back(-51.751*1000);
//                                    posPlace0.push_back(-180*10000);
//                                    posPlace0.push_back(-2*10000);
//                                    posPlace0.push_back(-82.1064*10000);
//                                    qDebug() << "Green object detected";
//                                }
//                                    break;
//                                case 2:         //Yellow object
//                                {
//                                    posPlace0.push_back(24*1000);
//                                    posPlace0.push_back(-200*1000);
//                                    posPlace0.push_back(-54*1000);
//                                    posPlace0.push_back(-180*10000);
//                                    posPlace0.push_back(3*10000);
//                                    posPlace0.push_back(-81*10000);
//                                    qDebug() << "Yellow object detected";
//                                }
//                                    break;
//                                case 3:         //Red object
//                                {
//                                    posPlace0.push_back(-70.369*1000);
//                                    posPlace0.push_back(-303.691*1000);
//                                    posPlace0.push_back(-51.738*1000);
//                                    posPlace0.push_back(-180*10000);
//                                    posPlace0.push_back(0*10000);
//                                    posPlace0.push_back(-102.2282*10000);
//                                    qDebug() << "Red object detected";
//                                }
//                                    break;
//                                default:
//                                    posPlace0.push_back(185*1000);
//                                    posPlace0.push_back(0);
//                                    posPlace0.push_back(27*1000);
//                                    posPlace0.push_back(180*10000);
//                                    posPlace0.push_back(0);
//                                    posPlace0.push_back(0);
//                                    qDebug() << "No object has been detect";
//                                    break;
//                            }

////                        qDebug() << "Size of posPlace0 after assignment:" << posPlace0.size();
////                        qDebug() << "position at P003:" << posPlace0;

//                        yrc1000micro_com->YRC1000microWriteVarPosition(003, posPlace0);       // ghi giá trị cho vị trí thả
//                        delay(30);
//                        yrc1000micro_com->YRC1000microWriteByte(12,1);                      // byte cho di chuyen den vi tri Place
//                        delay(30);

//                        yrc1000micro_com->YRC1000microGetByte(13);              // kiem tra xem da hoan thanh toan bo chuong trinh chua? byte dai dien cho Robot da den vi tri tha
//                        delay(50);
//                        if(yrc1000micro_com->byte_val == 1 || ui->txt_get_Yaw->text() == "1")
//                        {
//                            delay(1000);
//                            openGripper();
//                            delay(100);
//                            yrc1000micro_com->YRC1000microGetByte(07);
//                            delay(20);


////                            ui->txt_get_PosX->clear();
////                            ui->txt_get_PosY->clear();
////                            ui->txt_get_PosZ->clear();
////                            ui->txt_get_Roll->clear();
////                            ui->txt_get_Pitch->clear();
////                            ui->txt_get_Yaw->clear();

//                            yrc1000micro_com->YRC1000microWriteByte(7, 0);      // cờ báo Robot đến vị trí home P000                  - nhận về khi Robot đã đến home
//                            yrc1000micro_com->YRC1000microWriteByte(8, 0);      // cờ báo Robot di chuyển đến vị trí chờ P001         - flag Control
//                            yrc1000micro_com->YRC1000microWriteByte(9, 0);      // cờ báo Robot đã di chuyển đến vị trí gắp P001      - flag Control System
//                            yrc1000micro_com->YRC1000microWriteByte(10, 0);     // cờ báo Robot di chuyển đến vị trí gắp vật P002     - flag Control
//                            yrc1000micro_com->YRC1000microWriteByte(11, 0);     // cờ báo Robot đã di chuyển đến vị trí gắp vật P002  - flag Control System
//                            yrc1000micro_com->YRC1000microWriteByte(12, 0);     // cờ báo Robot di chuyển đến vị trí thả vật          - flag Control
//                            yrc1000micro_com->YRC1000microWriteByte(13, 0);     // cờ báo Robot đã hoàn thành chu trình               - nhận về khi Robot đã đến P003
//                            videoCapture->resetObjInPos();                      //reset tat ca cac flag

//                        }
//                        else{
//                            qDebug() << "Robot dang di chuyen den vi tri tha";
//                        }
//                    }
//                    else
//                    {
//                        qDebug() << "Robot chua den vi tri P002 tinh toan truoc";
//                    }
//                }
//                else {
//                    qDebug() << "Vat the chua toi vi tri capture";
//                }
//            }
//            else {
//                qDebug() << "Robot chua den vi tri cho P001" ;

//            }
//        }
//        else {
//             qDebug() << "Chua co vat the " ;
//        }
//        /* neu co vat the di vao vung detect -> move robot toi vi tri cho */
//        /* neu Robot da den vi tri cho -> cho -> di chuyen robot den vi tri vat se di chuyen den */
//        /* khi robot den vi tri vat se den -> delay 1 khoang -> goi ham dong tay gap */
//    }
//    else {
//        qDebug() << "Khong co tin hieu dau vao";
//        videoCapture->resetFlag();
//    }
//    qDebug() <<  "---------------------------------------------------------------------------------" ;
//}
/* chu trình chạy trong Robot
  NOP
  WHILE(ON)
  {
    *START                      __ LABEL TO JUMP
    CLEAR B007
    CLEAR B008
    CLEAR B009
    CLEAR B010
    CLEAR B011
    CLEAR B012
    CLEAR B013
    CLEAR B014

    MOVJ P000
    SET B007 1
    *WAIT_AT_P001               __ LABEL TO JUMP

    IF (B008 = 1)
    {
        MOVJ P001
        SETB B009 1
        *WAIT_AT_P002           __LABEL TO JUMP

        IF(BO10 =1)
        {
            MOVJ P002
            SET B011 1           __cờ báo robot đã đến vị trí P002 là vị trí gắp

            *WAIT_AT_P003       __LABEL TO JUMP
            IF(B012 = 1)
            {
                MOVJ P005
                MOVJ P004       ___

                MOVJ P003
                timer = 0.5s    __delay hay gì đó ĐỂ THẢ VẬT
                SETB B013       __cờ báo robot đẵ đến vị trí thả
                timer = 1s
                SETB B014
                JUMP START
            }
            JUMP WAIT_AT_P003
        }
        JUMP WAIT_AT_P002
    }
    JUMP WAIT_AT_P001
  }
  END
*/

//---------------------------- code while loop auto run system --------------------------------------------------

//    qDebug() << "before while";
//    while(1)
//    {
//        qDebug() << "in while and byte != 1";
////        delay(500);
//        if(yrc1000micro_com->byte_val == 1)
//        {
//            qDebug() << "byte value = 1";
//            break;
//        }

//    }
//    qDebug() << "after while";


/*
 * viết lại hàm calib tay gắp để mở tay gắp đúng hơn
*/
void MainWindow::autoRunSystem()
{
    qDebug() << "Calling autoRunSystem";
//    while (videoCapture->coVat == false){}
//    double Yaw = videoCapture->rotationAngle();
//    qDebug() << "angle" << Yaw;
    yrc1000micro_com->YRC1000microGetByte(07);
    delay(100);
    while ( /*(*/ yrc1000micro_com->byte_val == 1 || ui->txt_get_PosX->text() == "1"/*) && videoCapture->objInPos() == true*/ )     // chờ robot den vi tri home
    {
//        qDebug() << "byte value at while:"<< yrc1000micro_com->byte_val;
        openGripper();
        qDebug() << "Robot da den vi tri home";
        delay(100);             // tránh treo máy khi cout
        int centroidObjAtX = videoCapture -> centroidObj.x;

        if(centroidObjAtX > 380  || ui->txt_get_PosY->text() == "1")
        {
            qDebug() << "Vat the den vi tri detect, di chuyen robot den vi tri P001";

            //-----------------------------cập nhật góc xoay Yaw trước -------------
//            QVector<int32_t> posYaw;
//            double_t yawValue = videoCapture->rotationAngle();

//            posYaw.push_back(221708);
//            posYaw.push_back(-134495);
//            posYaw.push_back(-2911);
//            posYaw.push_back(-1800000);
//            posYaw.push_back(-19630);
//            posYaw.push_back(yawValue*10000);
//            yrc1000micro_com->YRC1000microWriteVarPosition(006,posYaw);
            //----------------------------------------------------------------------

            yrc1000micro_com->YRC1000microWriteByte(8, 1);
            delay(100);
            yrc1000micro_com->YRC1000microGetByte(9);
            delay(100);

            while( yrc1000micro_com->byte_val == 0)     // chở byte 9 len 1
            {
//                qDebug() << "Cho doi byte 9 len 1";
                delay(100);
                yrc1000micro_com->YRC1000microGetByte(9);
                delay(100);
                if( yrc1000micro_com->byte_val == 1 || ui->txt_get_PosY->text() == "1")
                {
                    qDebug() << "Byte 9 da len 1";
                    break;
                }
                if(ui->btn_auto->text() == "Auto Run")
                {
                    qDebug() << "co nhay vo doan code if(ui->btn_auto->text() == Auto Run) ";
                    break;
                }       // cần thiết để thoát vòng while() sau khi tắt autoRun, nếu không vòng while() vẫn chạy độc lập
            }

            while( yrc1000micro_com->byte_val == 1 || ui->txt_get_PosZ->text() == "1" )       // Robot da den vi tri P001
            {
                centroidObjAtX = videoCapture -> centroidObj.x;
//                qDebug() << "gia tri centroi la: " << centroidObjAtX;
                qDebug() << "Robot da den vi tri P001";
                delay(100);
                qDebug() << "gia tri bien centroidObjAtX:" << centroidObjAtX;
                if( ( centroidObjAtX > 380 && centroidObjAtX < 600) || ui->txt_get_Roll->text() == "1"  )
                {
                    qDebug() << "Bat dau lay gia tri P002 de gap vat";

                    /*-------------------------------- Get the position from objPos2Base ---------------*/
                    QVector<int32_t> posPick, posPlace0, posMiddle;
                    cv::Mat objPos2Base;

                    /*-------------------------------- Get the position from objPos2Base ---------------*/
////                    videoCapture->testCode(objPos2Base);                // dùng trong trường hợp debug thôi
////                    delay(100);
                    objPos2Base = videoCapture->getObjPos2Base();       // lấy vị trí gắp vật

//                    // Convert cv::Mat to QVector<double>
                    posPick.resize(objPos2Base.rows);
                    for (int i = 0; i < objPos2Base.rows; ++i)
                    {
                        posPick[i] = static_cast<int32_t>(std::round(objPos2Base.at<double>(i, 0)*1000));
                    }

//                    /*--------------- thông tin config thêm vào cho vị trí gắp vật ()-------------*/
                    int32_t yValue = 20*1000;           // giá trị vận tốc thêm vào
                    int32_t zValue = 30*1000;
//                    delay(500);
                    double_t yawValue = videoCapture->rotationAngle();

                    if(yawValue > 90.0)
                    {
                        yawValue -=180;
                    }
                    else if(yawValue < -90.0)
                    {
                        yawValue +=180;
                    }

//                    {
//                        QVector<int32_t> p006;
//                        p006.push_back(221.708*1000);
//                        p006.push_back(-134.495*1000);
//                        p006.push_back(-2.911*1000);
//                        p006.push_back(-180*10000);
//                        p006.push_back(0*10000);
//                        p006.push_back(static_cast<int32_t>(std::round(yawValue*10000)));
//                        yrc1000micro_com->YRC1000microWriteVarPosition(006, p006);
//                        delay(20);
//                        p006.clear();
//                    }
////                    double yawValue = 12.0614;                              // for Debug
                    posPick[1] = posPick[1] + yValue *1.5;  // gia tri van toc van * thoi gian gap sau x giây
//                    posPick[1] = posPick[1] + yValue*2.2;  // gia tri van toc van * thoi gian gap sau x giây
                    posPick[2] -= zValue;
                    posPick.push_back(180*10000);
                    posPick.push_back(0);
                    posPick.push_back(static_cast<int32_t>(std::round(yawValue*10000)));
                    qDebug() <<"yaw Value:" << yawValue;

                    if( posPick[2] < -102000) posPick[2] = -100000;
                    int objColor = videoCapture->getObjectColor();

                    if(objColor == 0)
                    {
//                        posPick[2] = -97200;
                        posPick[2] = posPick[2] + -1*1000;
//                        posPick[5] = std::round(yawBack*10000);
                    }
                    if(objColor == 2)
                    {
//                        posPick[2] = -106000;
                        posPick[2] = posPick[2] + -3*1000;
//                        posPick[5] -= (30*10000);
                    }
                    if(objColor == 1)
                    {
                        posPick[5] -= 10*10000;
                    }


                    qDebug() << "gan gia tri cho vi tri gap: " << posPick;              // cout gia trị P002 truyền vô Robot
                    yrc1000micro_com->YRC1000microWriteVarPosition(002,posPick);        // ghi gia trị của vị trí gắp vô P002 của Robot;
                    posPick.clear();
                    delay(100);
                    yrc1000micro_com->YRC1000microWriteByte(10, 1);
                    delay(100);
                    yrc1000micro_com->YRC1000microGetByte(11);
                    delay(100);
                    qDebug() << "Robot da den vi tri P002 de gap vat, delay de khep tay gap lai";

                    while( yrc1000micro_com->byte_val == 0 )
                    {
                        qDebug() << "Cho doi byte 11 len 1";
                        delay(50);
                        yrc1000micro_com->YRC1000microGetByte(11);
                        delay(50);
                        if( yrc1000micro_com->byte_val == 1 || ui->txt_get_Roll->text() == "1" )
                        {
                            static int countIndex = 0;
                            qDebug() << "Byte 11 da len 1";
                            break;
                        }
                        if(ui->btn_auto->text() == "Auto Run")
                        {
                            qDebug() << "co nhay vo doan code if(ui->btn_auto->text() == Auto Run) ";
                            break;
                        }       // cần thiết để thoát vòng while() sau khi tắt autoRun, nếu không vòng while() vẫn chạy độc lập
                    }

                    while( yrc1000micro_com->byte_val == 1 || ui->txt_get_Pitch->text() == "1")             // Robot da den vi tri P003
                    {
                        qDebug() << "Robot da gap vat xong, di chuyen den vi tri tha P003";
                        delay(50);                 // thêm vào để tránh treo GUI
                        grippObj();

////                        int objColor = ui->testColor->text().toInt();     //for debug purpose
                        posPlace0.clear();
                        switch (objColor)               // dựa vào thông tin màu sắc để di chuyển
                            {
                                case 0:         //Orange object, move to vi tri thả
                                {
                            //                vị trí thả
                                    posPlace0.push_back(-149*1000 + 42*orangeCount*1000);
                                    posPlace0.push_back(-196.521*1000);
                                    posPlace0.push_back(-50*1000);
                                    posPlace0.push_back(-180*10000);
                                    posPlace0.push_back(0*10000);
                                    posPlace0.push_back(-92.1135*10000);
                                    orangeCount += 1;
                                    if(orangeCount == 3)
                                    {
                                        orangeCount =0;
                                    }
                                    qDebug() << "Orange object detected";
                                }
                                    break;
                                case 1:         //Green object
                                {
                                    posPlace0.push_back(105*1000 - 35*greenCount*1000);
                                    posPlace0.push_back(-309.361*1000);
                                    posPlace0.push_back(-50.443*1000);
                                    posPlace0.push_back(-180*10000);
                                    posPlace0.push_back(-2*10000);
                                    posPlace0.push_back(-82.1064*10000);
                                    greenCount += 1;
                                    if(greenCount == 3)
                                    {
                                        greenCount =0;
                                    }
                                    qDebug() << "Green object detected";
                                }
                                    break;
                                case 2:         //Yellow object
                                {
                                    posPlace0.push_back(105*1000 - 35*yellowCount*1000);
                                    posPlace0.push_back(-188.431*1000);
                                    posPlace0.push_back(-50*1000);
                                    posPlace0.push_back(-180*10000);
                                    posPlace0.push_back(3*10000);
                                    posPlace0.push_back(-88.1015*10000);
                                    yellowCount += 1;
                                    if(yellowCount == 3)
                                    {
                                        yellowCount =0;
                                    }
                                    qDebug() << "Yellow object detected";
                                }
                                    break;
                                case 3:         //Red object
                                {
                                    posPlace0.push_back(-132*1000 + 45*redCount*1000);
                                    posPlace0.push_back(-287.674*1000);
                                    posPlace0.push_back(-50.443*1000);
                                    posPlace0.push_back(-180*10000);
                                    posPlace0.push_back(0*10000);
                                    posPlace0.push_back(-102.2282*10000);
                                    redCount += 1;
                                    if(redCount == 3)
                                    {
                                        redCount =0;
                                    }
                                    qDebug() << "Red object detected";
                                }
                                    break;
                                default:
                                    posPlace0.push_back(185*1000);
                                    posPlace0.push_back(0);
                                    posPlace0.push_back(27*1000);
                                    posPlace0.push_back(180*10000);
                                    posPlace0.push_back(0);
                                    posPlace0.push_back(0);
                                    qDebug() << "No object has been detect";
                                    break;
                            }
                        qDebug() << "position at P003:" << posPlace0;

                        yrc1000micro_com->YRC1000microWriteVarPosition(003,posPlace0);
                        delay(20);
                        //----------------nâng cao lên tại vị trí thả------------

                        QVector<int32_t> p007;
                        p007 = posPlace0;
                        p007[2] = -25*1000;
                        yrc1000micro_com->YRC1000microWriteVarPosition(007,p007);
                        delay(20);
                        //----------------nâng cao lên tại vị trí thả------------

                        yrc1000micro_com->YRC1000microWriteByte(12, 1);
                        delay(100);
                        yrc1000micro_com->YRC1000microGetByte(13);
                        delay(100);



                        while( yrc1000micro_com->byte_val == 0)
                        {
                            qDebug() << "Cho doi byte 13 len 1";
                            delay(100);
                            yrc1000micro_com->YRC1000microGetByte(13);
                            delay(100);
                            if( yrc1000micro_com->byte_val == 1 || ui->txt_get_Pitch->text() == "1")
                            {
                                qDebug() << "Byte 13 da len 1";
                                break;
                            }

                            if(ui->btn_auto->text() == "Auto Run")
                            {
                                qDebug() << "co nhay vo doan code if(ui->btn_auto->text() == Auto Run) ";
                                break;
                            }       // cần thiết để thoát vòng while() sau khi tắt autoRun, nếu không vòng while() vẫn chạy độc lập
                        }

                        while( yrc1000micro_com->byte_val == 1 || ui->txt_get_Yaw->text() == "1")               // robot da den vi tri P003 xong, re-call program
                        {
                            qDebug()<<"Robot da den vi tri tha P003, delay de mo tay gap";
                            delay(300);                                     // delay tranh treo GUI
                            openGripper();
                            yrc1000micro_com->YRC1000microGetByte(14);      // byte delay sau chờ thả, sau khi delay chờ thả xong thì đc set lên 1

                            while( yrc1000micro_com->byte_val == 0)
                            {
                                qDebug() << "Cho doi byte 14 len 1";
                                delay(100);
                                yrc1000micro_com->YRC1000microGetByte(14);
                                delay(100);
                                if( yrc1000micro_com->byte_val == 1  || ui->txt_get_Yaw->text() == "1")
                                {
                                    openGripper();
                                    qDebug() << "Byte 14 da len 1";
                                    break;
                                }

                                if(ui->btn_auto->text() == "Auto Run")
                                {
                                    qDebug() << "co nhay vo doan code if(ui->btn_auto->text() == Auto Run) ";
                                    break;
                                }       // cần thiết để thoát vòng while() sau khi tắt autoRun, nếu không vòng while() vẫn chạy độc lập
                            }

                            while ( yrc1000micro_com->byte_val == 1 || ui->testInput->text() == "1")
                            {

                                qDebug() << "Da tha xong, reset cac co, re-run program";
                                delay(100);                     // thêm vào để tránh tràn GUI
                                openGripper();
                                yrc1000micro_com->YRC1000microWriteByte(7, 0);      // cờ báo Robot đến vị trí home P000                  - nhận về khi Robot đã đến home
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(8, 0);      // cờ báo Robot di chuyển đến vị trí chờ P001         - flag Control
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(9, 0);      // cờ báo Robot đã di chuyển đến vị trí gắp P001      - flag Control System
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(10, 0);     // cờ báo Robot di chuyển đến vị trí gắp vật P002     - flag Control
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(11, 0);     // cờ báo Robot đã di chuyển đến vị trí gắp vật P002  - flag Control System
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(12, 0);     // cờ báo Robot di chuyển đến vị trí thả vật          - flag Control
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(13, 0);     // cờ báo Robot đã hoàn thành chu trình               - nhận về khi Robot đã đến P003
                                delay(20);
                                yrc1000micro_com->YRC1000microWriteByte(14, 0);     // cờ báo Robot đã hoàn thành chu trình               - nhận về khi Robot đã đến P003
                                delay(20);

                                videoCapture -> centroidObj.x = 0;


//                                {   // debug reset program
//                                    ui->txt_get_PosX->clear();
//                                    ui->txt_get_PosY->clear();
//                                    ui->txt_get_PosZ->clear();
//                                    ui->txt_get_Roll->clear();
//                                    ui->txt_get_Pitch->clear();
//                                    ui->txt_get_Yaw->clear();
                                    done = true;                    // biến bool kiểm tra vòng lặp chương trình phân loại của robot
//                                }

                                videoCapture->resetObjInPos();
//                                qDebug() << "value of ObjInPos:" << videoCapture-> objinPosTest();  // Debug purpose

                                if(ui->btn_auto->text() == "Auto Run") break;                   // cần thiết để thoát vòng while() sau khi tắt autoRun, nếu không vòng while() vẫn chạy độc lập
                                qDebug() << "------------------------------------New Block code at getByte 14 or 15 tùy chọn, nên có---------------------------------";
                                if( done == true )  break;
////                                if( stop == true )  break;     //tắt loop while khi tắt GUI
                            }

                            if(ui->btn_auto->text() == "Auto Run")
                            {
                                qDebug() << "co nhay vo doan code if(ui->btn_auto->text() == Auto Run) ";
                                break;
                            }       // cần thiết để thoát vòng while() sau khi tắt autoRun, nếu không vòng while() vẫn chạy độc lập
                            qDebug() << "------------------------------------New Block code at getByte 13---------------------------------";
                            if( done == true )
                            {
                                qDebug() << "co nhay vo doan code if( done == true ) ";
                                break;
                            }

////                            if( stop == true )  break;                                 //tắt loop while khi tắt GUI
                        }

                        qDebug() << "------------------------------------New Block code at getByte 11---------------------------------";
                        if(ui->btn_auto->text() == "Auto Run")
                        {
                            qDebug() << "co nhay vo doan code if(ui->btn_auto->text() == Auto Run) ";
                            break;
                        }       // cần thiết để thoát vòng while() sau khi tắt autoRun, nếu không vòng while() vẫn chạy độc lập

                        if( done == true )
                        {
                            qDebug() << "co nhay vo doan code if( done == true ) ";
                            break;
                        }
////                        if( stop == true )  break;                                     //tắt loop while khi tắt GUI
                    }
                }
                else
                {
                    qDebug() << "Robot chua den vi tri picking P002";
                }

                if(ui->btn_auto->text() == "Auto Run")
                {
                    qDebug() << "co nhay vo doan code if(ui->btn_auto->text() == Auto Run) ";
                    break;
                }       // cần thiết để thoát vòng while() sau khi tắt autoRun, nếu không vòng while() vẫn chạy độc lập
                qDebug() << "------------------------------------New Block code at getByte 09---------------------------------";
                if( done == true )
                {
                    qDebug() << "co nhay vo doan code if( done == true ) ";
                    break;
                }

////                if( stop == true )  break;                         //tắt loop while khi tắt GUI
            }
            qDebug() << "debug kiem tra vat the den vi tri detect";
        }
        else
        {
            qDebug() <<"vat the chua den vi tri detect";
        }
        qDebug() << "------------------------------------New Block code at getByte 07---------------------------------";
        if( done == true )
        {
            qDebug() << "co nhay vo doan code if( done == true ) ";
            break;
        }
//        delay(500);             // tránh treo máy
        if(ui->btn_auto->text() == "Auto Run")
        {
            qDebug() << "co nhay vo doan code if(ui->btn_auto->text() == Auto Run) ";
            break;
        }       // cần thiết để thoát vòng while() sau khi tắt autoRun, nếu không vòng while() vẫn chạy độc lập
//        if( stop == true )  break;                         //tắt loop while khi tắt GUI
    }
    qDebug() << "dang cho robot den vi tri home";
    qDebug() << "------------------------------------New Block code Debug---------------------------------";
    done = false;
}

    //--------------autoRunSystem test-------------------
void MainWindow::on_test_fnc_btn_clicked()
{
    yrc1000micro_com->YRC1000microGetByte(11);
    grippObj();
}
void MainWindow::grippObj()
{
    // Get the distance from the capture object
//    double distance = videoCapture->distanceGrip();             // nen sua gia tri nay lai va
    qDebug() << "Calling Close Gripper";                                                         // viet ham convert tu pixel -> xung PWM luon, gia tri lay o day chi co xung thoi
    double distance = ui->numberSampleCalib->text().toDouble();     // so xung = 10*cm +30;
    distance =(int)(10*distance +30);
    // Convert the distance to QString
    QString distanceStr = QString::number(distance, 'f', 5); // Format to 4 decimal places
    // Convert QString to QByteArray
    QByteArray byteArray = distanceStr.toUtf8();

    // Left justify the byte array to ensure it's 6 characters long
    byteArray = byteArray.leftJustified(6, '0', true);
//    qDebug() << "size of daata send " << sizeof(byteArray);
    // Send the byte array through the serial port
    _port.writeData(byteArray);
//    qDebug() << "End sending control gripper success";
}
void MainWindow::openGripper()
{
    qDebug() << "Calling Open Gripper";
    // Convert the distance to QString
    QString distanceStr = QString::number(680.0, 'f', 5); // Format to 4 decimal places
    // Convert QString to QByteArray
    QByteArray byteArray = distanceStr.toUtf8();

    // Left justify the byte array to ensure it's 6 characters long
    byteArray = byteArray.leftJustified(6, ' ', true);

    // Send the byte array through the serial port
    _port.writeData(byteArray);
//    qDebug() << "End sending uart success";
}

void MainWindow::on_reset_Sys_butt_clicked()
{
    videoCapture->resetObjInPos();
    yrc1000micro_com->YRC1000microWriteByte(7, 0);      // cờ báo Robot đến vị trí home P000                  - nhận về khi Robot đã đến home
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(8, 0);      // cờ báo Robot di chuyển đến vị trí chờ P001         - flag Control
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(9, 0);      // cờ báo Robot đã di chuyển đến vị trí gắp P001      - flag Control System
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(10, 0);     // cờ báo Robot di chuyển đến vị trí gắp vật P002     - flag Control
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(11, 0);     // cờ báo Robot đã di chuyển đến vị trí gắp vật P002  - flag Control System
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(12, 0);     // cờ báo Robot di chuyển đến vị trí thả vật          - flag Control
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(13, 0);     // cờ báo Robot đã hoàn thành chu trình               - nhận về khi Robot đã đến P003
    delay(20);
    yrc1000micro_com->YRC1000microWriteByte(14, 0);     // cờ báo Robot đã hoàn thành chu trình               - nhận về khi Robot đã đến P003
    delay(20);
    openGripper();
    on_btn_home_clicked();
    videoCapture -> centroidObj.x = 0;
    orangeCount = 0;
    greenCount = 0;
    yellowCount = 0;
    redCount = 0;

}
void MainWindow::delay(int n)
{
    QTime dieTime= QTime::currentTime().addMSecs(n);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
};


int MainWindow::findMaxX(const std::vector<cv::Point>& points) {
    if (points.empty()) {
        throw std::invalid_argument("Vector is empty!");
    }

    auto maxXPoint = std::max_element(points.begin(), points.end(),
        [](const cv::Point& p1, const cv::Point& p2) {
            return p1.x < p2.x; // So sánh giá trị x của các điểm
        });

    return maxXPoint->x;
}
