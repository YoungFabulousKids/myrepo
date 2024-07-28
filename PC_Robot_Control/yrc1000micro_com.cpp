#include "yrc1000micro_com.h"
#include "yrc1000micro_command.h"
#include <iostream>
#include <QMessageBox>

YRC1000micro_com::YRC1000micro_com(QObject *parent) : QObject(parent)
{
    request_id_index = 0;
    response_id_index = 0;

    robot_position.append(0);
    robot_position.append(0);
    robot_position.append(0);
    robot_position.append(0);
    robot_position.append(0);
    robot_position.append(0);

    robot_pulse.append(0);
    robot_pulse.append(0);
    robot_pulse.append(0);
    robot_pulse.append(0);
    robot_pulse.append(0);
    robot_pulse.append(0);
//    connect(&udp_server, SIGNAL(dataRecieveSignal()), this, SLOT (YRC1000micronDataCallback()));
    connect(&udp_server, &UDP::dataReceiveSignal, this, &YRC1000micro_com::YRC1000microDataCallback);
}
YRC1000micro_com::~YRC1000micro_com(){
    udp_server.~UDP();
}

bool YRC1000micro_com::YRC1000microConnect(){
    bool connect_satus = udp_server.udpConnect(udp_address,udp_port);
    return connect_satus;
}

void YRC1000micro_com::YRC1000microDisConnect(){
    udp_server.udpDisConnect();
}

void YRC1000micro_com::YRC1000microSetConnection(QHostAddress address, quint16 port){
    udp_address = address;
    udp_port = port;
    YRC1000microConnect();
}

void YRC1000micro_com::YRC1000microOnServo(){
    QByteArray data = yrc1000micro_command.setServoOn();
    udp_server.sendData(udp_address,udp_port,data);
//    qDebug() << data;
}

void YRC1000micro_com::YRC1000microOffServo(){
    QByteArray data = yrc1000micro_command.setServoOff();
    udp_server.sendData(udp_address,udp_port,data);
}

void YRC1000micro_com::YRC1000microMoveCartesian(quint8 coordinate,quint8 move_type, quint8 speed_type, double speed, QVector<double>* position)
{
    QByteArray data = yrc1000micro_command.setRobotPositionCartesian(coordinate, move_type, speed_type, speed, position);
    // Debug output: Hex value
//    QString hexData = data.toHex().toUpper();
//    qDebug() << "Hex data at set position:" << hexData;
    udp_server.sendData(udp_address,udp_port,data);
}

void YRC1000micro_com::YRC1000microHomePos()
{
    QByteArray data = yrc1000micro_command.HomePos();
    udp_server.sendData(udp_address,udp_port,data);
}

void YRC1000micro_com::YRC1000ReadPosition(){
    QByteArray data = yrc1000micro_command.readRobotPosition();
    udp_server.sendData(udp_address, udp_port, data);
//    qDebug() << "data read pos cmd : "<< data ;
}

void YRC1000micro_com::YRC1000microDataCallback(){
    QByteArray data = udp_server.getUdpData();
//    qDebug() << "Received Data: " << data;

    // Extract the request ID from the received data
    quint8 res_id_index = (quint8)data[CMD_REQUEST_ID];
    quint8 index_response = (quint8)data[CMD_ACK];
//    qDebug() << "request_id: " << res_id_index;

    // Use switch-case to handle different response IDs
    switch ( res_id_index)
    {
        case CMD_REQUEST_ID_GET_POSITION:
//        case CMD_REQUEST_ID_WRITE_POSITION:r
        // Handle response for Servo On command
            YRC1000microReadPositionResponse(data);
        break;

        case CMD_REQUEST_ID_GET_BYTE:
        // Handle response for Servo Off command
            byte_val = data.at(32);
            qDebug() << "value of byte: " << byte_val;
        break;

//        case CMD_REQUEST_ID_WRITE_VARPOS:
//            qDebug() << "size of command comeback:" << sizeof(data);
//            qDebug() << "command of responding writing variable position: " << data;
//            qDebug() << "thanh ghi duoc ghi gia tri bien la:" << (quint8)data[CMD_INSTANCE];
//            qDebug() << "----------------to Hex-------------";
//        break;

        // Add cases for other CMD_REQUEST_ID_* as needed
        default:
//            qDebug() << "Unknown request ID: " << res_id_index;
        break;
    }

        emit dataUIRecieveSiUIgnal();
}

void YRC1000micro_com::YRC1000microReadPositionResponse(QByteArray data){

    qint32 x_axis = 0;
    qint32 y_axis = 0;
    qint32 z_axis = 0;
    qint32 roll_angle = 0;
    qint32 pitch_angle = 0;
    qint32 yaw_angle = 0;

    x_axis = ((quint32)(quint8)data[DATA_X_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_X_AXIS+2])<<16 |
            ((quint32)(quint8)data[DATA_X_AXIS+1])<<8 | (quint32)(quint8)data[DATA_X_AXIS];

    y_axis = ((quint32)(quint8)data[DATA_Y_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_Y_AXIS+2])<<16 |
             ((quint32)(quint8)data[DATA_Y_AXIS+1])<<8 | (quint32)(quint8)data[DATA_Y_AXIS];

    z_axis = ((quint32)(quint8)data[DATA_Z_AXIS+3])<<24 |((quint32)(quint8)data[DATA_Z_AXIS+2])<<16 |
             ((quint32)(quint8)data[DATA_Z_AXIS+1])<<8 | (quint32)(quint8)data[DATA_Z_AXIS];

    roll_angle = ((quint32)(quint8)data[DATA_ROLL_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_ROLL_ANGLE+2])<<16 |
                 ((quint32)(quint8)data[DATA_ROLL_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_ROLL_ANGLE];

    pitch_angle = ((quint32)(quint8)data[DATA_PITCH_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_PITCH_ANGLE+2])<<16 |
                  ((quint32)(quint8)data[DATA_PITCH_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_PITCH_ANGLE];

    yaw_angle = ((quint32)(quint8)data[DATA_YAW_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_YAW_ANGLE+2])<<16 |
                ((quint32)(quint8)data[DATA_YAW_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_YAW_ANGLE];

    robot_position[0]=(x_axis/1000.0);
    robot_position[1]=(y_axis/1000.0);
    robot_position[2]=(z_axis/1000.0);
    robot_position[3]=(roll_angle/10000.0);
    robot_position[4]=(pitch_angle/10000.0);
    robot_position[5]=(yaw_angle/10000.0);
    qDebug() <<"robot position: " <<robot_position;
}

QVector<double> YRC1000micro_com::updateRobotPosition(){
    return robot_position;
}

void YRC1000micro_com::YRC1000microLoadJob(char* jobName)
{
    QByteArray data = yrc1000micro_command.selectJob(jobName);
    udp_server.sendData(udp_address,udp_port,data);
}

void YRC1000micro_com::YRC1000microStartJob()
{
    QByteArray data= yrc1000micro_command.startJob();
    udp_server.sendData(udp_address, udp_port, data);
}

//--------Read and Write Var Position ----------------------------------------------
void YRC1000micro_com::YRC1000microGetVarPosition(uint16_t index)
{
    QByteArray data= yrc1000micro_command.getVarPosition(index);
    udp_server.sendData(udp_address, udp_port, data);
}

void YRC1000micro_com::YRC1000microWriteVarPosition(uint16_t index, QVector<int32_t> pos)
{
    QByteArray data = yrc1000micro_command.writeVarPosition(index, pos);
    udp_server.sendData(udp_address, udp_port, data);
}
//--------Read and Write Byte ----------------------------------------------
void YRC1000micro_com::YRC1000microGetByte(uint16_t index)
{
    QByteArray data= yrc1000micro_command.getByte(index);
    udp_server.sendData(udp_address, udp_port, data);
}
void YRC1000micro_com::YRC1000microWriteByte(uint16_t index, uint8_t buffer)
{
    QByteArray data= yrc1000micro_command.writeByte(index, buffer);
    udp_server.sendData(udp_address, udp_port, data);
}
//--------Read and Write Byte ----------------------------------------------


void YRC1000micro_com::YRC1000microWriteVariablePositionAgain(uint16_t index, QVector<double>*pos)
{
    QByteArray data= yrc1000micro_command.writeVarPositionAgain(index, pos);
    udp_server.sendData(udp_address, udp_port, data);
}
