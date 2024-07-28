#-------------------------------------------------
#
# Project created by QtCreator 2024-05-29T11:28:28
#
#-------------------------------------------------

QT       += core gui
QT       += network
QT       += core
QT       += core gui network serialport
QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PC_Robot_Control
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    udp.cpp \
    yrc1000micro_com.cpp \
    yrc1000micro_command.cpp \
    serialport.cpp \
    capture.cpp \
    caculatematrix.cpp

HEADERS += \
        mainwindow.h \
    udp.h \
    yrc1000micro_com.h \
    yrc1000micro_command.h \
    serialport.h \
    capture.h \
    caculatematrix.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

LIBS += "C:\Program Files (x86)\Intel RealSense SDK 2.0\bin\x64\realsense2.dll" #-realsense2

INCLUDEPATH += 'C:\Program Files (x86)\Intel RealSense SDK 2.0\include'
DEPENDPATH += 'C:\Program Files (x86)\Intel RealSense SDK 2.0\include'

INCLUDEPATH += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\include
DEPENDPATH  += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\include


LIBS += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\x64\mingw\bin\libopencv_aruco470.dll
LIBS += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\x64\mingw\bin\libopencv_core470.dll
LIBS += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\x64\mingw\bin\libopencv_imgcodecs470.dll
LIBS += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\x64\mingw\bin\libopencv_highgui470.dll
LIBS += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\x64\mingw\bin\libopencv_calib3d470.dll

LIBS += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\x64\mingw\bin\libopencv_imgproc470.dll
LIBS += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\x64\mingw\bin\libopencv_video470.dll
LIBS += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\x64\mingw\bin\libopencv_videoio470.dll

LIBS += D:\Software\OpenCV2\OpenCV\build_MinGW_Module\install\x64\mingw\bin\libopencv_features2d470.dll
