TEMPLATE = app

CONFIG += console c++11

CONFIG -= app_bundle

CONFIG -= qt

SOURCES += main.cpp \
    serial.cpp

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv
                /usr/local/include/opencv2
QMAKE_CXXFLAGS += \
    -O3
LIBS += /usr/local/lib/libopencv_*

HEADERS += \
    serial.h
