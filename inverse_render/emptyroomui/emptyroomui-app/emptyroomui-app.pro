#-------------------------------------------------
#
# Project created by QtCreator 2015-04-08T10:29:48
#
#-------------------------------------------------

QT       += core gui opengl xml widgets

CONFIG += c++11

TARGET = ../emptyroomui
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
        eruiglwidget.cpp \
    hdrviewer.cpp \
    qxtspanslider.cpp \
    subprocessworker.cpp \
    hdrglwidget.cpp \
    hdrimageviewer.cpp

HEADERS  += mainwindow.h \
            eruiglwidget.h \
    hdrviewer.h \
    qxtspanslider.h \
    subprocessworker.h \
    hdrglwidget.h \
    hdrimageviewer.h

FORMS    += mainwindow.ui

GAPS_DIR = $$PWD/../../../GAPS/
INVRENDER_DIR = $$PWD/../../build/
SHADER_DIR = $$PWD/../../shaders/
unix: {
    LIBS += -L$$INVRENDER_DIR \
        -L$$GAPS_DIR/R3Graphics/ -L$$GAPS_DIR/R3Shapes/ -L$$GAPS_DIR/RNBasics/ -L$$GAPS_DIR/R2Shapes/ \
        -L$$OUT_PWD/../QGLViewer \
        -lrendering -lroommodel -ldatamanager -lHalf -lIlmImf \
        -lR3Graphics -lR3Shapes -lR2Shapes -lRNBasics -ljpeg -lpng \
        -lrt -lboost_system -lboost_filesystem \
        -lGLU -lQGLViewer -lpcl_common -lpcl_io -lpcl_io_ply
}
unix:!macx {
    LIBS += -lGLEW
}
INCLUDEPATH += $$PWD/../../
INCLUDEPATH += $$GAPS_DIR
DEPENDPATH += $$PWD/../../

unix: PRE_TARGETDEPS += $$INVRENDER_DIR/libdatamanager.a $$INVRENDER_DIR/libroommodel.a $$INVRENDER_DIR/librendering.a ../QGLViewer/libQGLViewer.a

OTHER_FILES += \
    $$SHADER_DIR/default.f.glsl \
    $$SHADER_DIR/flat.f.glsl \
    $$SHADER_DIR/avg.v.glsl \
    $$SHADER_DIR/labels.v.glsl \
    $$SHADER_DIR/normals.v.glsl \
    $$SHADER_DIR/singleimage.v.glsl \
    $$SHADER_DIR/default.v.glsl \
    $$SHADER_DIR/overlay.v.glsl \
    $$SHADER_DIR/labels.g.glsl \
    $$SHADER_DIR/overlay.g.glsl \
    $$SHADER_DIR/threshold.v.glsl \
    $$SHADER_DIR/threshold.g.glsl

QMAKE_POST_LINK += cp $$SHADER_DIR/* ../

