#-------------------------------------------------
#
# Project created by QtCreator 2015-04-08T10:29:48
#
#-------------------------------------------------

QT       += core gui opengl xml widgets

TARGET = emptyroomui
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

unix: {
    LIBS += -L$$PWD/../build/ \
        -L$$PWD/../../GAPS/R3Graphics/ -L$$PWD/../../GAPS/R3Shapes/ -L$$PWD/../../GAPS/RNBasics/ -L$$PWD/../../GAPS/R2Shapes/ \
        -lrendering -lmemory -lHalf -lIlmImf \
        -lR3Graphics -lR3Shapes -lR2Shapes -lRNBasics -ljpeg -lpng \
        -lrt -lboost_system -lboost_filesystem \
        -lQGLViewer -lpcl_common -lpcl_io -lpcl_io_ply
}
unix:!macx {
    LIBS += -lGLEW
}
INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../GAPS/
DEPENDPATH += $$PWD/../

unix: PRE_TARGETDEPS += $$PWD/../build/libmemory.a $$PWD/../build/librendering.a

OTHER_FILES += \
    $$PWD/../shaders/default.f.glsl \
    $$PWD/../shaders/flat.f.glsl \
    $$PWD/../shaders/avg.v.glsl \
    $$PWD/../shaders/labels.v.glsl \
    $$PWD/../shaders/normals.v.glsl \
    $$PWD/../shaders/singleimage.v.glsl \
    $$PWD/../shaders/default.v.glsl \
    $$PWD/../shaders/overlay.v.glsl \
    $$PWD/../shaders/labels.g.glsl \
    $$PWD/../shaders/overlay.g.glsl \
    $$PWD/../shaders/threshold.v.glsl \
    $$PWD/../shaders/threshold.g.glsl

QMAKE_POST_LINK += cp $$PWD/../shaders/* $$OUT_PWD/

