#-------------------------------------------------
#
# Project created by QtCreator 2015-04-08T10:29:48
#
#-------------------------------------------------

QT       += core gui opengl xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

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

unix: LIBS += -L$$PWD/../build/ \
        -L$$PWD/../../GAPS/R3Graphics/ -L$$PWD/../../GAPS/R3Shapes/ -L$$PWD/../../GAPS/RNBasics/ -L$$PWD/../../GAPS/R2Shapes/ \
        -lrendering -lmemory -lHalf -lIlmImf \
        -lR3Graphics -lR3Shapes -lR2Shapes -lRNBasics -ljpeg -lpng \
        -lGLU -lGLEW -lrt -lboost_system -lboost_filesystem \
        -lQGLViewer -lpcl_common -lpcl_io -lpcl_io_ply
#-L$$PWD/../../GAPS/R3Graphics/ -L$$PWD/../../GAPS/R3Shapes/ -L$$PWD/../../GAPS/RNBasics/ -L$$PWD/../../GAPS/R2Shapes/ \
              #-lR3Graphics -lR3Shapes -lR2Shapes -lRNBasics -ljpeg \
#-lGAPS -ljpeg \
INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../GAPS/
DEPENDPATH += $$PWD/../

unix: PRE_TARGETDEPS += $$PWD/../build/libmemory.a $$PWD/../build/librendering.a

OTHER_FILES += \
    hdr.v.glsl \
    hdr.f.glsl \
    hdr_log.f.glsl \
    hdr_gamma.f.glsl \
    default.v.glsl \
    default.f.glsl \
    averagesample.v.glsl
