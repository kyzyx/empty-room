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
            ../imagemanager.h \
            ../meshmanager.h \
    hdrviewer.h \
    qxtspanslider.h \
    subprocessworker.h \
    hdrglwidget.h \
    hdrimageviewer.h

FORMS    += mainwindow.ui

unix: LIBS += -L$$PWD/../build/ \
        -L$$PWD/../../GAPS/R3Graphics/ -L$$PWD/../../GAPS/R3Shapes/ -L$$PWD/../../GAPS/RNBasics/ -L$$PWD/../../GAPS/R2Shapes/ \
        -lmemory \
        -lR3Graphics -lR3Shapes -lR2Shapes -lRNBasics -ljpeg \
        -lGLU -lGLEW -lrt -lboost_system -lboost_filesystem \
        -lQGLViewer
#-L$$PWD/../../GAPS/R3Graphics/ -L$$PWD/../../GAPS/R3Shapes/ -L$$PWD/../../GAPS/RNBasics/ -L$$PWD/../../GAPS/R2Shapes/ \
              #-lR3Graphics -lR3Shapes -lR2Shapes -lRNBasics -ljpeg \
#-lGAPS -ljpeg \
INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../GAPS/
DEPENDPATH += $$PWD/../

unix: PRE_TARGETDEPS += $$PWD/../build/libmemory.a

OTHER_FILES += \
    hdr.v.glsl \
    hdr.f.glsl
