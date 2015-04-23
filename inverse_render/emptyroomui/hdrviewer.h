#ifndef HDRVIEWER_H
#define HDRVIEWER_H

#include <QWidget>
#include "qxtspanslider.h"
#include "hdrglwidget.h"
#include <QComboBox>
#include <QOpenGLWidget>
#include "hdrimageviewer.h"

class HDRViewer : public QWidget
{
    Q_OBJECT
public:
    explicit HDRViewer(QWidget *parent=0) : QWidget(parent), state(STATE_FIXED) {;}
    explicit HDRViewer(QWidget *parent, QWidget* renderwidget, HDRGlHelper* rendercontrol);
    ~HDRViewer();

protected:
    void init();
signals:
    void updateMappingType(int v);
    void updateRange(int lo, int hi);

protected slots:
    void userEditRange(int lo, int hi);
    void setSuggestRange(int lo, int hi);
    void fixRange(int lo, int hi, int v);
    void showTooltip();

    void notifyUpdateMappingType(int v) { rendercontrol->setMapping(v); emit updateMappingType(v); }
    void notifyUpdateRange(int lo, int hi) { emit updateRange(lo, hi); }

protected:
    HDRGlHelper* rendercontrol;
    QWidget* renderwidget;
    QxtSpanSlider* slider;
    QComboBox* tmo;
    enum {
        STATE_SUGGESTED,
        STATE_FIXED,
        STATE_EDITED,
    };
    int state;
};



class HDRImageViewerWidget : public HDRViewer {
    Q_OBJECT
public:
    explicit HDRImageViewerWidget(QWidget *parent = 0) :
    HDRViewer(parent)
    {
        v = new HDRImageViewer(this);
        v->resize(640,480);
        renderwidget = v;
        rendercontrol = v->getHelper();
        init();
    }

    ~HDRImageViewerWidget() {
        if (v) delete v;
    }

    bool setFloatImage(const float* data, int w, int h, int channels) {
        v->setFloatImage(data, w, h, channels);
    }

    bool setRGBImage(const unsigned char* data, int w, int h, int channels) {
        v->setRGBImage(data, w, h, channels);
    }

protected:
    HDRImageViewer* v;
};

#endif // HDRVIEWER_H
