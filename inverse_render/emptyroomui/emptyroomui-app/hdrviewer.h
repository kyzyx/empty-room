#ifndef HDRVIEWER_H
#define HDRVIEWER_H

#include <QWidget>
#include "qxtspanslider.h"
#include "hdrglwidget.h"
#include <QComboBox>
#include <QOpenGLWidget>
#include <QCheckBox>
#include "hdrimageviewer.h"

class HDRViewer : public QWidget
{
    Q_OBJECT
public:
    explicit HDRViewer(QWidget *parent=0) : QWidget(parent), currentindex(0), state(STATE_FIXED) {;}
    explicit HDRViewer(QWidget *parent, QWidget* renderwidget, HDRGlHelper* rendercontrol);
    ~HDRViewer();

    void copySettings(HDRViewer* v);
    int getCurrentMapping() const { return currentindex; }
    double getLowerBound() { return HDRGlHelper::LINTOLOG(currsettings.lo); }
    double getUpperBound() { return HDRGlHelper::LINTOLOG(currsettings.hi); };
protected:
    void init();
signals:
    void updateMappingType(int v);
    void updateRange(int lo, int hi);
public slots:
    void saveSettings();
    void saveSettings(int index);
    void loadSettings(int index);
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
    QCheckBox* red_check;
    QCheckBox* green_check;
    QCheckBox* blue_check;

    int currentindex;
    class Settings {
        public:
        Settings()
            :lo(0), hi(0), idx(0), r(true), g(true), b(true) {;}
        Settings(int l, int h, int i, bool rr, bool gg, bool bb)
            : lo(l), hi(h), idx(i),
              r(rr), g(gg), b(bb)
            {;}
        int lo, hi, idx;
        bool r, g, b;
    };
    Settings currsettings;
    std::vector<Settings> savedsettings;

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
        return v->setFloatImage(data, w, h, channels);
    }

    bool setRGBImage(const unsigned char* data, int w, int h, int channels) {
        return v->setRGBImage(data, w, h, channels);
    }
    void setErrorImage(int w, int h) {
        return v->setErrorImage(w, h);
    }

    void addLine(int x1, int y1, int x2, int y2) {
        v->addLine(x1,y1,x2,y2);
    }
    void clearLines() {
        v->clearLines();
    }

protected:
    HDRImageViewer* v;
};

#endif // HDRVIEWER_H
