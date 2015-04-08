#ifndef HDRVIEWER_H
#define HDRVIEWER_H

#include <QWidget>
#include <QLabel>
#include "qxtspanslider.h"
#include <QComboBox>
#include <QGridLayout>

class HDRViewer : public QWidget
{
    Q_OBJECT
public:
    explicit HDRViewer(QWidget *parent = 0);
    ~HDRViewer();

    void setFloatImage(const float* data, int w, int h, int channels=3);
    void setRGBImage(const unsigned char* data, int w, int h, int channels=3);

    void saveCurrentImage(QString filename);
    void updateImage();
    void updateSize(int w, int h);
signals:

public slots:
    void setMapping(int v);
    void setScale(int lo, int hi);

private:
    char floatToColor(float f) const;
    QLabel* imagelabel;
    QxtSpanSlider* slider;
    QComboBox* tmo;
    QGridLayout* layout;

    enum {
        TMO_LINEAR,
        TMO_LOG,
        TMO_GAMMA22,
    };
    int mapping;
    bool isfloatimage;
    float* image;
    int currw, currh, currch;
    int currsz;
};

#endif // HDRVIEWER_H
