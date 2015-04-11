#ifndef HDRIMAGEVIEWER_H
#define HDRIMAGEVIEWER_H
#include "hdrglwidget.h"

class HDRImageViewer : public HDRGlWidget {
    Q_OBJECT
public:
    explicit HDRImageViewer(QWidget *parent = 0);
    ~HDRImageViewer();

    bool setFloatImage(const float* data, int w, int h, int channels);
    bool setRGBImage(const unsigned char* data, int w, int h, int channels);
protected:
    virtual void _dosetup();
    virtual void _doresize(int width, int height);

    int currw, currh, currch;
    int currsz;
    float* image;

    GLuint tex;
protected slots:
    virtual void _dorender();

};
#endif // HDRIMAGEVIEWER_H
