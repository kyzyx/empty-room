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

    void addLine(int x1, int y1, int x2, int y2) {
        lines.push_back(x1);
        lines.push_back(y1);
        lines.push_back(x2);
        lines.push_back(y2);
    }

    void clearLines() { lines.clear(); }
protected:
    virtual void _dosetup();
    virtual void _doresize(int width, int height);

    int currw, currh, currch;
    int currsz;
    float* image;

    std::vector<int> lines;

    GLuint tex;
protected slots:
    virtual void _dorender();

};
#endif // HDRIMAGEVIEWER_H
