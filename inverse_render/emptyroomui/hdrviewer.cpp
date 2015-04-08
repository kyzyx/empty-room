#include "hdrviewer.h"
#include <QFile>
#include <QImage>
#include <QPixmap>

#include <cmath>

int LOGTOLIN(double v)
{
    if (v < 1e-5) return 0;
    if (v > 1e5) return 100;
    return log10(v)*10+50;
}
double LINTOLOG(int v) {
    return pow(10,(v-50)/10.);
}

HDRViewer::HDRViewer(QWidget *parent) :
    QWidget(parent), isfloatimage(false), image(NULL), mapping(TMO_LINEAR), currsz(0)
{
    imagelabel = new QLabel(this);
    imagelabel->resize(640,480);

    slider = new QxtSpanSlider(Qt::Horizontal, this);
    slider->setEnabled(false);
    slider->setMinimum(0);
    slider->setMaximum(100);
    slider->setValue(LOGTOLIN(1));
    connect(slider, SIGNAL(spanChanged(int, int)), this, SLOT(setScale(int, int)));

    tmo = new QComboBox(this);
    tmo->insertItem(TMO_LINEAR, "Linear Mapping");
    tmo->insertItem(TMO_LOG, "Logarithmic Mapping");
    tmo->insertItem(TMO_GAMMA22, "Gamma 2.2");
    tmo->setCurrentIndex(0);
    tmo->setEnabled(false);
    connect(tmo, SIGNAL(activated(int)), this, SLOT(setMapping(int)));

    layout = new QGridLayout(this);
    layout->addWidget(imagelabel,0,0,1,2);
    layout->addWidget(slider,1,0);
    layout->addWidget(tmo,1,1);
    layout->setColumnStretch(0,2);
    layout->setColumnStretch(1,1);
    setLayout(layout);
}
HDRViewer::~HDRViewer() {
    if (image) delete [] image;
    delete imagelabel;
    delete slider;
    delete tmo;
    delete layout;
}

void HDRViewer::setMapping(int v) {
    mapping = v;
    updateImage();
}

void HDRViewer::setScale(int lo, int hi) {
    updateImage();
}

void HDRViewer::updateSize(int w, int h) {
    imagelabel->resize(w,h);
    resize(w, h + slider->height() + layout->margin());
}


char HDRViewer::floatToColor(float f) const {
    double lo = LINTOLOG(slider->lowerValue());
    double hi = LINTOLOG(slider->upperValue());
    int r = 0;
    if (mapping == TMO_LINEAR) {
        r = (int) (255*(f-lo)/(hi-lo));
    } else if (mapping == TMO_LOG) {
        r = (int) (255*(log(f) - log(lo))/(log(hi) - log(lo)));
    } else if (mapping == TMO_GAMMA22) {
        r = (int) (255*pow((f-lo)/(hi-lo), 2.2));
    }
    if (r > 255) return 255;
    if (r < 0) return 0;
    return (char) r;
}

void HDRViewer::saveCurrentImage(QString filename) {
    QFile file(filename);
    file.open(QIODevice::WriteOnly);
    imagelabel->pixmap()->save(&file, "PNG");
}

void HDRViewer::updateImage() {
    if (!isfloatimage) return;
    QImage im(currw,currh,QImage::Format_RGB888);

    const float* curr = image;
    for (int i = 0; i < currh; ++i) {
        for (int j = 0; j < currw; ++j) {
            QRgb c;
            if (currch == 1) {
                char v = floatToColor(*(curr++));
                c = qRgb(v,v,v);
            } else if (currch == 3) {
                char b = floatToColor(*(curr++));
                char g = floatToColor(*(curr++));
                char r = floatToColor(*(curr++));
                c = qRgb(b,g,r);
            }
            im.setPixel(j,currh-i-1,c);
        }
    }
    imagelabel->setPixmap(QPixmap::fromImage(im));
}

void HDRViewer::setFloatImage(const float* data, int w, int h, int channels) {
    if (data == NULL) return;
    currw = w;
    currh = h;
    if (w*h*channels > currsz) {
        currsz = w*h*channels;
        if (image) delete [] image;
        image = new float[currsz];
    }
    currch = channels;
    isfloatimage = true;
    memcpy(image, data, sizeof(float)*w*h*channels);
    float maxi = 0;
    float mini = 1e5;
    for (int i = 0; i < w*h*channels; ++i) {
        if (image[i] > maxi) maxi = image[i];
        if (image[i] < mini) mini = image[i];
    }
    tmo->setEnabled(true);
    slider->setEnabled(true);
    slider->setUpperValue(LOGTOLIN(mini));
    slider->setLowerValue(LOGTOLIN(maxi));
    updateSize(w,h);
    updateImage();
}

void HDRViewer::setRGBImage(const unsigned char* data, int w, int h, int channels) {
    if (data == NULL) return;
    QImage im(w,h,QImage::Format_RGB888);
    isfloatimage = false;
    const unsigned char* curr = data;
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            QRgb c;
            if (channels == 1) {
                c = qRgb(*curr, *curr, *curr);
                ++curr;
            } else if (channels == 3) {
                char b = *(curr++);
                char g = *(curr++);
                char r = *(curr++);
                c = qRgb(r,g,b);
            }
            im.setPixel(j,h-i-1,c);
        }
    }
    updateSize(w,h);
    imagelabel->setPixmap(QPixmap::fromImage(im));
    tmo->setEnabled(false);
    slider->setEnabled(false);
}
