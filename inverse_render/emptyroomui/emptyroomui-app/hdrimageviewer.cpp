#include "hdrimageviewer.h"

HDRImageViewer::HDRImageViewer(QWidget *parent)
    : HDRGlWidget(parent), currw(0), currh(0), currch(0), currsz(0), image(NULL)
{
}
HDRImageViewer::~HDRImageViewer() {
    if (image) delete [] image;
    glDeleteTextures(1, &tex);
}

bool HDRImageViewer::setFloatImage(const float* data, int w, int h, int channels) {
    if (data == NULL) return false;
    makeCurrent();
    setMinimumSize(w,h);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    if (w != currw || h != currh) {
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, w, h, 0, GL_RGB, GL_FLOAT, 0);
        glBindTexture(GL_TEXTURE_2D, 0);

        currw = w;
        currh = h;
        if (w*h*3 > currsz) {
            currsz = w*h*3;
            if (image) delete [] image;
            image = new float[currsz];
        }
    }
    currch = channels;

    float ihi = 0;
    float ilo = 1e5;
    const float* curr = data;
    float* cf = image;
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            for (int k = 0; k < 3; ++k) {
                if (channels == 3) *(cf++) = *(curr++);
                else *(cf++) = *(curr);
            }
            if (channels != 3) ++curr;
        }
    }
    for (int i = 0; i < w*h*3; ++i) {
        if (image[i] > ihi) ihi = image[i];
        if (image[i] < ilo) ilo = image[i];
    }
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGB, GL_FLOAT, (GLvoid*) image);
    glBindTexture(GL_TEXTURE_2D, 0);
    helper.emitSuggestRange(ilo, ihi);
    update();
    return true;
}

bool HDRImageViewer::setRGBImage(const unsigned char* data, int w, int h, int channels) {
    if (data == NULL) return false;
    makeCurrent();
    setMinimumSize(w,h);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    glBindTexture(GL_TEXTURE_2D, tex);

    if (w != currw || h != currh) {
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, w, h, 0, GL_RGB, GL_FLOAT, 0);
        glBindTexture(GL_TEXTURE_2D, 0);

        currw = w;
        currh = h;
        if (w*h*3 > currsz) {
            currsz = w*h*3;
            if (image) delete [] image;
            image = new float[currsz];
        }
    }
    currch = channels;

    const unsigned char* curr = data;
    float* cf = image;
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            for (int k = 0; k < 3; ++k) {
                if (channels == 3) *(cf++) = *(curr++)/255.;
                else *(cf++) = *(curr)/255.;
            }
            if (channels != 3) ++curr;
        }
    }
    //helper.emitFixParams(0, 1, TMO_LINEAR);
    helper.emitSuggestRange(0,1);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGB, GL_FLOAT, (GLvoid*) image);
    glBindTexture(GL_TEXTURE_2D, 0);
    update();
    return true;
}

void HDRImageViewer::setErrorImage(int w, int h) {
    makeCurrent();
    setMinimumSize(w,h);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    glBindTexture(GL_TEXTURE_2D, tex);

    if (w != currw || h != currh) {
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, w, h, 0, GL_RGB, GL_FLOAT, 0);
        glBindTexture(GL_TEXTURE_2D, 0);

        currw = w;
        currh = h;
        if (w*h*3 > currsz) {
            currsz = w*h*3;
            if (image) delete [] image;
            image = new float[currsz];
        }
    }

    float* cf = image;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            for (int k = 0; k < 3; k++) {
                *cf++ = ((i/20) + (j/20))%2==0?1.f:0.f;
            }
        }
    }

    glBindTexture(GL_TEXTURE_2D, tex);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGB, GL_FLOAT, (GLvoid*) image);
    glBindTexture(GL_TEXTURE_2D, 0);
    update();
}

void HDRImageViewer::_dorender() {
    glViewport(0, 0, currw, currh);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, currw, currh, 0, 1, -1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex);
    glColor3f(1,1,1);
    glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.5f);
        glTexCoord2f(1.0f, 1.0f);
        glVertex3f(currw, 0.0f, 0.5f);
        glTexCoord2f(1.0f, 0.0f);
        glVertex3f(currw, currh, 0.5f);
        glTexCoord2f(0.0f, 0.0f);
        glVertex3f(0.0f, currh, 0.5f);
    glEnd();
    glColor3f(100,0,0);
    glBegin(GL_LINES);
        for (int i = 0; i < lines.size(); i+=4) {
            glVertex3f(lines[i], currh-lines[i+1], 0);
            glVertex3f(lines[i+2], currh-lines[i+3], 0);
        }
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);
}

void HDRImageViewer::_dosetup() {
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, 640, 480, 0, GL_RGB, GL_FLOAT, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void HDRImageViewer::_doresize(int w, int h) {
}
