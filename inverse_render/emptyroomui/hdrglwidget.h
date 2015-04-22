#ifndef HDRGLWIDGET_H
#define HDRGLWIDGET_H

#include <QGLWidget>
#include <QGLViewer/qglviewer.h>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/function.hpp>


enum {
    TMO_LINEAR,
    TMO_LOG,
    TMO_GAMMA22,
    TMO_BITWISE_INT,
    NUM_TMOS,
};

class HDRGlHelper : public QObject
{
    Q_OBJECT
public:
    HDRGlHelper();
    ~HDRGlHelper();

    static int LOGTOLIN(double v)
    {
        if (v < 1e-5) return 0;
        if (v > 1e5) return 100;
        return log10(v)*10+50;
    }
    static double LINTOLOG(int v) {
        return pow(10,(v-50)/10.);
    }

    void initializeHelper();
    void paintHelper();
    void resizeHelper(int width, int height);
    void emitSuggestRange(float lo, float hi) { emit suggestRange(LOGTOLIN(lo), LOGTOLIN(hi)); }
    void emitFixParams(float lo, float hi, int v) { emit fixParams(LOGTOLIN(lo), LOGTOLIN(hi), v); }

    void renderToTexture(boost::function<void()> f);
    void readFromTexture(int x, int y, int w, int h, float* data);

    void setRenderFunction(boost::function<void()> f) { renderfunc = f; }

protected:
    boost::function<void()> renderfunc;
    int mapping;
    float mini, maxi;

    int currw, currh;

    GLuint fbo, fbo_z, fbo_tex;

    GLuint vbo_fbo_vertices;

    enum {
        UNIFORM_FBO_TEXTURE,
        UNIFORM_HDR_BOUNDS,
        NUM_UNIFORMS
    };
    struct HDRShaderProgram {
        GLuint progid, v_coord;
        GLuint uniform_ids[NUM_UNIFORMS];
    };
    HDRShaderProgram progs[NUM_TMOS];

public slots:
    void setMapping(int v);
    void setScale(int lo, int hi);

signals:
    void suggestRange(int lo, int hi);
    void fixParams(int lo, int hi, int v);
    void update();
    void _render();

};

class HDRGlWidget : public QGLWidget {
    Q_OBJECT
public:
    explicit HDRGlWidget(QWidget *parent = 0) : QGLWidget(parent)
    {
        connect(&helper, SIGNAL(update()), this, SLOT(callupdate()));
        helper.setRenderFunction(boost::bind(&HDRGlWidget::calldorender, this));
    }

    HDRGlHelper* getHelper() { return &helper; }
protected:
    virtual void _dosetup() {;}
    void calldorender() { _dorender(); }
    virtual void _dorender() = 0;
    virtual void _doresize(int width, int height) { ;}
    void initializeGL() { helper.initializeHelper(); _dosetup(); }
    void paintGL() { helper.paintHelper(); }
    void resizeGL(int width, int height) { helper.resizeHelper(width, height); _doresize(width, height);}
    HDRGlHelper helper;
protected slots:
        void callupdate() { updateGL(); }
};

class HDRQGlViewerWidget : public QGLViewer {
    Q_OBJECT
    public:
    explicit HDRQGlViewerWidget(QWidget* parent = 0) : QGLViewer(parent)
    {
        connect(&helper, SIGNAL(update()), this, SLOT(callupdate()));
        helper.setRenderFunction(boost::bind(&HDRQGlViewerWidget::calldorender, this));
    }

    HDRGlHelper* getHelper() { return &helper; }

    static int RGBToIndex(float* rgb);
    static void IndexToRGB(int i, float* rgb);
protected:
    virtual void _dosetup() {;}
    void calldorender() { _dorender(); }
    virtual void _dorender() { QGLViewer::paintGL(); }
    virtual void _doresize(int width, int height) {;}
    void initializeGL() {
        QGLViewer::initializeGL();
        helper.initializeHelper();
        _dosetup();
    }
    virtual void select(const QPoint& point);
    void paintGL() {
        helper.paintHelper();
    }
    void resizeGL(int width, int height);

    HDRGlHelper helper;
protected slots:
        void callupdate() { updateGL(); }

};

#endif // HDRGLWIDGET_H
