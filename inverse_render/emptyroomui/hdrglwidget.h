#ifndef HDRGLWIDGET_H
#define HDRGLWIDGET_H

#include <QGLWidget>

#include <cmath>


enum {
    TMO_LINEAR,
    TMO_LOG,
    TMO_GAMMA22,
};

class HDRGlWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit HDRGlWidget(QWidget *parent = 0);
    ~HDRGlWidget();

    static int LOGTOLIN(double v)
    {
        if (v < 1e-5) return 0;
        if (v > 1e5) return 100;
        return log10(v)*10+50;
    }
    static double LINTOLOG(int v) {
        return pow(10,(v-50)/10.);
    }
protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    virtual void _dosetup() {;}
    virtual void _dorender() = 0;
    virtual void _doresize(int width, int height) {;}

    int mapping;
    float mini, maxi;

    GLuint fbo, fbo_z, fbo_tex;

    GLuint vbo_fbo_vertices;
    GLuint program_postproc, attribute_v_coord_postproc, uniform_fbo_texture;
signals:
    void suggestRange(int lo, int hi);
    void fixParams(int lo, int hi, int v);

public slots:
    void setMapping(int v);
    void setScale(int lo, int hi);
};

#endif // HDRGLWIDGET_H
