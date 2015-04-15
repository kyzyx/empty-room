#ifndef ERUIGLWIDGET_H
#define ERUIGLWIDGET_H

#include <QGLWidget>
#include "hdrglwidget.h"
#include "meshmanager.h"
#include "hdrviewer.h"
#include "imagemanager.h"

class ERUIGLWidget : public HDRQGlViewerWidget
{
    Q_OBJECT
public:
    explicit ERUIGLWidget(QWidget *parent = 0);
    void setMeshManager(MeshManager* manager);
    void setupMeshColors();
    void setupCameras(ImageManager* imgr);

    void lookThroughCamera(const CameraParams* cam);

protected:
    virtual void draw();
    virtual void init();
    virtual void drawWithNames();
    void postSelection(const QPoint &point);
    virtual QString helpString() const;

    void setupMeshGeometry();

    void renderCamera(const CameraParams& cam, bool id_only=false);

    std::vector<CameraParams> cameras;
    std::vector<int> camids;
    MeshManager* mmgr;

    bool hasColors, hasGeometry;

    // Camera rendering variables
    enum {
        CAMRENDER_NONE,
        CAMRENDER_AXES,
        CAMRENDER_FRUSTUM,
    };
    bool renderCurrent;
    int cameraRenderFormat;
    int selectedCamera;

    // OpenGL buffer ids
    GLuint vbo, ibo, cbo;
signals:
    void cameraSelected(int cameraindex);
public slots:
    void highlightCamera(int cameraindex);
    void showCameras(bool show);
    void showCurrentCamera(bool show);
};

class ERUIGLViewer : public HDRViewer {
    Q_OBJECT
public:
    explicit ERUIGLViewer(QWidget *parent = 0) :
    HDRViewer(parent)
    {
        v = new ERUIGLWidget(this);
        renderwidget = v;
        rendercontrol = v->getHelper();
        connect(v, SIGNAL(cameraSelected(int)), this, SLOT(emitCameraSelected(int)));
        init();
    }
    ~ERUIGLViewer() {
        if (v) delete v;
    }

    void setMeshManager(MeshManager* manager) {
        v->setMeshManager(manager);
    }
    void setupMeshColors() { v->setupMeshColors(); }
    void setupCameras(ImageManager* imgr) { v->setupCameras(imgr); }
    void lookThroughCamera(const CameraParams* cam) { v->lookThroughCamera(cam); }

signals:
    void cameraSelected(int cameraindex);
protected slots:
    void emitCameraSelected(int cameraindex) { emit cameraSelected(cameraindex); }
public slots:
    void highlightCamera(int cameraindex) { v->highlightCamera(cameraindex); }
    void showCameras(bool show) { v->showCameras(show); }
    void showCurrentCamera(bool show) { v->showCurrentCamera(show); }

protected:
    ERUIGLWidget* v;
};



#endif // ERUIGLWIDGET_H
