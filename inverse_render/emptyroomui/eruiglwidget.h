#ifndef ERUIGLWIDGET_H
#define ERUIGLWIDGET_H

#include <QGLWidget>
#include "hdrglwidget.h"
#include "meshmanager.h"
#include "hdrviewer.h"
#include "imagemanager.h"
#include "roommodel.h"

enum {
    VIEW_GEOMETRY,
    VIEW_AVERAGE,
    VIEW_IMAGEID,
    NUM_VIEWOPTIONS
};

class ERUIRenderOptions : public QObject {
Q_OBJECT
public:
    ERUIRenderOptions(QGLWidget* parent=NULL) :
        qglw(parent),
        renderCameras(true),
        renderCurrentCamera(true),
        renderMesh(true),
        renderRoom(false),
        cameraRenderFormat(CAMRENDER_FRUSTUM)
    {;}

public:
    bool shouldRenderAnyCameras() {
        return renderCameras || renderCurrentCamera;
    }

    int getCameraFormat(bool isCurrent=false) {
        if (renderCameras || (renderCurrentCamera && isCurrent)) {
            return cameraRenderFormat;
        } else {
            return CAMRENDER_NONE;
        }
    }
    bool shouldRenderMesh() { return renderMesh; }
    bool shouldRenderRoom() { return renderRoom; }

    enum {
        CAMRENDER_NONE=0,
        CAMRENDER_AXES,
        CAMRENDER_FRUSTUM,
    };
protected:
    QGLWidget* qglw;
    bool renderCameras;
    bool renderCurrentCamera;
    bool renderMesh;
    bool renderRoom;

    int cameraRenderFormat;

public slots:
    void setRenderCameras(bool shouldRenderCamera) {
        renderCameras = shouldRenderCamera;
        if (qglw) qglw->updateGL();
    }
    void setRenderCurrentCamera(bool shouldRenderCurrentCamera) {
        renderCurrentCamera = shouldRenderCurrentCamera;
        if (qglw) qglw->updateGL();
    }
    void setRenderMesh(bool shouldRenderMesh) {
        renderMesh = shouldRenderMesh;
        if (qglw) qglw->updateGL();
    }
    void setRenderRoom(bool shouldRenderRoom) {
        renderRoom = shouldRenderRoom;
        if (qglw) qglw->updateGL();
    }
    void setCameraRenderFormat(int camformat) {
        cameraRenderFormat = camformat;
        if (qglw) qglw->updateGL();
    }
};

class ERUIGLWidget : public HDRQGlViewerWidget
{
    Q_OBJECT
public:
    explicit ERUIGLWidget(QWidget *parent = 0);
    ~ERUIGLWidget();
    void setMeshManager(MeshManager* manager);
    void setupMeshColors();
    void setupCameras(ImageManager* imgr);

    void lookThroughCamera(const CameraParams* cam);
    void setRoomModel(roommodel::RoomModel* model);
    ERUIRenderOptions* renderOptions() { return &renderoptions; }
protected:
    virtual void draw();
    virtual void init();
    virtual void drawWithNames();
    void postSelection(const QPoint &point);
    virtual QString helpString() const;

    void setupMeshGeometry();
    void setupRoomGeometry(roommodel::RoomModel* model);

    void renderRoom();
    void renderMesh();
    void renderCamera(const CameraParams& cam, bool id_only=false);

    std::vector<CameraParams> cameras;
    std::vector<int> camids;
    MeshManager* mmgr;
    roommodel::RoomModel* room;
    R3Vector trans;

    bool hasColors, hasGeometry;

    ERUIRenderOptions renderoptions;
    int selectedCamera;
    int numroomtriangles;

    // OpenGL buffer ids
    GLuint vbo, ibo, cbo;
    GLuint roomvbo, roomcbo;
    GLuint sampletex[3];
    GLuint vaoid;


    GLuint progids[NUM_VIEWOPTIONS];
    GLuint uniformids[NUM_VIEWOPTIONS][5];
signals:
    void cameraSelected(int cameraindex);
public slots:
    void highlightCamera(int cameraindex);
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
    ERUIRenderOptions* renderOptions() { return v->renderOptions(); }
    void setupMeshColors() { v->setupMeshColors(); }
    void setupCameras(ImageManager* imgr) { v->setupCameras(imgr); }
    void lookThroughCamera(const CameraParams* cam) { v->lookThroughCamera(cam); }
    void setRoomModel(roommodel::RoomModel* model) { v->setRoomModel(model); }
signals:
    void cameraSelected(int cameraindex);
protected slots:
    void emitCameraSelected(int cameraindex) { emit cameraSelected(cameraindex); }
public slots:
    void highlightCamera(int cameraindex) { v->highlightCamera(cameraindex); }

protected:
    ERUIGLWidget* v;
};



#endif // ERUIGLWIDGET_H
