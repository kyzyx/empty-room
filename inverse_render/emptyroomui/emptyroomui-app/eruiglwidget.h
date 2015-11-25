#ifndef ERUIGLWIDGET_H
#define ERUIGLWIDGET_H

#include <QCursor>
#include <QOpenGLWidget>
#include "hdrglwidget.h"
#include "datamanager/meshmanager.h"
#include "hdrviewer.h"
#include "datamanager/imagemanager.h"
#include "roommodel/roommodel.h"
#include "rendermanager.h"

class ERUIRenderOptions : public QObject {
Q_OBJECT
public:
    ERUIRenderOptions(QOpenGLWidget* parent=NULL) :
        renderer(NULL),
        qglw(parent),
        renderCameras(true),
        renderCurrentCamera(true),
        renderCameraTrajectory(true),
        renderMesh(true),
        renderRoom(false),
        renderWfHistogram(false),
        currentCamera(0),
        cameraRenderFormat(CAMRENDER_FRUSTUM),
        meshRenderFormat(VIEW_DEFAULT),
        overlayLoThreshold(0), overlayHiThreshold(1000)
    { renderOverlay.resize(NUM_VIEW_TYPES, false);}

public:
    bool shouldRenderAnyCameras() const {
        return renderCameras || renderCurrentCamera;
    }

    int getCameraFormat(bool isCurrent=false) const {
        if (renderCameras || (renderCurrentCamera && isCurrent)) {
            return cameraRenderFormat;
        } else {
            return CAMRENDER_NONE;
        }
    }
    int getCurrentCamera() const { return currentCamera; }
    int getMeshRenderFormat() const { return meshRenderFormat; }
    int getLowerThreshold() const { return overlayLoThreshold; }
    int getUpperThreshold() const { return overlayHiThreshold; }
    int getOverlayLabelIndex() const { return overlayIndex; }
    int getWfThreshold() const { return wfthreshold; }
    bool shouldRenderCameraTrajectory() const { return renderCameraTrajectory; }
    bool shouldOverlay(int n) const { return renderOverlay[n]; }
    bool shouldRenderMesh() const { return renderMesh; }
    bool shouldRenderRoom() const { return renderRoom; }
    bool shouldRenderWfHistogram() const { return renderWfHistogram; }

    void setRenderManager(RenderManager* rm) { renderer = rm; }
    enum {
        CAMRENDER_NONE=0,
        CAMRENDER_AXES,
        CAMRENDER_FRUSTUM,
    };
protected:
    RenderManager* renderer;
    QOpenGLWidget* qglw;
    bool renderCameras;
    bool renderCurrentCamera;
    bool renderCameraTrajectory;
    bool renderMesh;
    bool renderRoom;
    bool renderWfHistogram;
    std::vector<bool> renderOverlay;

    int currentCamera;
    int cameraRenderFormat;
    int meshRenderFormat;
    int overlayIndex;
    int overlayLoThreshold, overlayHiThreshold;
    int wfthreshold;

public slots:
    void setRenderWfHistogram(bool shouldRenderHistogram) {
        renderWfHistogram = shouldRenderHistogram;
        if (qglw) qglw->update();
    }

    void setRenderCameras(bool shouldRenderCamera) {
        renderCameras = shouldRenderCamera;
        if (qglw) qglw->update();
    }
    void setRenderCurrentCamera(bool shouldRenderCurrentCamera) {
        renderCurrentCamera = shouldRenderCurrentCamera;
        if (qglw) qglw->update();
    }
    void setRenderMesh(bool shouldRenderMesh) {
        renderMesh = shouldRenderMesh;
        if (qglw) qglw->update();
    }
    void setRenderRoom(bool shouldRenderRoom) {
        renderRoom = shouldRenderRoom;
        if (qglw) qglw->update();
    }
    void setRenderTrajectory(bool shouldRenderTrajectory) {
        renderCameraTrajectory = shouldRenderTrajectory;
        if (qglw) qglw->update();
    }

    void setCameraRenderFormat(int camformat) {
        cameraRenderFormat = camformat;
        if (qglw) qglw->update();
    }
    void setWfThreshold(int threshold) {
        wfthreshold = threshold;
        if (qglw) qglw->update();
    }

    void setMeshRenderFormat(int meshformat) {
        meshRenderFormat = meshformat;
        if (meshRenderFormat == VIEW_AVERAGE && renderer)
            renderer->precalculateAverageSamples();
        else if (meshRenderFormat == VIEW_SINGLEIMAGE && renderer)
            renderer->precalculateSingleImage(currentCamera);
        if (qglw) qglw->update();
    }
    void showSelected(bool show=true) {
        renderOverlay[VIEW_SELECTOVERLAY] = show;
        if (qglw) qglw->update();
    }

    void setOverlay(int overlay) {
        if (overlay) {
            renderOverlay[VIEW_LABELOVERLAY] = true;
            overlayIndex = overlay;
        } else {
            renderOverlay[VIEW_LABELOVERLAY] = false;
        }
        if (qglw) qglw->update();
    }
    void setOverlayThresholded(bool overlay) {
        if (overlay) {
            if (renderer && !renderer->hasPrecalculatedColors()) renderer->precalculateAverageSamples();
            renderOverlay[VIEW_THRESHOLD] = true;
        } else {
            renderOverlay[VIEW_THRESHOLD] = false;
        }
        if (qglw) qglw->update();
    }

    void setLowerThreshold(int t) {
        overlayLoThreshold = t;
        if (qglw) qglw->update();
    }
    void setUpperThreshold(int t) {
        overlayHiThreshold = t;
        if (qglw) qglw->update();
    }
    void setCurrentCamera(int n) {
        currentCamera = n;
        if (meshRenderFormat == VIEW_SINGLEIMAGE && renderer) renderer->precalculateSingleImage(n);
        if (qglw) qglw->update();
    }
};

class ERUIGLWidget : public HDRQGlViewerWidget
{
    Q_OBJECT
public:
    explicit ERUIGLWidget(QWidget *parent = 0);
    ~ERUIGLWidget();

    RenderManager* renderManager() { makeCurrent(); return &rendermanager; }
    ERUIRenderOptions* renderOptions() { return &renderoptions; }

    void updateMeshAuxiliaryData();
    void setRoomModel(roommodel::RoomModel* model);
    void setupMeshColors();
    void setMeshManager(MeshManager* meshmanager);

    void lookThroughCamera(const CameraParams* cam);
    void setupCameras(ImageManager* imgr);
    void setupCameras(std::vector<CameraParams>& cams);
    void setCameraColor(int i, RNRgb rgb) { camcolors[i] = rgb; }
    void setOrientation(roommodel::RoomModel* room) {
        orientationroommodel = room;
    }
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent* e);

    void updateCursor();

    void setVertexSelectMode(int mode) { vertexselectmode = mode; }
    void setVertexBrushSize(int size) { vertexbrushsize = size; updateCursor(); }
    int getVertexBrushSize() const { return vertexbrushsize; }
    void setInteractionMode(int mode);

    void computeWallFindingHistogram(double resolution);

    int interactionmode;
    enum {
        INTERACTIONMODE_TRACKBALL,
        INTERACTIONMODE_SELECT,
        NUMINTERACTIONMODES,
    };
protected:
    virtual void _dosetup();
    virtual void draw();
    virtual void init();
    virtual void drawWithNames();
    void postSelection(const QPoint &point);
    virtual QString helpString() const;

    void renderCamera(const CameraParams& cam, bool id_only=false);
    void renderHistogram();

    RenderManager rendermanager;
    std::vector<CameraParams> cameras;
    std::vector<int> camids;
    std::vector<RNRgb> camcolors;
    roommodel::RoomModel* orientationroommodel;

    ERUIRenderOptions renderoptions;
    int selectedCamera;

    int vertexselectmode, vertexbrushsize;

    int* grid;
    int gw, gh, gmax;
    double gres;
    QCursor* cursor;
    QCursor* defaultcursor;
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
    ERUIRenderOptions* renderOptions() { return v->renderOptions(); }

    RenderManager* renderManager() { return v->renderManager(); }
    void setMeshManager(MeshManager* manager) {
        v->setMeshManager(manager);
    }
    void setupMeshColors() {
        v->setupMeshColors();
    }
    void setupCameras(ImageManager* imgr) {
        v->setupCameras(imgr);
    }
    void setupCameras(std::vector<CameraParams>& cams) {
        v->setupCameras(cams);
    }
    void setRoomModel(roommodel::RoomModel* model) {
        v->setRoomModel(model);
    }
    void lookThroughCamera(const CameraParams* cam) {
        v->lookThroughCamera(cam);
    }
    void updateMeshAuxiliaryData() {
        v->updateMeshAuxiliaryData();
    }
    void computeWallFindingHistogram(double resolution) {
        v->computeWallFindingHistogram(resolution);
    }
    void setOrientation(roommodel::RoomModel* room) {
        v->setOrientation(room);
    }
    void setCameraColor(int i, RNRgb rgb) {
        v->setCameraColor(i, rgb);
    }
    void setVertexSelectMode(int mode) {
        v->setVertexSelectMode(mode);
    }
    void setVertexBrushSize(int size) {
        v->setVertexBrushSize(size);
    }
    int getVertexBrushSize() {
        return v->getVertexBrushSize();
    }
    void setInteractionMode(int mode) {
        v->setInteractionMode(mode);
    }

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
