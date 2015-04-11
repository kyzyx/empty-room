#ifndef ERUIGLWIDGET_H
#define ERUIGLWIDGET_H

#include <QGLWidget>
#include "hdrglwidget.h"
#include "meshmanager.h"
#include "hdrviewer.h"

class ERUIGLWidget : public HDRQGlViewerWidget
{
    Q_OBJECT
public:
    explicit ERUIGLWidget(QWidget *parent = 0);
    void setMeshManager(MeshManager* manager);
    void setupMeshColors();
protected:
    virtual void draw();
    virtual void init();
    virtual QString helpString() const;

    void setupMeshGeometry();


    MeshManager* mmgr;

    bool hasColors, hasGeometry;
    GLuint vbo, ibo, cbo;
signals:

public slots:

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
        init();
    }

    ~ERUIGLViewer() {
        if (v) delete v;
    }

    void setMeshManager(MeshManager* manager) { v->setMeshManager(manager); setSuggestRange(0,1); }
    void setupMeshColors() { v->setupMeshColors(); }

protected:
    ERUIGLWidget* v;
};



#endif // ERUIGLWIDGET_H
