#ifndef ERUIGLWIDGET_H
#define ERUIGLWIDGET_H

#include <QGLWidget>
#include <QGLViewer/qglviewer.h>
#include "meshmanager.h"

class ERUIGLWidget : public QGLViewer
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

#endif // ERUIGLWIDGET_H
