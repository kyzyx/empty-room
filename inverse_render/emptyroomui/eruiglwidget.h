#ifndef ERUIGLWIDGET_H
#define ERUIGLWIDGET_H

#include <QGLWidget>

class ERUIGLWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit ERUIGLWidget(QWidget *parent = 0);
protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void draw();

signals:

public slots:

};

#endif // ERUIGLWIDGET_H
