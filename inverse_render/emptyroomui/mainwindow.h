#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "meshmanager.h"
#include "imagemanager.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *event);
    void updateImage(int idx, int type);

private slots:
    void on_actionQuit_triggered();

    void on_actionOpen_Mesh_triggered();

    void on_actionOpen_Images_triggered();

private:
    Ui::MainWindow *ui;
    ImageManager* imgr;
    MeshManager* mmgr;


    int imageindex;
    int typeindex;
};

#endif // MAINWINDOW_H
