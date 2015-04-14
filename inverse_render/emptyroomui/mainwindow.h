#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "meshmanager.h"
#include "imagemanager.h"
#include "subprocessworker.h"
#include <QProgressBar>
#include <QSettings>

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

    void on_prevImageButton_clicked();

    void on_imageTypeComboBox_currentIndexChanged(int index);

    void on_loadImageButton_clicked();

    void on_nextImageButton_clicked();

    void meshLoaded();
    void imagesLoaded();
    void vertexDataLoaded();

    void on_loadReprojectButton_clicked();

private:
    Ui::MainWindow *ui;
    ImageManager* imgr;
    MeshManager* mmgr;

    std::vector<SubprocessWorker*> workers;
    QProgressBar* progressbar;

    QString meshfilename;
    QString camfilename;

    QSettings* settings;
    QString settingsfilename;

    int imageindex;
    int typeindex;
};

#endif // MAINWINDOW_H
