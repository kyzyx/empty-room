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

    void loadVertexData(QString meshfile, QString datafile);
private slots:
    void updateImage(int idx)         { updateImage(idx, typeindex); }
    void on_prevImageButton_clicked() { updateImage(--imageindex, typeindex); }
    void on_nextImageButton_clicked() { updateImage(++imageindex, typeindex); }
    void on_imageTypeComboBox_currentIndexChanged(int index) { updateImage(imageindex, index); }
    void on_loadImageButton_clicked();

    void on_actionQuit_triggered();

    void on_actionOpen_Mesh_triggered();
    void on_actionOpen_Images_triggered();

    void on_loadReprojectButton_clicked();

    void meshLoaded();
    void imagesLoaded();
    void allLoaded();
    void vertexDataLoaded();

    void on_actionLoad_Last_Mesh_Camera_File_triggered();
    void on_actionLoad_Last_Intermediates_triggered();

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
