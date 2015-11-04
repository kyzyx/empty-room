#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "datamanager/meshmanager.h"
#include "datamanager/imagemanager.h"
#include "subprocessworker.h"
#include "roommodel/roommodel.h"
#include <QProgressBar>
#include <QSettings>
#include <QTemporaryFile>

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

    void loadVertexSampleData(QString meshfile, QString datafile);
    void loadVertexLabelData(QString meshflie, QString datafile);
    void loadFloorPlan(QString floorplanfile);
private slots:
    // Interface
    void updateImage(int idx)         { updateImage(idx, typeindex); }
    void on_prevImageButton_clicked() { updateImage(--imageindex, typeindex); }
    void on_nextImageButton_clicked() { updateImage(++imageindex, typeindex); }
    void on_imageTypeComboBox_currentIndexChanged(int index) { updateImage(imageindex, index); }
    void on_loadImageButton_clicked();

    void showwfrestooltip(int v);
    void showwfthresholdtooltip(int v);

    void on_showMeshCheckbox_toggled(bool checked);
    void on_showRoomCheckbox_toggled(bool checked);

    void on_showCameraCheckbox_toggled(bool checked);
    void onCameraSelection(int selected);
    void on_showCurrentCameraCheckbox_toggled(bool checked);
    void on_autoLookCheckbox_toggled(bool checked);
    void on_showTrajectoryCheckbox_toggled(bool checked);

    // Data communications
    void addLine(QString l);

    // Actions
    void on_actionQuit_triggered();

    void on_reprojectButton_clicked();
    void on_wallfindButton_clicked();
    void on_DebugWallfindingButton_clicked();
    void on_wf_resolutionSlider_valueChanged(int value);
    void on_computeLabelImagesButton_clicked();
    void on_edgeFilterButton_clicked();
    void on_commitLightsButton_clicked();
    void on_computeVerticalLinesButton_clicked();
    void on_hemicubeButton_clicked();

    // Load/Save
    void on_actionOpen_Mesh_triggered();
    void on_actionOpen_Images_triggered();
    void on_actionLoad_Last_Mesh_Camera_File_triggered();
    void on_actionLoad_Last_Intermediates_triggered();
    void on_actionSave_Reprojection_Results_triggered();
    void on_actionSave_Wallfinding_Floor_Plan_triggered();
    void on_actionSave_Per_Vertex_Labels_triggered();
    void on_actionSave_Label_Images_2_triggered();
    void on_actionSave_Edge_Images_triggered();
    void saveAllImages(const char* type);
    void on_actionLoad_Reprojection_Results_triggered();
    void on_actionLoad_Wallfinding_Floor_Plan_triggered();
    void on_actionLoad_Per_Vertex_Labels_triggered();
    void on_actionExport_Room_Model_triggered();
    void on_actionExport_Mesh_with_Colors_triggered();

    void on_actionReload_Per_Vertex_Samples_triggered();

    // Data Completions
    void meshLoaded();
    void imagesLoaded();
    void meshAndImagesLoaded();
    void labelDataLoaded();
    void samplesLoaded();
    void floorPlanLoaded();
    void edgeImagesLoaded();
    void edgesAndFloorPlanLoaded();

    void checkEnableHemicubes();

    // Task Completions
    void wallfindingDone();
    void orientationfindingDone();
    void partialVertexDataLoaded(int percent);



private:
    Ui::MainWindow *ui;
    ImageManager* imgr;
    MeshManager* mmgr;
    roommodel::RoomModel* room;
    roommodel::RoomModel* orientationtransform;

    std::vector<SubprocessWorker*> workers;
    QProgressBar* progressbar;

    QString meshfilename;
    QString camfilename;

    QSettings* settings;
    QString settingsfilename;

    QTemporaryFile* temproommodel;
    QString roommodelfile;

    std::vector<int> lines;
    int imageindex;
    int typeindex;

    // Load state
    bool hasFloorPlan;
    bool hasEdgeImages;

    int imagedisplaymode;
    std::vector<CameraParams> hemicubecams;
    std::vector<int> wallindices;
    std::vector<int> floorindices;
};

#endif // MAINWINDOW_H
