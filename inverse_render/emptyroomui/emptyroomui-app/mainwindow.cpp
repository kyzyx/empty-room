#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "datamanager/fileimageserver.h"
#include <QKeyEvent>
#include <QFileDialog>
#include <QIntValidator>
#include <QCheckBox>
#include <QThread>
#include <QToolTip>


/*
 * Action Flowchart
 * ----------------
 * LoadMesh -> Mesh
 * LoadImages -> Images [LabelImages]
 * LoadReproject -> Reproject
 * LoadFloorPlan -> FloorPlan
 * LoadLabels -> Labels
 * DoReprojection -> Reproject
 * DoWallfinding -> Floorplan Labels
 * LoadEquations -> Equations
 * RenderHemicubes -> Hemicubes Equations
 * DoSolving -> Lighting
 * ComputeLabelImages -> LabelImages
 * DoEdgefinding -> EdgeImages
 * DoVoting -> ???
 *
 * Mesh: (Do/Load)Wallfinding+Options LoadLabels LoadReproject -- DisplayMesh RenderOptions
 * Images: -- DisplayImageOptions
 * Mesh*Images: DoReprojection DoEdgeFinding -- DisplayCameras CameraViewOptions
 * Reproject: SaveReprojection
 * FloorPlan: SaveFloorPlan -- DisplayFloorPlan
 * Labels: SaveLabels ComputeLabelImages
 * Reproject*Labels: RenderHemicubes
 * Hemicubes: -- DisplayHemicubes
 * Equations: DoSolving
 *
 * Reproject*Lighting*LabelImages: DoFloorTex
 *
 * LabelImages: SaveLabelImages -- ViewLabelImages
 * EdgeImages: DoVoting SaveEdgeImages - ViewEdgeImages
 */
class MeshDialog : public QFileDialog
{
    public:
        explicit MeshDialog(QWidget *parent=0) : QFileDialog(parent), ccw(0) {;}

        void addCheckboxes() {
            QHBoxLayout *hbl = new QHBoxLayout();
            ccw = new QCheckBox(QString("Flip Normals"), this);
            hbl->addWidget(ccw);
            ccw->setChecked(true);
            QGridLayout* mainLayout = dynamic_cast<QGridLayout*>(layout());
            if (mainLayout) {
                int numRows = mainLayout->rowCount();
                mainLayout->addLayout( hbl, numRows,0,1,-1);
            } else {
                QVBoxLayout* l = dynamic_cast<QVBoxLayout*>(layout());
                if (l) {
                    l->addLayout(hbl);
                }
            }
        }
        bool isCcw() const {
            if (ccw) return ccw->isChecked();
            return false;
        }
    private:
        QCheckBox *ccw;
};
class ImageDialog : public QFileDialog
{
    public:
        explicit ImageDialog(QWidget *parent=0) : QFileDialog(parent), flipx(0), flipy(0) { ; }
        void addCheckBoxes() {
            QHBoxLayout *hbl = new QHBoxLayout();
            flipx = new QCheckBox(QString("Flip horizontally"), this);
            flipy = new QCheckBox(QString("Flip vertically"), this);
            lazyload = new QCheckBox(QString("Lazily Load Images"), this);
            hbl->addWidget(flipx);
            hbl->addWidget(flipy);
            hbl->addWidget(lazyload);
            QGridLayout* mainLayout = dynamic_cast<QGridLayout*>(layout());
            if (mainLayout) {
                int numRows = mainLayout->rowCount();
                mainLayout->addLayout( hbl, numRows,0,1,-1);
            } else {

                QVBoxLayout* l = dynamic_cast<QVBoxLayout*>(layout());
                if (l) {
                    l->addLayout(hbl);
                }
            }
        }

        bool isFlipX() const {
            if (flipx) return flipx->isChecked();
            return false;
        }
        bool isFlipY() const {
            if (flipy) return flipy->isChecked();
            return false;
        }
        bool isLazyLoad() const {
            if (lazyload) return lazyload->isChecked();
            return false;
        }
    private:
        QCheckBox *flipx;
        QCheckBox *flipy;
        QCheckBox *lazyload;
};

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    imgr(NULL), mmgr(NULL),
    typeindex(0), imageindex(0),
    room(NULL), orientationtransform(NULL),
    lazyloadimages(false),
    imagedisplaymode(1)
{
    ui->setupUi(this);

    progressbar = new QProgressBar(ui->statusBar);
    progressbar->setMaximumSize(200,ui->statusBar->height());
    ui->statusBar->addPermanentWidget(progressbar);
    progressbar->setValue(100);
    settingsfilename = QApplication::applicationDirPath() + "/settings.ini";
    settings = new QSettings(settingsfilename, QSettings::NativeFormat);
    connect(ui->wf_resolutionSlider, SIGNAL(sliderMoved(int)), this, SLOT(showwfrestooltip(int)));
    connect(ui->wf_wallthresholdSlider, SIGNAL(sliderMoved(int)), this, SLOT(showwfthresholdtooltip(int)));
}

MainWindow::~MainWindow()
{
    delete ui;
    for (int i = 0; i < workers.size(); ++i) {
        workers[i]->terminate();
        delete workers[i];
    }
    if (mmgr) delete mmgr;
    if (imgr) delete imgr;
    if (room) delete room;
}

void MainWindow::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape) {
        close();
    } else if (e->key() == Qt::Key_Right) {
        if (ui->nextImageButton->isEnabled())
            updateImage(++imageindex);
    } else if (e->key() == Qt::Key_Left) {
        if (ui->prevImageButton->isEnabled())
            updateImage(--imageindex);
    } else {
        QWidget::keyPressEvent(e);
    }
}
// -----------------
// Interface Actions
// -----------------
void MainWindow::updateImage(int idx, int type)
{
    if (imagedisplaymode > 0) {
        imageindex = (idx+imgr->size())%imgr->size();
        typeindex = type%imgr->getNumImageTypes();
        int w = imgr->getCamera(0)->width;
        int h = imgr->getCamera(0)->height;
        switch (imgr->getImageType(typeindex).getType()) {
            case CV_32FC3:
                ui->imageWidget->setFloatImage((const float*) imgr->getImage(typeindex, imageindex), w, h, 3);
                break;
            case CV_32FC1:
                ui->imageWidget->setFloatImage((const float*) imgr->getImage(typeindex, imageindex), w, h, 1);
                break;
            case CV_8UC3:
                ui->imageWidget->setRGBImage((const unsigned char*) imgr->getImage(typeindex, imageindex), w, h, 3);
                break;
            case CV_8UC1:
                ui->imageWidget->setRGBImage((const unsigned char*) imgr->getImage(typeindex, imageindex), w, h, 1);
                break;
            default:
                return;
        }
        ui->imageWidget->clearLines();
        for (int i = 0; i < lines.size(); i+=5) {
            if (lines[i] == imageindex) {
                ui->imageWidget->addLine(lines[i+1], lines[i+2], lines[i+3], lines[i+4]);
            }
        }
        ui->meshWidget->highlightCamera(idx);
        if (ui->autoLookCheckbox->isChecked()) ui->meshWidget->lookThroughCamera(imgr->getCamera(idx));

        if (ui->meshWidget->renderOptions()->getMeshRenderFormat() == VIEW_SINGLEIMAGE) {
            ui->meshWidget->renderManager()->setShaderAuxInt(imageindex);
        }
        if (imageindex == 0) ui->prevImageButton->setEnabled(false);
        else ui->prevImageButton->setEnabled(true);
        if (imageindex == imgr->size() - 1) ui->nextImageButton->setEnabled(false);
        else ui->nextImageButton->setEnabled(true);
        ui->imageNumBox->setText(QString::number(imageindex));
    } else {
        imageindex = idx%hemicubecams.size();
        int w = hemicubecams[imageindex].width;
        float* face = new float[w*w*3];
        float* img = new float[4*w*w*3];
        bzero(img, 4*w*w*3*sizeof(float));
        int type = VIEW_AVERAGE;
        // center
        ui->meshWidget->renderManager()->readFromRender(&hemicubecams[imageindex], face, type, true);
        for (int i = 0; i < w; ++i) {
            float* row = img + 2*w*3 * (w/2+i);
            memcpy(row + w/2*3, face + i*w*3, w*3*sizeof(float));
        }
        // top
        CameraParams cam = hemicubecams[imageindex];
        std::swap(cam.towards, cam.up);
        cam.up = -cam.up;
        cam.right = -cam.right;
        ui->meshWidget->renderManager()->readFromRender(&cam, face, type, true);
        for (int i = 0; i < w/2; ++i) {
            float* row = img + 2*w*3 * (3*w/2 + i);
            memcpy(row + w/2*3, face + i*w*3, w*3*sizeof(float));
        }
        // bottom
        cam.towards = -cam.towards;
        cam.up = -cam.up;
        ui->meshWidget->renderManager()->readFromRender(&cam, face, type, true);
        for (int i = 0; i < w/2; ++i) {
            float* row = img + 2*w*3 * (w/2-i-1);
            memcpy(row + w/2*3, face + (w-i)*w*3, w*3*sizeof(float));
        }
        // left
        cam = hemicubecams[imageindex];
        std::swap(cam.towards, cam.right);
        cam.towards = -cam.towards;
        ui->meshWidget->renderManager()->readFromRender(&cam, face, type, true);
        for (int i = 0; i < w; ++i) {
            memcpy(img + 2*w*3*(w/2+i), face + i*w*3 + 3*w/2, w/2*3*sizeof(float));
        }
        // right
        cam = hemicubecams[imageindex];
        std::swap(cam.towards, cam.right);
        cam.right = -cam.right;
        ui->meshWidget->renderManager()->readFromRender(&cam, face, type, true);
        for (int i = 0; i < w; ++i) {
            memcpy(img + 2*w*3*(w/2+i)+3*3*w/2, face + i*w*3, w/2*3*sizeof(float));
        }
        // TODO: !!! Display type
        // TODO: !!! Normalization transform on sample cameras
        // TODO: !!! Display stats (Total incoming light, etc.)
        ui->imageWidget->setFloatImage(img, 2*w, 2*w, 3);
        ui->meshWidget->highlightCamera(idx);
        if (ui->autoLookCheckbox->isChecked()) ui->meshWidget->lookThroughCamera(&hemicubecams[imageindex]);
        if (imageindex == 0) ui->prevImageButton->setEnabled(false);
        else ui->prevImageButton->setEnabled(true);
        if (imageindex == hemicubecams.size() - 1) ui->nextImageButton->setEnabled(false);
        else ui->nextImageButton->setEnabled(true);
        ui->imageNumBox->setText(QString::number(imageindex));
    }
}

void MainWindow::on_loadImageButton_clicked()
{
    int n = ui->imageNumBox->text().toInt();
    updateImage(n, typeindex);
}

void MainWindow::onCameraSelection(int selected) {
    if (imageindex == selected) {
        if (imagedisplaymode > 0) ui->meshWidget->lookThroughCamera(imgr->getCamera(selected));
        else ui->meshWidget->lookThroughCamera(&hemicubecams[selected]);
    }
    updateImage(selected);
}

void MainWindow::on_autoLookCheckbox_toggled(bool checked)
{
    if (checked) {
        if (imagedisplaymode > 0) ui->meshWidget->lookThroughCamera(imgr->getCamera(imageindex));
        else ui->meshWidget->lookThroughCamera(&hemicubecams[imageindex]);
    }
}

void MainWindow::on_showCameraCheckbox_toggled(bool checked)
{
    ui->meshWidget->renderOptions()->setRenderCameras(checked);
    if (checked) {
        ui->showCurrentCameraCheckbox->setChecked(true);
        ui->meshWidget->renderOptions()->setRenderCurrentCamera(true);
    }
}

void MainWindow::on_showCurrentCameraCheckbox_toggled(bool checked)
{
    ui->meshWidget->renderOptions()->setRenderCurrentCamera(checked);
}

void MainWindow::on_showMeshCheckbox_toggled(bool checked)
{
    ui->meshWidget->renderOptions()->setRenderMesh(checked);
}

void MainWindow::on_showRoomCheckbox_toggled(bool checked)
{
    ui->meshWidget->renderOptions()->setRenderRoom(checked);
}

void MainWindow::on_showTrajectoryCheckbox_toggled(bool checked)
{
    ui->meshWidget->renderOptions()->setRenderTrajectory(checked);
}

void MainWindow::showwfrestooltip(int v) {
    QToolTip::showText(QCursor::pos(), QString::number(v*0.001), ui->wf_resolutionSlider);
}

void MainWindow::showwfthresholdtooltip(int v) {
    QToolTip::showText(QCursor::pos(), QString::number(v), ui->wf_wallthresholdSlider);
}

void MainWindow::on_actionQuit_triggered()
{
    close();
}

// --------------------------
// Basic Data Loading Actions
// --------------------------
void MainWindow::on_actionOpen_Mesh_triggered()
{
    MeshDialog mdialog(this);
    mdialog.setNameFilter("PLY Meshes (*.ply)");
    mdialog.setFileMode(QFileDialog::ExistingFile);
    mdialog.setOption(QFileDialog::DontUseNativeDialog);
    mdialog.setDirectory(settings->value("lastworkingdirectory", "").toString());
    mdialog.selectFile(QFileInfo(settings->value("lastmeshfile", "").toString()).fileName());
    mdialog.addCheckboxes();
    if (mdialog.exec()) {
        meshfilename = mdialog.selectedFiles().first();
        settings->setValue("lastmeshfile", meshfilename);
        QDir cwd = QDir(meshfilename);
        cwd.cdUp();
        settings->setValue("lastworkingdirectory", cwd.canonicalPath());
        QString cmd = settings->value("loadmesh_binary", "dataserver -meshfile %1 -p").toString();
        cmd = cmd.arg(meshfilename);
        if (mdialog.isCcw()) {
            cmd = cmd + " -ccw";
            settings->setValue("lastmeshflags", " -ccw");
        }
        progressbar->setValue(0);
        SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
        workers.push_back(w);
        QThread* thread = new QThread;
        connect(thread, SIGNAL(started()), w, SLOT(run()));
        connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
        connect(w, SIGNAL(done()), this, SLOT(meshLoaded()));
        w->moveToThread(thread);
        thread->start();
    }
}
void MainWindow::meshLoaded() {
    ui->wallfindButton->setEnabled(true);
    ui->loadWallsButton->setEnabled(true);
    ui->loadReprojectButton->setEnabled(true);
    ui->actionLoad_Last_Intermediates->setEnabled(true);
    ui->actionLoad_Reprojection_Results->setEnabled(true);
    ui->actionLoad_Per_Vertex_Labels->setEnabled(true);
    ui->wf_resolutionSlider->setEnabled(true);
    ui->wf_wallthresholdSlider->setEnabled(true);
    ui->showMeshCheckbox->setEnabled(true);
    ui->DebugWallfindingButton->setEnabled(true);
    ui->actionReload_Per_Vertex_Samples->setEnabled(true);
    mmgr = new MeshManager(meshfilename.toStdString());
    ui->meshWidget->setMeshManager(mmgr);

    ui->viewTypeComboBox->setEnabled(true);
    ui->viewTypeComboBox->clear();
    for (int i = 0; i < ui->meshWidget->renderManager()->getNumShaderTypes(); ++i) {
        if (ui->meshWidget->renderManager()->getShader(i).getFlags() & SHADERFLAGS_PASS) {
            continue;
        } else {
            ui->viewTypeComboBox->insertItem(i, QString::fromStdString(ui->meshWidget->renderManager()->getShader(i).getDescription()));
        }
    }
    connect(ui->viewTypeComboBox, SIGNAL(currentIndexChanged(int)), ui->meshWidget->renderOptions(), SLOT(setMeshRenderFormat(int)));
    connect(ui->viewTypeComboBox, SIGNAL(currentIndexChanged(int)), ui->meshWidget, SLOT(loadSettings(int)));
    meshAndImagesLoaded();
}

void MainWindow::on_actionOpen_Images_triggered()
{
    ImageDialog idialog(this);
    idialog.setNameFilter("Camera Files (*.cam)");
    idialog.setFileMode(QFileDialog::ExistingFile);
    idialog.setOption(QFileDialog::DontUseNativeDialog);
    idialog.setDirectory(settings->value("lastworkingdirectory", "").toString());
    idialog.selectFile(QFileInfo(settings->value("lastcamerafile", "").toString()).fileName());
    idialog.addCheckBoxes();
    if (idialog.exec()) {
        camfilename = idialog.selectedFiles().first();
        settings->setValue("lastcamerafile", camfilename);
        QDir cwd = QDir(camfilename);
        cwd.cdUp();
        settings->setValue("lastworkingdirectory", cwd.canonicalPath());
        QString cmd = settings->value("loadimage_binary", "dataserver -camfile %1 -p").toString();
        cmd = cmd.arg(camfilename);
        bool flipx = false;
        bool flipy = false;
        if (idialog.isFlipX()) flipx = true;
        if (idialog.isFlipY()) flipy = true;
        if (idialog.isLazyLoad()) lazyloadimages = true;
        QString extraflags = "";
        if (idialog.isFlipX()) extraflags = extraflags + " -flip_x";
        if (idialog.isFlipY()) extraflags = extraflags + " -flip_y";
        cmd += extraflags;
        settings->setValue("lastimageflags", extraflags);

        if (lazyloadimages) {
            imgr = new FileImageServer(camfilename.toStdString(), flipx, flipy);
            imagesLoaded();
        } else {
            progressbar->setValue(0);
            SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
            workers.push_back(w);
            QThread* thread = new QThread;
            connect(thread, SIGNAL(started()), w, SLOT(run()));
            connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
            connect(w, SIGNAL(done()), this, SLOT(imagesLoaded()));
            w->moveToThread(thread);
            thread->start();
        }
    }
}
void MainWindow::imagesLoaded() {
    if (!lazyloadimages) imgr = new ImageManager(camfilename.toStdString());
    ui->imageTypeComboBox->setEnabled(true);
    ui->nextImageButton->setEnabled(true);
    ui->imageNumBox->setEnabled(true);
    ui->loadImageButton->setEnabled(true);
    ui->edgeFilterButton->setEnabled(true);

    ui->imageNumBox->setValidator(new QIntValidator(0, imgr->size()-1, this));

    ui->imageTypeComboBox->clear();
    for (int i = 0; i < imgr->getNumImageTypes(); ++i) {
        ui->imageTypeComboBox->insertItem(i, QString::fromStdString(imgr->getImageType(i).getName()));
    }
    updateImage(0, typeindex);
    connect(ui->imageTypeComboBox, SIGNAL(currentIndexChanged(int)), ui->imageWidget, SLOT(loadSettings(int)));

    bool edgeImagesInitialized = true;
    for (int i = 0; i < imgr->size(); ++i) {
        if (!(imgr->getFlags("edges", i) & ImageManager::DF_INITIALIZED)) {
            edgeImagesInitialized = false;
            break;
        }
    }
    if (edgeImagesInitialized) edgeImagesLoaded();
    meshAndImagesLoaded();
}

void MainWindow::on_actionLoad_Last_Mesh_Camera_File_triggered()
{
    camfilename = settings->value("lastcamerafile", "").toString();
    meshfilename = settings->value("lastmeshfile", "").toString();
    QString cmd = settings->value("loadall_binary", "dataserver -camfile %1 -meshfile %2 -p").toString();
    cmd = cmd.arg(camfilename, meshfilename);
    QString extraflags = settings->value("lastimageflags", "").toString();
    cmd += extraflags;
    extraflags = settings->value("lastmeshflags", "").toString();
    cmd += extraflags;
    progressbar->setValue(0);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(done()), this, SLOT(imagesLoaded()));
    connect(w, SIGNAL(done()), this, SLOT(meshLoaded()));
    w->moveToThread(thread);
    thread->start();
}

void MainWindow::meshAndImagesLoaded() {
    if (imgr && mmgr) {
        ui->reprojectButton->setEnabled(true);
        ui->showCameraCheckbox->setEnabled(true);
        ui->showCurrentCameraCheckbox->setEnabled(true);
        ui->autoLookCheckbox->setEnabled(true);
        ui->showTrajectoryCheckbox->setEnabled(true);
        // Render cameras
        ui->meshWidget->setupCameras(imgr);
        connect(ui->meshWidget, SIGNAL(cameraSelected(int)), this, SLOT(onCameraSelection(int)));
    }
}

// --------------------
// Reprojection Actions
// --------------------
#include <boost/interprocess/sync/sharable_lock.hpp>
void MainWindow::on_actionLoad_Reprojection_Results_triggered()
{
    if (mmgr) {
        QString lwd = settings->value("lastworkingdirectory", "").toString();
        //QString cfile = settings->value("lastreprojectfile", lwd).toString();
        QString datafilename = QFileDialog::getOpenFileName(this, "Open Reprojection Samples", lwd);
        if (!datafilename.isEmpty()) {
            QDir cwd = QDir(datafilename);
            cwd.cdUp();
            settings->setValue("lastworkingdirectory", cwd.canonicalPath());
            settings->setValue("lastreprojectfile", datafilename);
            loadVertexSampleData(meshfilename, datafilename);
        }
    }
}
void MainWindow::on_actionLoad_Last_Intermediates_triggered()
{
    loadVertexSampleData(meshfilename, settings->value("lastreprojectfile", "").toString());
    loadVertexLabelData(meshfilename, settings->value("lastlabelsfile", "").toString());
}
void MainWindow::samplesLoaded() {
    if (mmgr->loadSamples()) {
        boost::interprocess::sharable_lock<MeshManager::shmutex> lock(*(mmgr->getMutex(MeshManager::NUM_CHANNELS,0)));
        ui->meshWidget->setupMeshColors();
    }
    ui->actionSave_Reprojection_Results->setEnabled(true);
    checkEnableHemicubes();

    ui->actionExport_Mesh_with_Colors->setEnabled(true);
    ui->lightsliderlabel->setEnabled(true);
    ui->overlayThresholdCheckbox->setEnabled(true);
    connect(ui->overlayThresholdCheckbox, SIGNAL(toggled(bool)), ui->meshWidget->renderOptions(), SLOT(setOverlayThresholded(bool)));
    ui->commitLightsButton->setEnabled(true);
    ui->lightThresholdSlider->setEnabled(true);
    connect(ui->lightThresholdSlider, SIGNAL(valueChanged(int)), ui->meshWidget->renderOptions(), SLOT(setLowerThreshold(int)));
}
void MainWindow::partialVertexDataLoaded(int percent) {
    static int lastpercent = 0;
    if (percent == 100) {
        samplesLoaded();
        return;
    }
    if (lastpercent/5 != percent/5) {
        if (mmgr->loadSamples()) {
            boost::interprocess::sharable_lock<MeshManager::shmutex> lock(*(mmgr->getMutex(MeshManager::NUM_CHANNELS,0)));
            ui->meshWidget->setupMeshColors();
        }
    }
    lastpercent = percent;
}

void MainWindow::on_actionReload_Per_Vertex_Samples_triggered()
{
    samplesLoaded();
}

enum Label {
    LABEL_NONE=0,
    LABEL_WALL,
    LABEL_CEILING,
    LABEL_FLOOR,
    LABEL_CORNER
};

void MainWindow::labelDataLoaded() {
    ui->computeLabelImagesButton->setEnabled(true);
    ui->meshWidget->updateMeshAuxiliaryData();
    ui->actionSave_Per_Vertex_Labels->setEnabled(true);
    ui->overlayTypeComboBox->setEnabled(true);
    ui->overlayTypeComboBox->insertItem(0,"No Overlay");
    ui->overlayTypeComboBox->insertItem(1,"Overlay identified lights");
    ui->overlayTypeComboBox->insertItem(2,"Overlay semantic data");
    connect(ui->overlayTypeComboBox, SIGNAL(currentIndexChanged(int)), ui->meshWidget->renderOptions(), SLOT(setOverlay(int)));

    // Initialize index arrays
    {
        //boost::interprocess::shared_lock<MeshManager::shmutex> lock(*(mmgr->getMutex(MeshManager::TYPE_CHANNEL,0)));
        for (int i = 0; i < mmgr->NVertices(); ++i) {
            char label =  mmgr->getLabel(i, MeshManager::TYPE_CHANNEL);
            if (label == LABEL_WALL) wallindices.push_back(i);
            else if (label == LABEL_FLOOR) floorindices.push_back(i);
        }
    }
    checkEnableHemicubes();
}

void MainWindow::checkEnableHemicubes() {
    if (mmgr->hasSamples() && ui->computeLabelImagesButton->isEnabled()) {
        ui->hemicubeButton->setEnabled(true);
        ui->hemicubeResLineEdit->setEnabled(true);
        ui->numSamplesLineEdit->setEnabled(true);

        ui->numSamplesLineEdit->setValidator(new QIntValidator(1, std::min(wallindices.size(), floorindices.size()), this));
        ui->hemicubeResLineEdit->setValidator(new QIntValidator(20, 800, this));
    }
}

void MainWindow::loadVertexSampleData(QString meshfile, QString datafile) {
    if (datafile.isEmpty()) return;
    QString cmd = settings->value("loadsamples_binary", "dataserver -meshfile %1 -samplesfile %2 -p").toString();
    cmd = cmd.arg(meshfile, datafile);
    progressbar->setValue(0);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(done()), this, SLOT(samplesLoaded()));
    w->moveToThread(thread);
    thread->start();
}
void MainWindow::loadVertexLabelData(QString meshfile, QString datafile) {
    if (datafile.isEmpty()) return;
    QString cmd = settings->value("loadlabels_binary", "dataserver -meshfile %1 -labelsfile %2 -p").toString();
    cmd = cmd.arg(meshfile, datafile);
    progressbar->setValue(0);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(done()), this, SLOT(labelDataLoaded()));
    w->moveToThread(thread);
    thread->start();
}

void MainWindow::on_reprojectButton_clicked()
{
    QString cmd = settings->value("reproject_binary", "reprojectapp -camfile %1 -meshfile %2 -p").toString();
    cmd = cmd.arg(camfilename, meshfilename);
    QString extraflags;
    if (lazyloadimages) extraflags += " -noshm";
    cmd += extraflags;
    progressbar->setValue(0);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(percentChanged(int)), this, SLOT(partialVertexDataLoaded(int)));
    w->moveToThread(thread);
    thread->start();
}

void MainWindow::on_commitLightsButton_clicked()
{
    QString cmd = settings->value("reproject_binary", "reprojectapp -camfile %1 -meshfile %2 -p").toString();
    cmd = cmd.arg(camfilename, meshfilename);
    QString extraflags = " -label_lights_only -hdr_threshold %1";
    if (lazyloadimages) extraflags += " -noshm";
    cmd += extraflags.arg(QString::number(ui->lightThresholdSlider->value()));
    progressbar->setValue(0);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(done()), this, SLOT(labelDataLoaded()));
    w->moveToThread(thread);
    thread->start();
}

void MainWindow::on_actionLoad_Per_Vertex_Labels_triggered()
{
    if (mmgr) {
        QString lwd = settings->value("lastworkingdirectory", "").toString();
        //QString cfile = settings->value("lastlabelsfile", lwd).toString();
        QString datafilename = QFileDialog::getOpenFileName(this, "Load Per-Vertex Labels", lwd);
        if (!datafilename.isEmpty()) {
            QDir cwd = QDir(datafilename);
            cwd.cdUp();
            settings->setValue("lastworkingdirectory", cwd.canonicalPath());
            settings->setValue("lastlabelsfile", datafilename);
            loadVertexLabelData(meshfilename, datafilename);
        }
    }
}

// -------------------
// Wallfinding Actions
// -------------------
void MainWindow::on_wallfindButton_clicked()
{
    temproommodel = new QTemporaryFile();
    temproommodel->open();
    temproommodel->close();
    QString cmd = settings->value("wallfind_binary", "wallfindapp -meshfile %1 -outputroommodel %2 -p").toString();
    cmd = cmd.arg(meshfilename, temproommodel->fileName());
    QString extraflags = "";
    extraflags += " -resolution " + QString::number(ui->wf_resolutionSlider->value()*0.001);
    extraflags += " -wallthreshold " + QString::number(ui->wf_wallthresholdSlider->value());
    cmd += extraflags;
    progressbar->setValue(0);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(done()), this, SLOT(wallfindingDone()));
    w->moveToThread(thread);
    thread->start();
}

void MainWindow::wallfindingDone() {
    loadFloorPlan(temproommodel->fileName());
    labelDataLoaded();
}

void MainWindow::floorPlanLoaded() {
    hasFloorPlan = true;
    ui->meshWidget->setRoomModel(room);
    ui->showRoomCheckbox->setEnabled(true);
    ui->actionSave_Wallfinding_Floor_Plan->setEnabled(true);
    ui->actionExport_Room_Model->setEnabled(true);
    edgesAndFloorPlanLoaded();
}

void MainWindow::loadFloorPlan(QString floorplanfile) {
    if (floorplanfile.isEmpty()) return;
    if (room) delete room;
    roommodelfile = floorplanfile;
    room = new roommodel::RoomModel;
    roommodel::load(*room, floorplanfile.toStdString());
    floorPlanLoaded();
}

void MainWindow::on_actionLoad_Wallfinding_Floor_Plan_triggered()
{
    QString lwd = settings->value("lastworkingdirectory", "").toString();
    //QString cfile = settings->value("lastlabelsfile", lwd).toString();
    QString datafilename = QFileDialog::getOpenFileName(this, "Open Floor Plan", lwd, "JSON Files (*.json)");
    if (!datafilename.isEmpty()) {
        QDir cwd = QDir(datafilename);
        cwd.cdUp();
        settings->setValue("lastworkingdirectory", cwd.canonicalPath());
        settings->setValue("lastfloorplanfile", datafilename);
        loadFloorPlan(datafilename);
    }
}


void MainWindow::on_wf_resolutionSlider_valueChanged(int value)
{
    if (ui->meshWidget->renderOptions()->shouldRenderWfHistogram()) {
        if (room || orientationtransform)
            ui->meshWidget->computeWallFindingHistogram(ui->wf_resolutionSlider->value()*0.001);
      }
}

void MainWindow::on_DebugWallfindingButton_clicked()
{
    if (ui->meshWidget->renderOptions()->shouldRenderWfHistogram()) {
        ui->DebugWallfindingButton->setText("Debug Wallfinding");
        ui->meshWidget->renderOptions()->setRenderWfHistogram(false);
    } else {
        if (room || orientationtransform) {
            ui->DebugWallfindingButton->setText("Hide Wallfinding Debug");
            ui->meshWidget->computeWallFindingHistogram(ui->wf_resolutionSlider->value()*0.001);
            ui->meshWidget->renderOptions()->setRenderWfHistogram(true);
            ui->meshWidget->renderOptions()->setWfThreshold(ui->wf_wallthresholdSlider->value());
            connect(ui->wf_wallthresholdSlider, SIGNAL(valueChanged(int)), ui->meshWidget->renderOptions(), SLOT(setWfThreshold(int)));
        } else {
            ui->DebugWallfindingButton->setEnabled(false);
            temproommodel = new QTemporaryFile();
            temproommodel->open();
            temproommodel->close();
            QString cmd = settings->value("wallfind_binary", "wallfindapp -meshfile %1 -outputroommodel %2 -p").toString();
            cmd = cmd.arg(meshfilename, temproommodel->fileName());
            QString extraflags = " -orientation ";
            cmd += extraflags;
            progressbar->setValue(0);
            SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
            workers.push_back(w);
            QThread* thread = new QThread;
            connect(thread, SIGNAL(started()), w, SLOT(run()));
            connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
            connect(w, SIGNAL(done()), this, SLOT(orientationfindingDone()));
            w->moveToThread(thread);
            thread->start();
        }
    }
}

void MainWindow::orientationfindingDone() {
    orientationtransform = new roommodel::RoomModel;
    roommodel::load(*orientationtransform, temproommodel->fileName().toStdString());
    ui->meshWidget->setOrientation(orientationtransform);

    ui->DebugWallfindingButton->setEnabled(true);
    ui->DebugWallfindingButton->setText("Hide Wallfinding Debug");
    ui->meshWidget->computeWallFindingHistogram(ui->wf_resolutionSlider->value()*0.001);
    ui->meshWidget->renderOptions()->setRenderWfHistogram(true);
    ui->meshWidget->renderOptions()->setWfThreshold(ui->wf_wallthresholdSlider->value());
    connect(ui->wf_wallthresholdSlider, SIGNAL(valueChanged(int)), ui->meshWidget->renderOptions(), SLOT(setWfThreshold(int)));
}


// ------------------------
// Inverse Lighting Actions
// ------------------------
void MainWindow::on_computeLabelImagesButton_clicked()
{
    progressbar->setValue(0);
    for (int i = 0; i < imgr->size(); ++i) {
        progressbar->setValue(100*i/imgr->size());
        ui->meshWidget->renderManager()->createLabelImage(imgr->getCamera(i), imgr->getImageWriteable("labels",i));
        int f = imgr->getFlags("labels", i);
        imgr->setFlags("labels", i, f|ImageManager::DF_INITIALIZED);
        if (lazyloadimages) {
            imgr->saveImage("labels", i);
        }
    }
    progressbar->setValue(100);
}

#include <random>

void MainWindow::on_hemicubeButton_clicked()
{
    if (imagedisplaymode > 0) {
        hemicubecams.clear();
        int numsamples = ui->numSamplesLineEdit->text().toInt();
        std::default_random_engine generator;
        std::uniform_int_distribution<int> dist(0, wallindices.size());
        for (int i = 0; i < numsamples; ++i) {
            int n;
            do {
                n = dist(generator);
            } while (mmgr->getLabel(wallindices[n]) > 0 || mmgr->getVertexSampleCount(wallindices[n]) == 0);

            CameraParams cam;
            cam.fov = 90;
            cam.height = ui->hemicubeResLineEdit->text().toInt();
            cam.width = ui->hemicubeResLineEdit->text().toInt();
            cam.pos = mmgr->VertexPosition(wallindices[n]);
            cam.towards = mmgr->VertexNormal(wallindices[n]);
            cam.up = R3yaxis_vector;
            cam.right = cam.towards;
            cam.right.Cross(cam.up);
            cam.focal_length = cam.width/2;
            hemicubecams.push_back(cam);
        }
        ui->meshWidget->setupCameras(hemicubecams);
        ui->showTrajectoryCheckbox->setEnabled(false);
        ui->showTrajectoryCheckbox->setChecked(false);
        ui->hemicubeButton->setText("Revert display to cameras");
    } else {
        ui->meshWidget->setupCameras(imgr);
        ui->showTrajectoryCheckbox->setEnabled(true);
        ui->hemicubeButton->setText("Render hemicubes");
    }
    imagedisplaymode = -imagedisplaymode;
    updateImage(0);

    /*
    progressbar->setValue(0);
    // FIXME - where to put images
    // FIXME - progress bar in computeSamples
    HemicubeRenderer hr(ui->meshWidget->renderManager(), ui->hemicubeResLineEdit->text().toInt());
    std::vector<SampleData> samples;
    std::vector<float*> images;
    hr.computeSamples(samples, wallindices, ui->numSamplesLineEdit->text().toInt(), 1., &images);
    progressbar->setValue(100);*/
}

// ---------------------------
// Door/Window Finding Actions
// ---------------------------
void MainWindow::on_edgeFilterButton_clicked()
{
    QString cmd = settings->value("compute_edges_binary", "edgeimageapp -camfile %1 -p").toString();
    cmd = cmd.arg(camfilename);
    QString extraflags = "";
    if (lazyloadimages) extraflags += " -noshm";
    cmd += extraflags;
    progressbar->setValue(0);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(done()), progressbar, SLOT(edgeImagesLoaded()));
    w->moveToThread(thread);
    thread->start();
}

void MainWindow::edgeImagesLoaded() {
    hasEdgeImages = true;
    edgesAndFloorPlanLoaded();
}

void MainWindow::edgesAndFloorPlanLoaded() {
    if (hasEdgeImages && hasFloorPlan) {
        ui->computeVerticalLinesButton->setEnabled(true);
    }
}

void MainWindow::addLine(QString l) {
    QStringList ls = l.split(" ");
    for (int i = 0; i < 5; ++i) lines.push_back(ls[i].toInt());
    updateImage(imageindex, typeindex);
}

void MainWindow::on_computeVerticalLinesButton_clicked()
{
    QString cmd = settings->value("compute_rwo_binary", "linefindapp -camfile %1 -roommodel %2 -p").toString();
    cmd = cmd.arg(camfilename, roommodelfile);
    QString extraflags = "";
    if (lazyloadimages) extraflags += " -noshm";
    cmd += extraflags;
    progressbar->setValue(0);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(data(QString)), this, SLOT(addLine(QString)));
    w->moveToThread(thread);
    thread->start();

}

// ----------------------------
// Saving Intermediates Actions
// ----------------------------
void MainWindow::on_actionSave_Reprojection_Results_triggered()
{
    if (mmgr) {
        QString lwd = settings->value("lastworkingdirectory", "").toString();
        QString datafilename = QFileDialog::getSaveFileName(this, "Save Reprojection Samples", lwd);
        if (!datafilename.isEmpty()) {
            QDir cwd = QDir(datafilename);
            cwd.cdUp();
            settings->setValue("lastworkingdirectory", cwd.canonicalPath());
            // settings->setValue("lastreprojectfile", datafilename);

            mmgr->writeSamplesToFile(datafilename.toStdString(), boost::bind(&QProgressBar::setValue, progressbar, _1));
        }
    }
}
void MainWindow::on_actionSave_Wallfinding_Floor_Plan_triggered()
{
    if (mmgr) {
        QString lwd = settings->value("lastworkingdirectory", "").toString();
        QString datafilename = QFileDialog::getSaveFileName(this, "Save Floor Plan", lwd, "JSON files (*.json)");
        if (!datafilename.isEmpty()) {
            QDir cwd = QDir(datafilename);
            cwd.cdUp();
            settings->setValue("lastworkingdirectory", cwd.canonicalPath());
            settings->setValue("lastfloorplanfile", datafilename);
            roommodel::save(*room, datafilename.toStdString());
        }
    }
}
void MainWindow::on_actionSave_Per_Vertex_Labels_triggered()
{
    if (mmgr) {
        QString lwd = settings->value("lastworkingdirectory", "").toString();
        QString datafilename = QFileDialog::getSaveFileName(this, "Save Per-Vertex Labels", lwd);
        if (!datafilename.isEmpty()) {
            QDir cwd = QDir(datafilename);
            cwd.cdUp();
            settings->setValue("lastworkingdirectory", cwd.canonicalPath());
            settings->setValue("lastlabelsfile", datafilename);
            mmgr->writeLabelsToFile(datafilename.toStdString());
        }
    }
}

void MainWindow::on_actionSave_Label_Images_2_triggered()
{
    saveAllImages("labels");
}
void MainWindow::on_actionSave_Edge_Images_triggered()
{
    saveAllImages("edges");
}

void MainWindow::saveAllImages(const char *type) {
    if (imgr && !lazyloadimages) {
        progressbar->setValue(0);
        for (int i = 0; i < imgr->size(); ++i) {
            progressbar->setValue(100*i/imgr->size());
            if (imgr->getFlags(type, i) & ImageManager::DF_INITIALIZED)
                imgr->saveImage(type, i);
        }
        progressbar->setValue(100);
    }
}

// Mesh exporting
void MainWindow::on_actionExport_Room_Model_triggered()
{
    if (!room) return;
    QString lwd = settings->value("lastworkingdirectory", "").toString();
    QString filename = QFileDialog::getSaveFileName(this, "Save Room Model as PLY", lwd, "PLY files (*.ply)");
    if (!filename.isEmpty()) room->saveToPly(filename.toStdString());
}

void MainWindow::on_actionExport_Mesh_with_Colors_triggered()
{
    if (mmgr) {
        QString lwd = settings->value("lastworkingdirectory", "").toString();
        QString filename = QFileDialog::getSaveFileName(this, "Save Mesh as PLY", lwd, "PLY files (*.ply)");
        if (!filename.isEmpty()) mmgr->writePlyMesh(filename.toStdString(), 1/ui->meshWidget->getUpperBound());
    }
}
