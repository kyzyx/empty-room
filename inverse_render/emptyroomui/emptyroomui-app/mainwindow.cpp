#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "datamanager/fileimageserver.h"
#include "datamanager/imageio.h"
#include <QKeyEvent>
#include <QFileDialog>
#include <QIntValidator>
#include <QInputDialog>
#include <QCheckBox>
#include <QThread>
#include <QToolTip>

#include <set>


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
            lazyload->setChecked(true);
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
    lazyloadimages(false), hr(NULL),
    imagedisplaymode(1), tempformfactors(NULL)
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
    if (tempformfactors) {
        tempformfactors->close();
        delete tempformfactors;
    }
    if (temproommodel) {
        temproommodel->close();
        delete temproommodel;
    }
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
void transpose(float* f, int w) {
    float tmp;
    for (int i = 0; i < w; i++) {
        for (int j = i+1; j < w; j++) {
            for (int k = 0; k < 3; k++) {
                tmp = f[3*(i*w+j)+k];
                f[3*(i*w+j)+k] = f[3*(j*w+i)+k];
                f[3*(j*w+i)+k] = tmp;
            }
        }
    }
}

void MainWindow::updateImage(int idx, int type)
{
    if (imagedisplaymode > 0) {
        imageindex = (idx+imgr->size())%imgr->size();
        typeindex = type%imgr->getNumImageTypes();
        int w = imgr->getCamera(0)->width;
        int h = imgr->getCamera(0)->height;
        const void* img = imgr->getImage(typeindex, imageindex);
        if (imgr->getFlags(typeindex, imageindex) & ImageManager::DF_INITIALIZED) {
            switch (imgr->getImageType(typeindex).getType()) {
                case CV_32FC3:
                    ui->imageWidget->setFloatImage((const float*) img, w, h, 3);
                    break;
                case CV_32FC1:
                    ui->imageWidget->setFloatImage((const float*) img, w, h, 1);
                    break;
                case CV_8UC3:
                    ui->imageWidget->setRGBImage((const unsigned char*) img, w, h, 3);
                    break;
                case CV_8UC1:
                    ui->imageWidget->setRGBImage((const unsigned char*) img, w, h, 1);
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
        }
        else {
            ui->imageWidget->setErrorImage(w, h);
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
        ui->meshWidget->renderManager(); // Get the correct OpenGL Context
        imageindex = idx%hemicubecams.size();
        typeindex = type%6;
        int w = hemicubecams[imageindex].width;
        float* face = new float[w*w*3];
        float* img = new float[4*w*w*3];
        bzero(img, 4*w*w*3*sizeof(float));
        int rtype = (typeindex&1)?VIEW_LABELS:VIEW_AVERAGE;
        if (hr) {
            int camidx = hemicubeindices[imageindex];
            bool envmap = !(typeindex&1);
            std::vector<Light*> lights;
            SampleData s = hr->computeSample(camidx, lights, envmap?face:NULL, envmap?NULL:face);
            s.netIncoming /= (1-s.fractionUnknown);
            RNRgb rgb;
            rgb[0] = s.radiosity.r/s.netIncoming.r;
            rgb[1] = s.radiosity.g/s.netIncoming.g;
            rgb[2] = s.radiosity.b/s.netIncoming.b;
            std::cout << "Sample " << camidx << ": ";
            std::cout << s.radiosity.r << "," << s.radiosity.g << "," << s.radiosity.b << " ";
            std::cout << s.netIncoming.r << "," << s.netIncoming.g << "," << s.netIncoming.b << " ";
            std::cout << rgb[0] << "," << rgb[1] << "," << rgb[2] << " ";
            std::cout << s.fractionUnknown << std::endl;
            ui->meshWidget->setCameraColor(imageindex, rgb);
        }
        if (typeindex < 2) {
            for (int i = 0; i < w; ++i) {
                float* row = img + 2*w*3 * (w/2+i);
                memcpy(row + w/2*3, face + i*w*3, w*3*sizeof(float));
            }
        } else {
            if (typeindex == 5) {
                for (int i = 0; i < w*w*3; i++) face[i] = 1;
            }
            // center
            if (typeindex != 5) ui->meshWidget->renderManager()->readFromRender(&hemicubecams[imageindex], face, rtype, true);
            if (typeindex&4) hr->weightTopHemicube(face, w*w*M_PI/4);
            for (int i = 0; i < w; ++i) {
                float* row = img + 2*w*3 * (w/2+i);
                memcpy(row + w/2*3, face + i*w*3, w*3*sizeof(float));
            }
            // top
            CameraParams cam = hemicubecams[imageindex];
            std::swap(cam.towards, cam.up);
            cam.up = -cam.up;
            cam.right = -cam.right;
            if (typeindex == 5) {
                for (int i = 0; i < w*w*3; i++) face[i] = 1;
            }
            if (typeindex != 5) ui->meshWidget->renderManager()->readFromRender(&cam, face, rtype, true);
            if (typeindex&4) hr->weightSideHemicube(face, w*w*M_PI/4);
            for (int i = 0; i < w/2; ++i) {
                float* row = img + 2*w*3 * (3*w/2 + i);
                memcpy(row + w/2*3, face + i*w*3, w*3*sizeof(float));
            }
            // bottom
            cam.towards = -cam.towards;
            cam.up = -cam.up;
            if (typeindex == 5) {
                for (int i = 0; i < w*w*3; i++) face[i] = 1;
            }
            if (typeindex != 5) ui->meshWidget->renderManager()->readFromRender(&cam, face, rtype, true);
            if (typeindex&4) hr->weightSideHemicube(face, w*w*M_PI/4);
            for (int i = 0; i < w/2; ++i) {
                float* row = img + 2*w*3 * (w/2-i-1);
                memcpy(row + w/2*3, face + (w-i-1)*w*3, w*3*sizeof(float));
            }
            // left
            cam = hemicubecams[imageindex];
            std::swap(cam.towards, cam.right);
            cam.towards = -cam.towards;
            if (typeindex == 5) {
                for (int i = 0; i < w*w*3; i++) face[i] = 1;
            }
            if (typeindex != 5) ui->meshWidget->renderManager()->readFromRender(&cam, face, rtype, true);
            if (typeindex&4) {
                transpose(face, w);
                hr->weightSideHemicube(face, w*w*M_PI/4);
                transpose(face, w);
            }
            for (int i = 0; i < w; ++i) {
                memcpy(img + 2*w*3*(w/2+i), face + i*w*3 + 3*w/2, w/2*3*sizeof(float));
            }
            // right
            cam = hemicubecams[imageindex];
            std::swap(cam.towards, cam.right);
            cam.right = -cam.right;
            if (typeindex == 5) {
                for (int i = 0; i < w*w*3; i++) face[i] = 1;
            }
            if (typeindex != 5) ui->meshWidget->renderManager()->readFromRender(&cam, face, rtype, true);
            if (typeindex&4) {
                transpose(face, w);
                hr->weightSideHemicube(face, w*w*M_PI/4);
                transpose(face, w);
            }
            for (int i = 0; i < w; ++i) {
                memcpy(img + 2*w*3*(w/2+i)+3*3*w/2, face + i*w*3, w/2*3*sizeof(float));
            }
        }
        ui->imageWidget->setFloatImage(img, 2*w, 2*w, 3);
        delete [] img;
        delete [] face;

        // TODO: !!! Display type
        // TODO: !!! Display stats (Total incoming light, etc.)

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
        } else {
            settings->setValue("lastmeshflags", "");
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
    ui->actionLoad_Vertex_Incident_Lighting->setEnabled(true);
    ui->wf_resolutionSlider->setEnabled(true);
    ui->wf_wallthresholdSlider->setEnabled(true);
    ui->showMeshCheckbox->setEnabled(true);
    ui->DebugWallfindingButton->setEnabled(true);
    ui->actionReload_Per_Vertex_Samples->setEnabled(true);

    ui->actionCursor_Mode_Select->setEnabled(true);
    ui->actionSelect_All_Vertices->setEnabled(true);
    ui->actionSelect_Ceiling_Vertices->setEnabled(true);
    ui->actionSelect_Floor_Vertices->setEnabled(true);
    ui->actionSelect_Wall_Vertices->setEnabled(true);
    ui->actionSelect_None->setEnabled(true);
    ui->actionAdd_To_Selection->setEnabled(true);
    ui->actionRemove_From_Selection->setEnabled(true);
    ui->actionIncrease_Selection_Brush_Size->setEnabled(true);
    ui->actionDecrease_Selection_Brush_Size->setEnabled(true);
    ui->actionSave_Selected_Vertices->setEnabled(true);
    ui->actionCommit_Selected_Vertices_as_Light->setEnabled(true);
    ui->actionCommit_Selected_Vertices_as_Line_Light->setEnabled(true);
    ui->actionSelect_Mesh_Component->setEnabled(true);

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
            settings->setValue("lazyloadimages", "1");
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
            settings->setValue("lazyloadimages", "0");
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
    QString imageflags = settings->value("lastimageflags", "").toString();
    QString meshflags = settings->value("lastmeshflags", "").toString();

    int r = settings->value("lazyloadimages", "0").toInt();
    if (r) {
        lazyloadimages = true;
        bool flipx = imageflags.contains("-flip_x");
        bool flipy = imageflags.contains("-flip_y");
        imgr = new FileImageServer(camfilename.toStdString(), flipx, flipy);
        imagesLoaded();

        QString cmd = settings->value("loadmesh_binary", "dataserver -meshfile %1 -p").toString();
        cmd = cmd.arg(meshfilename);
        cmd += meshflags;
        progressbar->setValue(0);
        SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
        workers.push_back(w);
        QThread* thread = new QThread;
        connect(thread, SIGNAL(started()), w, SLOT(run()));
        connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
        connect(w, SIGNAL(done()), this, SLOT(meshLoaded()));
        w->moveToThread(thread);
        thread->start();
    } else {
        lazyloadimages = false;
        QString cmd = settings->value("loadall_binary", "dataserver -camfile %1 -meshfile %2 -p").toString();
        cmd = cmd.arg(camfilename, meshfilename);
        cmd += imageflags;
        cmd += meshflags;
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
    ui->overlayTypeComboBox->clear();
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
            else if (label == LABEL_CEILING) ceilingindices.push_back(i);
        }
    }
    // Initialize light arrays
    {
        lights.clear();
        lightintensities.clear();
        std::set<unsigned char> lightids;
        for (int i = 0; i < mmgr->size(); i++) {
            unsigned char l = mmgr->getLabel(i,MeshManager::LABEL_CHANNEL);
            if (l) lightids.insert(l);
        }
        for (auto lightinfo : lightids) {
            int t = LIGHTTYPE(lightinfo);
            Light* l = NewLightFromLightType(t);
            lights.push_back(l);
            if ((l->typeId() & LIGHTTYPE_LINE) || (l->typeId() & LIGHTTYPE_POINT)) {
                lightintensities.push_back(new IRGBLight(l));
                for (int i = 0; i < 3; i++) lightintensities.back()->coef(i) = sqrt(1/3.);
            } else {
                lightintensities.push_back(new RGBLight(l));
            }
        }
    }
    checkEnableHemicubes();
}

void MainWindow::checkEnableHemicubes() {
    if (mmgr->hasSamples() && ui->computeLabelImagesButton->isEnabled()) {
        ui->hemicubeButton->setEnabled(true);
        ui->solveButton->setEnabled(true);
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
    if (lazyloadimages) {
        extraflags += settings->value("lastimageflags", "").toString();
        extraflags += " -noshm";
    }
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
        //ImageIO::flip((char*)imgr->getImageWriteable("labels", i), imgr->width(), imgr->height(), imgr->getImageType("labels").getSize());
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
        if (!hr) hr = new HemicubeRenderer(ui->meshWidget->renderManager(), ui->hemicubeResLineEdit->text().toInt());
        ui->meshWidget->updateMeshAuxiliaryData();
        hemicubecams.clear();
        int numsamples = ui->numSamplesLineEdit->text().toInt();
        std::default_random_engine generator;
        std::uniform_int_distribution<int> dist(0, wallindices.size());
        for (int i = 0; i < numsamples; ++i) {
            int n;
            do {
                n = dist(generator);
            } while (mmgr->getLabel(wallindices[n], MeshManager::LABEL_CHANNEL) > 0 || mmgr->getVertexSampleCount(wallindices[n]) == 0);

            CameraParams cam;
            cam.fov = 90;
            cam.height = ui->hemicubeResLineEdit->text().toInt();
            cam.width = ui->hemicubeResLineEdit->text().toInt();
            cam.pos = mmgr->VertexPosition(wallindices[n]);
            cam.towards = mmgr->VertexNormal(wallindices[n]);
            cam.up = R3yaxis_vector;
            if (room) cam.up = room->globaltransform.Inverse()*cam.up;
            else if (orientationtransform) cam.up = orientationtransform->globaltransform.Inverse()*cam.up;
            cam.towards -= cam.up.Dot(cam.towards)*cam.up;
            cam.towards.Normalize();
            cam.right = cam.towards;
            cam.right.Cross(cam.up);
            cam.focal_length = cam.width/2;
            hemicubecams.push_back(cam);
            hemicubeindices.push_back(wallindices[n]);
        }
        ui->meshWidget->setupCameras(hemicubecams);
        ui->showTrajectoryCheckbox->setEnabled(false);
        ui->showTrajectoryCheckbox->setChecked(false);
        ui->hemicubeButton->setText("Revert display to cameras");
        ui->imageTypeComboBox->clear();
        ui->imageTypeComboBox->insertItem(0, QString("Hemicube Face"));
        ui->imageTypeComboBox->insertItem(1, QString("Hemicube Face Visibility & Labels"));
        ui->imageTypeComboBox->insertItem(2, QString("Environment cubemap"));
        ui->imageTypeComboBox->insertItem(3, QString("Environment cubemap Visibility & Labels"));
        ui->imageTypeComboBox->insertItem(4, QString("Hemicube-weighted Environment cubemap"));
        ui->imageTypeComboBox->insertItem(5, QString("Hemicube weights"));
        ui->meshWidget->renderManager()->precalculateAverageSamples();
    } else {
        ui->meshWidget->setupCameras(imgr);
        ui->showTrajectoryCheckbox->setEnabled(true);
        ui->hemicubeButton->setText("Render hemicubes");
        ui->imageTypeComboBox->clear();
        for (int i = 0; i < imgr->getNumImageTypes(); ++i) {
            ui->imageTypeComboBox->insertItem(i, QString::fromStdString(imgr->getImageType(i).getName()));
        }
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


void MainWindow::on_solveButton_clicked()
{
    QString cmd = settings->value("solver_binary", "solverapp -meshfile %1 -p").toString();
    cmd = cmd.arg(meshfilename);
    QString extraflags = "";
    extraflags += " -hemicuberesolution " + ui->hemicubeResLineEdit->text();
    if (tempformfactors) {
        extraflags += " -inputbinaryfile " + tempformfactors->fileName();
    } else if (formfactorsfile.length() > 0) {
        extraflags += " -inputbinaryfile " + formfactorsfile;
    } else {
        tempformfactors = new QTemporaryFile();
        tempformfactors->open();
        tempformfactors->close();
        extraflags += " -outputbinaryfile " + tempformfactors->fileName();
    }
    if (lightintensities.size()) {
        templights = new QTemporaryFile();
        templights->open();
        templights->close();
        writeLightsToFile(templights->fileName().toStdString(), lightintensities);
        extraflags += " -inputlightfile " + templights->fileName();
    }
    // regularization
    cmd += extraflags;
    progressbar->setValue(0);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(data(QString)), this, SLOT(solveDataReceived(QString)));
    connect(w,SIGNAL(done()), this, SLOT(solveCompleted()));
    w->moveToThread(thread);
    thread->start();
}

void MainWindow::solveCompleted() {
    if (templights) {
        templights->close();
        delete templights;
        templights = NULL;
    }
    ui->actionSave_Vertex_Incident_Lighting->setEnabled(true);
}

void MainWindow::solveDataReceived(QString s) {
    QStringList toks = s.split(" ");
    if (room && toks[0] == QString("Material")) {
        roommodel::Material mat = roommodel::Material(toks[2].toFloat(), toks[3].toFloat(), toks[4].toFloat());
        if (toks[1] == QString("0")) {
            room->wallMaterial = mat;
        } else if (toks[1] == QString("1")) {
            room->ceilingMaterial = mat;
        } else if (toks[1] == QString("2")) {
            room->floorMaterial = mat;
        }
    }
}

void MainWindow::on_actionLoad_Vertex_Incident_Lighting_triggered()
{
    QString lwd = settings->value("lastworkingdirectory", "").toString();
    //QString cfile = settings->value("lastincidentlightfile", lwd).toString();
    QString datafilename = QFileDialog::getOpenFileName(this, "Load Per-Vertex Incident Lighting", lwd);
    if (!datafilename.isEmpty()) {
        QDir cwd = QDir(datafilename);
        cwd.cdUp();
        settings->setValue("lastworkingdirectory", cwd.canonicalPath());
        settings->setValue("lastincidentlightfile", datafilename);
        formfactorsfile = datafilename;
        if (tempformfactors) {
            tempformfactors->close();
            delete tempformfactors;
            tempformfactors = NULL;
        }
    }
}

// ---------------------------
// Door/Window Finding Actions
// ---------------------------
void MainWindow::on_edgeFilterButton_clicked()
{
    QString cmd = settings->value("compute_edges_binary", "edgeimageapp -camfile %1 -p").toString();
    cmd = cmd.arg(camfilename);
    QString extraflags = "";
    if (lazyloadimages) {
        extraflags += settings->value("lastimageflags", "").toString();
        extraflags += " -noshm";
    }
    cmd += extraflags;
    progressbar->setValue(0);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(done()), this, SLOT(edgeImagesLoaded()));
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
    lines.clear();
    QString cmd = settings->value("compute_rwo_binary", "linefindapp -camfile %1 -roommodel %2 -p").toString();
    cmd = cmd.arg(camfilename, roommodelfile);
    QString extraflags = "";
    if (lazyloadimages) {
        extraflags += settings->value("lastimageflags", "").toString();
        extraflags += " -noshm";
    }
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


void MainWindow::on_actionSave_Vertex_Incident_Lighting_triggered()
{
    QString lwd = settings->value("lastworkingdirectory", "").toString();
    QString datafilename = QFileDialog::getSaveFileName(this, "Save Per-Vertex Incident Lighting", lwd);
    if (!datafilename.isEmpty()) {
        QDir cwd = QDir(datafilename);
        cwd.cdUp();
        settings->setValue("lastworkingdirectory", cwd.canonicalPath());
        settings->setValue("lastincidentlightfile", datafilename);
        tempformfactors->copy(datafilename);
        formfactorsfile = datafilename;
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
        if (!filename.isEmpty()) {
            if (ui->meshWidget->getCurrentMapping() == TMO_GAMMA22) {
                mmgr->writePlyMesh(filename.toStdString(), 1/ui->meshWidget->getUpperBound(), 1/2.2);
            } else {
                mmgr->writePlyMesh(filename.toStdString(), 1/ui->meshWidget->getUpperBound());
            }
        }
    }
}


void MainWindow::on_actionSave_Light_Locations_triggered()
{
    QString lwd = settings->value("lastworkingdirectory", "").toString();
    QString filename = QFileDialog::getSaveFileName(this, "Save Lights", lwd, "Text files (*.txt)");
    if (!filename.isEmpty()) {
        writeLightsToFile(filename.toStdString(), lightintensities);
    }
}


void MainWindow::on_actionLoad_Light_Locations_triggered()
{
    QString lwd = settings->value("lastworkingdirectory", "").toString();
    QString filename = QFileDialog::getOpenFileName(this, "Save Lights", lwd, "Text files (*.txt)");
    if (!filename.isEmpty()) {
        readLightsFromFile(filename.toStdString(), lightintensities);
        for (int i = 0; i < lightintensities.size(); i++) {
            if (lightintensities[i]->typeId() & LIGHTTYPE_RGB) {
                lights[i] = ((RGBLight*)lightintensities[i])->getLight(0);
            } else if (lightintensities[i]->typeId() & LIGHTTYPE_IRGB) {
                lights[i] = ((IRGBLight*)lightintensities[i])->getLight();
            }
        }
        for (int i = 0; i < lights.size(); i++) {
            if (lights[i] && lights[i]->typeId() == (LIGHTTYPE_LINE | LIGHTTYPE_ENVMAP)) {
                LineLight* l = (LineLight*) lights[i];
                ui->meshWidget->addLine(l->getPosition(0), l->getPosition(1));
            }
        }
    }
}

// -----------------
// Interaction Modes
// -----------------
void MainWindow::on_actionCursor_Mode_Select_triggered()
{
    selectAction(ERUIGLWidget::INTERACTIONMODE_SELECT);
    ui->meshWidget->renderOptions()->showSelected();
}

void MainWindow::on_actionCursor_Mode_Trackball_triggered()
{
    selectAction(ERUIGLWidget::INTERACTIONMODE_TRACKBALL);
}
void MainWindow::selectAction(int n) {
    std::vector<QAction*> actions;
    actions.push_back(ui->actionCursor_Mode_Trackball);
    actions.push_back(ui->actionCursor_Mode_Select);

    for (int i = 0; i < ERUIGLWidget::NUMINTERACTIONMODES; i++) {
        actions[i]->setChecked(i == n);
    }
    ui->meshWidget->setInteractionMode(n);
}

// ------------------------
// Vertex Selection Actions
// ------------------------
void MainWindow::on_actionSelect_All_Vertices_triggered()
{
    std::vector<int> allindices;
    for (int i = 0; i < mmgr->size(); i++) allindices.push_back(i);
    ui->meshWidget->renderManager()->selectVertices(allindices);
    ui->meshWidget->renderOptions()->showSelected();
}

void MainWindow::on_actionSelect_None_triggered()
{
    ui->meshWidget->renderManager()->clearSelectedVertices();
    ui->meshWidget->renderOptions()->showSelected(false);
}

void MainWindow::on_actionAdd_To_Selection_triggered()
{
    ui->meshWidget->setVertexSelectMode(SELECT_UNION);
    ui->actionAdd_To_Selection->setChecked(true);
    ui->actionRemove_From_Selection->setChecked(false);
}

void MainWindow::on_actionRemove_From_Selection_triggered()
{
    ui->meshWidget->setVertexSelectMode(SELECT_DIFF);
    ui->actionAdd_To_Selection->setChecked(false);
    ui->actionRemove_From_Selection->setChecked(true);
}

void MainWindow::on_actionSelect_Wall_Vertices_triggered()
{
    ui->meshWidget->renderManager()->selectVertices(wallindices);
    ui->meshWidget->renderOptions()->showSelected();
}

void MainWindow::on_actionSelect_Floor_Vertices_triggered()
{
    ui->meshWidget->renderManager()->selectVertices(floorindices);
    ui->meshWidget->renderOptions()->showSelected();
}

void MainWindow::on_actionSelect_Ceiling_Vertices_triggered()
{
    ui->meshWidget->renderManager()->selectVertices(ceilingindices);
    ui->meshWidget->renderOptions()->showSelected();
}

void MainWindow::on_actionIncrease_Selection_Brush_Size_triggered()
{
    int currsize = ui->meshWidget->getVertexBrushSize();
    currsize += 3;
    ui->meshWidget->setVertexBrushSize(currsize);
    QString msg = QString("Set brush size to %1 px").arg(ui->meshWidget->getVertexBrushSize());
    ui->statusBar->showMessage(msg);
}

void MainWindow::on_actionDecrease_Selection_Brush_Size_triggered()
{
    int currsize = ui->meshWidget->getVertexBrushSize();
    if (currsize > 1) {
        currsize -= 3;
        ui->meshWidget->setVertexBrushSize(currsize);
    }
    QString msg = QString("Set brush size to %1 px").arg(ui->meshWidget->getVertexBrushSize());
    ui->statusBar->showMessage(msg);
}

void MainWindow::on_actionSave_Selected_Vertices_triggered()
{
    bool ok;
    int lbl = QInputDialog::getInt(this, "Commit selected vertices to label...", "Label: ", 1, 0, 1024, 1, &ok);
    if (ok) {
        std::vector<int> selected;
        ui->meshWidget->renderManager()->getSelectedVertices(selected);
        for (int i = 0; i < mmgr->size(); i++) {
            if (mmgr->getLabel(i, MeshManager::TYPE_CHANNEL) == lbl) {
                mmgr->setLabel(i, 0, MeshManager::TYPE_CHANNEL);
            }
        }
        for (int i = 0; i < selected.size(); i++) {
            mmgr->setLabel(selected[i], lbl, MeshManager::TYPE_CHANNEL);
        }

        ui->meshWidget->renderManager()->updateMeshAuxiliaryData();
        if (lbl == LABEL_WALL) {
            wallindices = selected;
        } else if (lbl == LABEL_CEILING) {
            ceilingindices = selected;
        } else if (lbl == LABEL_FLOOR) {
            floorindices = selected;
        }
    }
}

void MainWindow::on_actionCommit_Selected_Vertices_as_Light_triggered()
{
    bool ok;
    int lbl = QInputDialog::getInt(this, "Commit selected vertices to light...", "Label: ", 1, 0, 1024, 1, &ok);
    if (ok) {
        std::vector<int> selected;
        ui->meshWidget->renderManager()->getSelectedVertices(selected);
        for (int i = 0; i < mmgr->size(); i++) {
            if (mmgr->getLabel(i, MeshManager::LABEL_CHANNEL) == lbl) {
                mmgr->setLabel(i, 0, MeshManager::LABEL_CHANNEL);
            }
        }
        for (int i = 0; i < selected.size(); i++) {
            mmgr->setLabel(selected[i], lbl, MeshManager::LABEL_CHANNEL);
        }

        int t = LIGHTTYPE(lbl);
        int idx = LIGHTID(lbl) - 1;
        if (idx < lights.size() && lights[idx] && t != lights[idx]->typeId()) {
            Light* l = NewLightFromLightType(t);
            if (idx >= lights.size()) {
                lights.resize(idx+1, NULL);
                lightintensities.resize(idx+1, NULL);
            }
            lights[idx] = l;
            if ((l->typeId() & LIGHTTYPE_LINE) || (l->typeId() & LIGHTTYPE_POINT)) {
                lightintensities[idx] = (new IRGBLight(l));
                for (int i = 0; i < 3; i++) lightintensities[idx]->coef(i) = sqrt(1/3.);
            } else {
                lightintensities[idx] = (new RGBLight(l));
            }
        }
        ui->meshWidget->renderManager()->updateMeshAuxiliaryData();
    }
}

void MainWindow::on_actionCommit_Selected_Vertices_as_Line_Light_triggered()
{
    bool ok;
    int lid = QInputDialog::getInt(this, "Commit selected vertices to line light...", "Label: ", 1, 1, 1024, 1, &ok);
    unsigned char linelighttypeid = 5; // magic number - LIGHTTYPE_ENVMAP | LIGHTTYPE_LINE
    unsigned char lbl = ((unsigned char) lid) | (linelighttypeid << LIGHT_TYPESHIFT);
    if (ok) {
        std::vector<int> selected;
        ui->meshWidget->renderManager()->getSelectedVertices(selected);
        for (int i = 0; i < mmgr->size(); i++) {
            if (mmgr->getLabel(i, MeshManager::LABEL_CHANNEL) == lbl) {
                mmgr->setLabel(i, 0, MeshManager::LABEL_CHANNEL);
            }
        }
        std::vector<Eigen::Vector3d> selectedcoords;
        for (int i = 0; i < selected.size(); i++) {
            mmgr->setLabel(selected[i], lbl, MeshManager::LABEL_CHANNEL);
            R3Point p = mmgr->VertexPosition(selected[i]);
            selectedcoords.push_back(Eigen::Vector3d(p[0], p[1], p[2]));
        }

        int idx = lid - 1;
        if (idx >= lights.size()) {
            lights.resize(idx+1, NULL);
            lightintensities.resize(idx+1, NULL);
        }
        LineLight* l = new LineLight();
        l->computeFromPoints(selectedcoords);
        l->setSymmetric();
        ui->meshWidget->addLine(l->getPosition(0), l->getPosition(1));
        lights[idx] = l;
        lightintensities[idx] = (new IRGBLight(l));
        for (int i = 0; i < 3; i++) lightintensities[idx]->coef(i) = sqrt(1/3.);

        ui->meshWidget->renderManager()->updateMeshAuxiliaryData();
    }
}

void MainWindow::on_actionSelect_Mesh_Component_triggered()
{
    ui->meshWidget->selectNextComponent();
}
