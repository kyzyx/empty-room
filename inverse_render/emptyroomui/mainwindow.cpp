#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QKeyEvent>
#include <QFileDialog>
#include <QIntValidator>
#include <QCheckBox>
#include <QThread>
#include <QToolTip>

class MeshDialog : public QFileDialog
{
public:
 explicit MeshDialog(QWidget *parent=0) :
     QFileDialog( parent ),
     ccw(0)
    {
     QGridLayout* mainLayout = dynamic_cast<QGridLayout*>(layout());

     if ( ! mainLayout ) {
      assert(0); // in case of future changes
     } else {
      QHBoxLayout *hbl = new QHBoxLayout();

      // add some widgets
      ccw = new QCheckBox(QString("Flip Normals"), this);
      hbl->addWidget(ccw);
      ccw->setChecked(true);

      int numRows = mainLayout->rowCount();

      // add the new layout to the bottom of mainLayout
      // and span all columns
      mainLayout->addLayout( hbl, numRows,0,1,-1);
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
 explicit ImageDialog(QWidget *parent=0) :
     QFileDialog( parent ),
     flipx(0), flipy(0)
    {
     QGridLayout* mainLayout = dynamic_cast<QGridLayout*>(layout());

     if ( ! mainLayout ) {
      assert(0); // in case of future changes
     } else {
      QHBoxLayout *hbl = new QHBoxLayout();

      // add some widgets
      flipx = new QCheckBox(QString("Flip horizontally"), this);
      flipy = new QCheckBox(QString("Flip vertically"), this);
        hbl->addWidget(flipx);
        hbl->addWidget(flipy);
      int numRows = mainLayout->rowCount();

      // add the new layout to the bottom of mainLayout
      // and span all columns
      mainLayout->addLayout( hbl, numRows,0,1,-1);
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
private:
 QCheckBox *flipx;
 QCheckBox *flipy;
};

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    imgr(NULL), mmgr(NULL),
    typeindex(0), imageindex(0), room(NULL)
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
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

void MainWindow::updateImage(int idx, int type)
{
    imageindex = idx%imgr->size();
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
    ui->meshWidget->highlightCamera(idx);

    if (imageindex == 0) ui->prevImageButton->setEnabled(false);
    else ui->prevImageButton->setEnabled(true);
    if (imageindex == imgr->size() - 1) ui->nextImageButton->setEnabled(false);
    else ui->nextImageButton->setEnabled(true);
    ui->imageNumBox->setText(QString::number(imageindex));
}

void MainWindow::on_actionQuit_triggered()
{
    close();
}

void MainWindow::on_actionOpen_Mesh_triggered()
{
    MeshDialog mdialog(this);
    mdialog.setNameFilter("PLY Meshes (*.ply)");
    mdialog.setFileMode(QFileDialog::ExistingFile);
    mdialog.setOption(QFileDialog::DontUseNativeDialog);
    mdialog.setDirectory(settings->value("lastworkingdirectory", "").toString());
    mdialog.selectFile(settings->value("lastmeshfile", "").toString());
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
    ui->wf_resolutionSlider->setEnabled(true);
    ui->wf_wallthresholdSlider->setEnabled(true);
    mmgr = new MeshManager(meshfilename.toStdString());
    ui->meshWidget->setMeshManager(mmgr);
    allLoaded();
}

void MainWindow::on_actionOpen_Images_triggered()
{
    ImageDialog idialog(this);
    idialog.setNameFilter("Camera Files (*.cam)");
    idialog.setFileMode(QFileDialog::ExistingFile);
    idialog.setOption(QFileDialog::DontUseNativeDialog);
    idialog.setDirectory(settings->value("lastworkingdirectory", "").toString());
    idialog.selectFile(settings->value("lastcamerafile", "").toString());
    if (idialog.exec()) {
        camfilename = idialog.selectedFiles().first();
        settings->setValue("lastcamerafile", camfilename);
        QDir cwd = QDir(camfilename);
        cwd.cdUp();
        settings->setValue("lastworkingdirectory", cwd.canonicalPath());
        QString cmd = settings->value("loadimage_binary", "dataserver -camfile %1 -p").toString();
        cmd = cmd.arg(camfilename);
        QString extraflags = "";
        if (idialog.isFlipX()) extraflags = extraflags + " -flip_x";
        if (idialog.isFlipY()) extraflags = extraflags + " -flip_y";
        cmd += extraflags;
        settings->setValue("lastimageflags", extraflags);
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
void MainWindow::imagesLoaded() {
    imgr = new ImageManager(camfilename.toStdString());
    ui->imageTypeComboBox->setEnabled(true);
    ui->nextImageButton->setEnabled(true);
    ui->imageNumBox->setEnabled(true);
    ui->loadImageButton->setEnabled(true);

    ui->imageNumBox->setValidator(new QIntValidator(0, imgr->size()-1, this));

    ui->imageTypeComboBox->clear();
    for (int i = 0; i < imgr->getNumImageTypes(); ++i) {
        ui->imageTypeComboBox->insertItem(i, QString::fromStdString(imgr->getImageType(i).getName()));
    }
    updateImage(0, typeindex);
    allLoaded();
}

void MainWindow::onCameraSelection(int selected) {
    if (imageindex == selected) ui->meshWidget->lookThroughCamera(imgr->getCamera(selected));
    updateImage(selected);

}

void MainWindow::allLoaded() {
    if (imgr && mmgr) {
        ui->reprojectButton->setEnabled(true);
        ui->showCameraCheckbox->setEnabled(true);
        ui->showCurrentCameraCheckbox->setEnabled(true);
        // Render cameras
        ui->meshWidget->setupCameras(imgr);
        connect(ui->meshWidget, SIGNAL(cameraSelected(int)), this, SLOT(onCameraSelection(int)));
    }
}
#include <boost/interprocess/sync/sharable_lock.hpp>
void MainWindow::vertexDataLoaded() {
    if (mmgr->loadSamples()) {
        boost::interprocess::sharable_lock<MeshManager::shmutex> lock(*(mmgr->getMutex(MeshManager::NUM_CHANNELS,0)));
        ui->meshWidget->setupMeshColors();
    }
    ui->saveReprojectButton->setEnabled(true);
}
void MainWindow::partialVertexDataLoaded(int percent) {
    if (percent == 100) {
        vertexDataLoaded();
        return;
    }
    if (mmgr->loadSamples()) {
        boost::interprocess::sharable_lock<MeshManager::shmutex> lock(*(mmgr->getMutex(MeshManager::NUM_CHANNELS,0)));
        ui->meshWidget->setupMeshColors();
    }
}

void MainWindow::on_loadImageButton_clicked()
{
    int n = ui->imageNumBox->text().toInt();
    updateImage(n, typeindex);
}

void MainWindow::on_loadReprojectButton_clicked()
{
    if (mmgr) {
        QString lwd = settings->value("lastworkingdirectory", "").toString();
        QString cfile = settings->value("lastreprojectfile", lwd).toString();
        QString datafilename = QFileDialog::getOpenFileName(this, "Open Reprojection Samples", cfile);
        QDir cwd = QDir(datafilename);
        cwd.cdUp();
        settings->setValue("lastworkingdirectory", cwd.canonicalPath());
        settings->setValue("lastreprojectfile", datafilename);
        loadVertexData(meshfilename, datafilename);
    }
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

void MainWindow::on_actionLoad_Last_Intermediates_triggered()
{
    loadVertexData(meshfilename, settings->value("lastreprojectfile", "").toString());
}

void MainWindow::loadVertexData(QString meshfile, QString datafile) {
    QString cmd = settings->value("loaddata_binary", "dataserver -meshfile %1 -datafile %2 -p").toString();
    cmd = cmd.arg(meshfile, datafile);
    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(done()), this, SLOT(vertexDataLoaded()));
    w->moveToThread(thread);
    thread->start();
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

void MainWindow::on_reprojectButton_clicked()
{
    QString cmd = settings->value("reproject_binary", "reprojectapp -camfile %1 -meshfile %2 -p").toString();
    cmd = cmd.arg(camfilename, meshfilename);
    QString extraflags;
    cmd += extraflags;

    SubprocessWorker* w = new SubprocessWorker(NULL, cmd);
    workers.push_back(w);
    QThread* thread = new QThread;
    connect(thread, SIGNAL(started()), w, SLOT(run()));
    connect(w, SIGNAL(percentChanged(int)), progressbar, SLOT(setValue(int)));
    connect(w, SIGNAL(percentChanged(int)), this, SLOT(partialVertexDataLoaded(int)));
    w->moveToThread(thread);
    thread->start();
}

void MainWindow::on_saveReprojectButton_clicked()
{
    if (mmgr) {
        QString lwd = settings->value("lastworkingdirectory", "").toString();
        QString cfile = settings->value("lastreprojectfile", lwd).toString();
        QString datafilename = QFileDialog::getSaveFileName(this, "Open Reprojection Samples", cfile);
        QDir cwd = QDir(datafilename);
        cwd.cdUp();
        settings->setValue("lastworkingdirectory", cwd.canonicalPath());
        // settings->setValue("lastreprojectfile", datafilename);

        mmgr->writeSamplesToFile(datafilename.toStdString(), boost::bind(&QProgressBar::setValue, progressbar, _1));
    }
}

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
    room = new roommodel::RoomModel;
    roommodel::load(*room, temproommodel->fileName().toStdString());
    ui->meshWidget->setRoomModel(room);
    //delete temproommodel;
}

void MainWindow::showwfrestooltip(int v) {
    QToolTip::showText(QCursor::pos(), QString::number(v*0.001), ui->wf_resolutionSlider);
}

void MainWindow::showwfthresholdtooltip(int v) {
    QToolTip::showText(QCursor::pos(), QString::number(v), ui->wf_wallthresholdSlider);
}
