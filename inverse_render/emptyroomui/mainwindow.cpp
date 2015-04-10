#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QKeyEvent>
#include <QFileDialog>
#include <QIntValidator>
#include <QCheckBox>
#include <QThread>

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
    typeindex(0), imageindex(0)
{
    ui->setupUi(this);

    progressbar = new QProgressBar(ui->statusBar);
    progressbar->setMaximumSize(200,ui->statusBar->height());
    ui->statusBar->addPermanentWidget(progressbar);
    progressbar->setValue(100);
    settingsfilename = QApplication::applicationDirPath() + "/settings.ini";
    settings = new QSettings(settingsfilename, QSettings::NativeFormat);
}

MainWindow::~MainWindow()
{
    delete ui;
    for (int i = 0; i < workers.size(); ++i) {
        workers[i]->terminate();
    }
    if (mmgr) delete mmgr;
    if (imgr) delete imgr;
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
    mdialog.selectFile(settings->value("lastmeshfile", "").toString());
    if (mdialog.exec()) {
        meshfilename = mdialog.selectedFiles().first();
        settings->setValue("lastmeshfile", meshfilename);
        QString cmd = settings->value("loadmesh_binary", "dataserver -meshfile %1 -p").toString();
        cmd = cmd.arg(meshfilename);
        if (mdialog.isCcw()) cmd = cmd + " -ccw";

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
    if (imgr) {
        ui->reprojectButton->setEnabled(true);
    }
    mmgr = new MeshManager(meshfilename.toStdString());
    ui->meshWidget->setMeshManager(mmgr);
    ui->meshWidget->repaint();
}

void MainWindow::on_actionOpen_Images_triggered()
{
    ImageDialog idialog(this);
    idialog.setNameFilter("Camera Files (*.cam)");
    idialog.setFileMode(QFileDialog::ExistingFile);
    idialog.setOption(QFileDialog::DontUseNativeDialog);
    idialog.selectFile(settings->value("lastcamfile", "").toString());
    if (idialog.exec()) {
        camfilename = idialog.selectedFiles().first();
        settings->setValue("lastcamerafile", camfilename);
        QString cmd = settings->value("loadimage_binary", "dataserver -camfile %1 -p").toString();
        cmd = cmd.arg(camfilename);
        if (idialog.isFlipX()) cmd = cmd + " -flip_x";
        if (idialog.isFlipY()) cmd = cmd + " -flip_y";
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
    if (mmgr) {
        ui->reprojectButton->setEnabled(true);
    }
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
}

void MainWindow::on_prevImageButton_clicked()
{
    updateImage(--imageindex, typeindex);
    ui->nextImageButton->setEnabled(true);
    if (imageindex == 0) ui->prevImageButton->setEnabled(false);
}

void MainWindow::on_nextImageButton_clicked()
{
    updateImage(++imageindex, typeindex);
    ui->prevImageButton->setEnabled(true);
    if (imageindex == imgr->size()-1) ui->nextImageButton->setEnabled(false);
}
void MainWindow::on_imageTypeComboBox_currentIndexChanged(int index)
{
    updateImage(imageindex, index);
}

void MainWindow::on_loadImageButton_clicked()
{
    int n = ui->imageNumBox->text().toInt();
    updateImage(n, typeindex);
}

void MainWindow::on_loadReprojectButton_clicked()
{
    if (mmgr) {
        QString samplesfile = QFileDialog::getOpenFileName(this, "Open Reprojection Samples");
        mmgr->readSamplesFromFile(samplesfile.toStdString());
        ui->meshWidget->setupMeshColors();
    }
}
