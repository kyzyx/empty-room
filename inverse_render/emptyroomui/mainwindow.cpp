#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QKeyEvent>
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    imgr(NULL), mmgr(NULL),
    typeindex(0), imageindex(0)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
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
}

void MainWindow::on_actionQuit_triggered()
{
    close();
}

void MainWindow::on_actionOpen_Mesh_triggered()
{
    QString meshfilename = QFileDialog::getOpenFileName(this,"Open Mesh", "", "PLY Meshes (*.ply)");
    // Trigger server thread, add progress bar, then
    mmgr = new MeshManager(meshfilename.toStdString());
    // Render mesh
}

void MainWindow::on_actionOpen_Images_triggered()
{
    QString camfilename = QFileDialog::getOpenFileName(this,"Open Camera File", "", "Camera Files (*.cam)");
    // Trigger server thread, add progress bar, then
    imgr = new ImageManager(camfilename.toStdString());
    updateImage(0,0);
}
