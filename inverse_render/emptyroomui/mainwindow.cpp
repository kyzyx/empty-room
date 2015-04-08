#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QKeyEvent>
#include <QFileDialog>
#include <QIntValidator>

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
