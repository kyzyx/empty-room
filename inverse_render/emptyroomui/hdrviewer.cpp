#include "hdrviewer.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QToolTip>
#include <QCheckBox>

HDRViewer::HDRViewer(QWidget *parent, QWidget* hdrwidget, HDRGlHelper* hdrcontrol) :
    QWidget(parent), state(STATE_FIXED), renderwidget(hdrwidget), rendercontrol(hdrcontrol)
{
    init();
}
void HDRViewer::init() {
    setFocusPolicy(Qt::StrongFocus);
    connect(rendercontrol, SIGNAL(fixParams(int,int,int)), this, SLOT(fixRange(int,int,int)));
    connect(rendercontrol, SIGNAL(suggestRange(int,int)), this, SLOT(setSuggestRange(int,int)));

    QWidget* container = new QWidget(this);

    QCheckBox* red_check = new QCheckBox(this);
    red_check->setText("R");
    red_check->setChecked(true);
    connect(red_check, SIGNAL(toggled(bool)), rendercontrol, SLOT(renderRedChannel(bool)));
    connect(red_check, SIGNAL(toggled(bool)), this, SLOT(update()));
    QCheckBox* green_check = new QCheckBox(this);
    green_check->setText("G");
    green_check->setChecked(true);
    connect(green_check, SIGNAL(toggled(bool)), rendercontrol, SLOT(renderGreenChannel(bool)));
    connect(green_check, SIGNAL(toggled(bool)), this, SLOT(update()));
    QCheckBox* blue_check = new QCheckBox(this);
    blue_check->setText("B");
    blue_check->setChecked(true);
    connect(blue_check, SIGNAL(toggled(bool)), rendercontrol, SLOT(renderBlueChannel(bool)));
    connect(blue_check, SIGNAL(toggled(bool)), this, SLOT(update()));

    slider = new QxtSpanSlider(Qt::Horizontal, container);
    slider->setEnabled(false);
    slider->setMinimum(0);
    slider->setMaximum(100);
    slider->setMinimumHeight(30);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    connect(slider, SIGNAL(spanChanged(int, int)), this, SLOT(userEditRange(int, int)));
    connect(slider, SIGNAL(spanChanged(int,int)), rendercontrol, SLOT(setScale(int,int)));
    connect(slider, SIGNAL(spanChanged(int, int)), this, SLOT(notifyUpdateRange(int,int)));
    connect(slider, SIGNAL(upperPositionChanged(int)), this, SLOT(showTooltip()));
    connect(slider, SIGNAL(lowerPositionChanged(int)), this, SLOT(showTooltip()));

    tmo = new QComboBox(container);
    tmo->insertItem(TMO_LINEAR, "Linear Mapping");
    tmo->insertItem(TMO_LOG, "Logarithmic Mapping");
    tmo->insertItem(TMO_GAMMA22, "Gamma 2.2");
    tmo->insertItem(TMO_BITWISE_INT, "Float-to-int mapping");
    tmo->setCurrentIndex(0);
    tmo->setEnabled(false);
    tmo->setMinimumHeight(30);
    tmo->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    connect(tmo, SIGNAL(activated(int)), rendercontrol, SLOT(setMapping(int)));
    connect(tmo, SIGNAL(activated(int)), this, SLOT(notifyUpdateMappingType(int)));

    QHBoxLayout* ll = new QHBoxLayout(container);
    ll->addWidget(red_check);
    ll->addWidget(green_check);
    ll->addWidget(blue_check);
    ll->addWidget(slider);
    ll->addWidget(tmo);
    container->setLayout(ll);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(renderwidget);
    renderwidget->setFocusPolicy(Qt::StrongFocus);
    renderwidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    layout->addWidget(container);
    setLayout(layout);
}
HDRViewer::~HDRViewer() {
    delete slider;
    delete tmo;
}

void HDRViewer::setSuggestRange(int lo, int hi) {
    switch (state) {
        case STATE_FIXED:
            slider->setEnabled(true);
            tmo->setEnabled(true);
        case STATE_SUGGESTED:
            slider->setUpperPosition(hi);
            slider->setLowerPosition(lo);
            state = STATE_SUGGESTED;
            break;
        case STATE_EDITED:
            break;
    }
}

void HDRViewer::fixRange(int lo, int hi, int v) {
    slider->setUpperPosition(hi);
    slider->setLowerPosition(lo);
    tmo->setCurrentIndex(v);
    slider->setEnabled(false);
    tmo->setEnabled(false);
    state = STATE_FIXED;
}

void HDRViewer::userEditRange(int lo, int hi) {
    state = STATE_EDITED;
    double lower = HDRGlHelper::LINTOLOG(lo);
    double upper = HDRGlHelper::LINTOLOG(hi);
    slider->setToolTip(QString("%1 - %2").arg(QString::number(lower), QString::number(upper)));
}

void HDRViewer::showTooltip() {
    double lower = HDRGlHelper::LINTOLOG(slider->lowerValue());
    double upper = HDRGlHelper::LINTOLOG(slider->upperValue());
    QToolTip::showText(QCursor::pos(), QString("%1 - %2").arg(QString::number(lower), QString::number(upper)), slider);
}
