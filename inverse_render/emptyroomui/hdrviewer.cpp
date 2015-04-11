#include "hdrviewer.h"

HDRViewer::HDRViewer(QWidget *parent, QWidget* hdrwidget, HDRGlHelper* hdrcontrol) :
    QWidget(parent), state(STATE_FIXED), renderwidget(hdrwidget), rendercontrol(hdrcontrol)
{
    init();
}
void HDRViewer::init() {
    connect(rendercontrol, SIGNAL(fixParams(int,int,int)), this, SLOT(fixRange(int,int,int)));
    connect(rendercontrol, SIGNAL(suggestRange(int,int)), this, SLOT(setSuggestRange(int,int)));

    slider = new QxtSpanSlider(Qt::Horizontal, this);
    slider->setEnabled(false);
    slider->setMinimum(0);
    slider->setMaximum(100);
    slider->setMinimumHeight(30);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    connect(slider, SIGNAL(spanChanged(int, int)), this, SLOT(userEditRange(int, int)));
    connect(slider, SIGNAL(spanChanged(int,int)), rendercontrol, SLOT(setScale(int,int)));
    connect(slider, SIGNAL(spanChanged(int, int)), this, SLOT(notifyUpdateRange(int,int)));

    tmo = new QComboBox(this);
    tmo->insertItem(TMO_LINEAR, "Linear Mapping");
    tmo->insertItem(TMO_LOG, "Logarithmic Mapping");
    tmo->insertItem(TMO_GAMMA22, "Gamma 2.2");
    tmo->setCurrentIndex(0);
    tmo->setEnabled(false);
    tmo->setMinimumHeight(30);
    tmo->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    connect(tmo, SIGNAL(activated(int)), rendercontrol, SLOT(setMapping(int)));
    connect(tmo, SIGNAL(activated(int)), this, SLOT(notifyUpdateMappingType(int)));

    layout = new QGridLayout(this);
    layout->addWidget(renderwidget,0,0,1,2);
    layout->setAlignment(renderwidget,Qt::AlignTop);
    renderwidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(slider,1,0);
    layout->addWidget(tmo,1,1);
    layout->setColumnStretch(0,2);
    layout->setColumnStretch(1,1);
    setLayout(layout);
}
HDRViewer::~HDRViewer() {
    delete slider;
    delete tmo;
    delete layout;
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
}
