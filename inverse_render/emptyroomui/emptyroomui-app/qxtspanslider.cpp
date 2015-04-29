#include "qxtspanslider.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QApplication>
#include <QStylePainter>
#include <QStyleOptionSlider>

QxtSpanSliderPrivate::QxtSpanSliderPrivate() :
        lower(0),
        upper(0),
        lowerPos(0),
        upperPos(0),
        offset(0),
        position(0),
        lastPressed(QxtSpanSlider::NoHandle),
        mainControl(QxtSpanSlider::LowerHandle),
        lowerPressed(QStyle::SC_None),
        upperPressed(QStyle::SC_None),
        movement(QxtSpanSlider::FreeMovement),
        firstMovement(false),
        blockTracking(false)
{
}

void QxtSpanSliderPrivate::initStyleOption(QStyleOptionSlider* option, QxtSpanSlider::SpanHandle handle) const
{
    const QxtSpanSlider* p = pvt;
    p->initStyleOption(option);
    option->sliderPosition = (handle == QxtSpanSlider::LowerHandle ? lowerPos : upperPos);
    option->sliderValue = (handle == QxtSpanSlider::LowerHandle ? lower : upper);
}

int QxtSpanSliderPrivate::pixelPosToRangeValue(int pos) const
{
    QStyleOptionSlider opt;
    initStyleOption(&opt);

    int sliderMin = 0;
    int sliderMax = 0;
    int sliderLength = 0;
    const QSlider* p = pvt;
    const QRect gr = p->style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, p);
    const QRect sr = p->style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, p);
    if (p->orientation() == Qt::Horizontal)
    {
        sliderLength = sr.width();
        sliderMin = gr.x();
        sliderMax = gr.right() - sliderLength + 1;
    }
    else
    {
        sliderLength = sr.height();
        sliderMin = gr.y();
        sliderMax = gr.bottom() - sliderLength + 1;
    }
    return QStyle::sliderValueFromPosition(p->minimum(), p->maximum(), pos - sliderMin,
                                           sliderMax - sliderMin, opt.upsideDown);
}

void QxtSpanSliderPrivate::handleMousePress(const QPoint& pos, QStyle::SubControl& control, int value, QxtSpanSlider::SpanHandle handle)
{
    QStyleOptionSlider opt;
    initStyleOption(&opt, handle);
    QxtSpanSlider* p = pvt;
    const QStyle::SubControl oldControl = control;
    control = p->style()->hitTestComplexControl(QStyle::CC_Slider, &opt, pos, p);
    const QRect sr = p->style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, p);
    if (control == QStyle::SC_SliderHandle)
    {
        position = value;
        offset = pick(pos - sr.topLeft());
        lastPressed = handle;
        p->setSliderDown(true);
        emit p->sliderPressed(handle);
    }
    if (control != oldControl)
        p->update(sr);
}

void QxtSpanSliderPrivate::setupPainter(QPainter* painter, Qt::Orientation orientation, qreal x1, qreal y1, qreal x2, qreal y2) const
{
    QColor highlight = pvt->palette().color(QPalette::Highlight);
    QLinearGradient gradient(x1, y1, x2, y2);
    gradient.setColorAt(0, highlight.dark(120));
    gradient.setColorAt(1, highlight.light(108));
    painter->setBrush(gradient);

    if (orientation == Qt::Horizontal)
        painter->setPen(QPen(highlight.dark(130), 0));
    else
        painter->setPen(QPen(highlight.dark(150), 0));
}

void QxtSpanSliderPrivate::drawSpan(QStylePainter* painter, const QRect& rect) const
{
    QStyleOptionSlider opt;
    initStyleOption(&opt);
    const QSlider* p = pvt;

    // area
    QRect groove = p->style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, p);
    if (opt.orientation == Qt::Horizontal)
        groove.adjust(0, 0, -1, 0);
    else
        groove.adjust(0, 0, 0, -1);

    // pen & brush
    painter->setPen(QPen(p->palette().color(QPalette::Dark).light(110), 0));
    if (opt.orientation == Qt::Horizontal)
        setupPainter(painter, opt.orientation, groove.center().x(), groove.top(), groove.center().x(), groove.bottom());
    else
        setupPainter(painter, opt.orientation, groove.left(), groove.center().y(), groove.right(), groove.center().y());

    // draw groove
    painter->drawRect(rect.intersected(groove));
}

void QxtSpanSliderPrivate::drawHandle(QStylePainter* painter, QxtSpanSlider::SpanHandle handle) const
{
    QStyleOptionSlider opt;
    initStyleOption(&opt, handle);
    opt.subControls = QStyle::SC_SliderHandle;
    QStyle::SubControl pressed = (handle == QxtSpanSlider::LowerHandle ? lowerPressed : upperPressed);
    if (pressed == QStyle::SC_SliderHandle)
    {
        opt.activeSubControls = pressed;
        opt.state |= QStyle::State_Sunken;
    }
    painter->drawComplexControl(QStyle::CC_Slider, opt);
}

void QxtSpanSliderPrivate::triggerAction(QAbstractSlider::SliderAction action, bool main)
{
    int value = 0;
    bool no = false;
    bool up = false;
    const int min = pvt->minimum();
    const int max = pvt->maximum();
    const QxtSpanSlider::SpanHandle altControl = (mainControl == QxtSpanSlider::LowerHandle ? QxtSpanSlider::UpperHandle : QxtSpanSlider::LowerHandle);

    blockTracking = true;

    switch (action)
    {
    case QAbstractSlider::SliderSingleStepAdd:
        if ((main && mainControl == QxtSpanSlider::UpperHandle) || (!main && altControl == QxtSpanSlider::UpperHandle))
        {
            value = qBound(min, upper + pvt->singleStep(), max);
            up = true;
            break;
        }
        value = qBound(min, lower + pvt->singleStep(), max);
        break;
    case QAbstractSlider::SliderSingleStepSub:
        if ((main && mainControl == QxtSpanSlider::UpperHandle) || (!main && altControl == QxtSpanSlider::UpperHandle))
        {
            value = qBound(min, upper - pvt->singleStep(), max);
            up = true;
            break;
        }
        value = qBound(min, lower - pvt->singleStep(), max);
        break;
    case QAbstractSlider::SliderToMinimum:
        value = min;
        if ((main && mainControl == QxtSpanSlider::UpperHandle) || (!main && altControl == QxtSpanSlider::UpperHandle))
            up = true;
        break;
    case QAbstractSlider::SliderToMaximum:
        value = max;
        if ((main && mainControl == QxtSpanSlider::UpperHandle) || (!main && altControl == QxtSpanSlider::UpperHandle))
            up = true;
        break;
    case QAbstractSlider::SliderMove:
        if ((main && mainControl == QxtSpanSlider::UpperHandle) || (!main && altControl == QxtSpanSlider::UpperHandle))
            up = true;
    case QAbstractSlider::SliderNoAction:
        no = true;
        break;
    default:
        qWarning("QxtSpanSliderPrivate::triggerAction: Unknown action");
        break;
    }

    if (!no && !up)
    {
        if (movement == QxtSpanSlider::NoCrossing)
            value = qMin(value, upper);
        else if (movement == QxtSpanSlider::NoOverlapping)
            value = qMin(value, upper - 1);

        if (movement == QxtSpanSlider::FreeMovement && value > upper)
        {
            swapControls();
            pvt->setUpperPosition(value);
        }
        else
        {
            pvt->setLowerPosition(value);
        }
    }
    else if (!no)
    {
        if (movement == QxtSpanSlider::NoCrossing)
            value = qMax(value, lower);
        else if (movement == QxtSpanSlider::NoOverlapping)
            value = qMax(value, lower + 1);

        if (movement == QxtSpanSlider::FreeMovement && value < lower)
        {
            swapControls();
            pvt->setLowerPosition(value);
        }
        else
        {
            pvt->setUpperPosition(value);
        }
    }

    blockTracking = false;
    pvt->setLowerValue(lowerPos);
    pvt->setUpperValue(upperPos);
}

void QxtSpanSliderPrivate::swapControls()
{
    qSwap(lower, upper);
    qSwap(lowerPressed, upperPressed);
    lastPressed = (lastPressed == QxtSpanSlider::LowerHandle ? QxtSpanSlider::UpperHandle : QxtSpanSlider::LowerHandle);
    mainControl = (mainControl == QxtSpanSlider::LowerHandle ? QxtSpanSlider::UpperHandle : QxtSpanSlider::LowerHandle);
}

void QxtSpanSliderPrivate::updateRange(int min, int max)
{
    Q_UNUSED(min);
    Q_UNUSED(max);
    // setSpan() takes care of keeping span in range
    pvt->setSpan(lower, upper);
}

void QxtSpanSliderPrivate::movePressedHandle()
{
    switch (lastPressed)
    {
        case QxtSpanSlider::LowerHandle:
            if (lowerPos != lower)
            {
                bool main = (mainControl == QxtSpanSlider::LowerHandle);
                triggerAction(QAbstractSlider::SliderMove, main);
            }
            break;
        case QxtSpanSlider::UpperHandle:
            if (upperPos != upper)
            {
                bool main = (mainControl == QxtSpanSlider::UpperHandle);
                triggerAction(QAbstractSlider::SliderMove, main);
            }
            break;
        default:
            break;
    }
}

/*!
    \class QxtSpanSlider
    \inmodule QxtWidgets
    \brief The QxtSpanSlider widget is a QSlider with two handles.

    QxtSpanSlider is a slider with two handles. QxtSpanSlider is
    handy for letting user to choose an span between min/max.

    The span color is calculated based on QPalette::Highlight.

    The keys are bound according to the following table:
    \table
    \header \o Orientation    \o Key           \o Handle
    \row    \o Qt::Horizontal \o Qt::Key_Left  \o lower
    \row    \o Qt::Horizontal \o Qt::Key_Right \o lower
    \row    \o Qt::Horizontal \o Qt::Key_Up    \o upper
    \row    \o Qt::Horizontal \o Qt::Key_Down  \o upper
    \row    \o Qt::Vertical   \o Qt::Key_Up    \o lower
    \row    \o Qt::Vertical   \o Qt::Key_Down  \o lower
    \row    \o Qt::Vertical   \o Qt::Key_Left  \o upper
    \row    \o Qt::Vertical   \o Qt::Key_Right \o upper
    \endtable

    Keys are bound by the time the slider is created. A key is bound
    to same handle for the lifetime of the slider. So even if the handle
    representation might change from lower to upper, the same key binding
    remains.

    \image qxtspanslider.png "QxtSpanSlider in Plastique style."

    \bold {Note:} QxtSpanSlider inherits QSlider for implementation specific
    reasons. Adjusting any single handle specific properties like
    \list
    \o QAbstractSlider::sliderPosition
    \o QAbstractSlider::value
    \endlist
    has no effect. However, all slider specific properties like
    \list
    \o QAbstractSlider::invertedAppearance
    \o QAbstractSlider::invertedControls
    \o QAbstractSlider::minimum
    \o QAbstractSlider::maximum
    \o QAbstractSlider::orientation
    \o QAbstractSlider::pageStep
    \o QAbstractSlider::singleStep
    \o QSlider::tickInterval
    \o QSlider::tickPosition
    \endlist
    are taken into consideration.
 */

/*!
    \enum QxtSpanSlider::HandleMovementMode

    This enum describes the available handle movement modes.

    \value FreeMovement The handles can be moved freely.
    \value NoCrossing The handles cannot cross, but they can still overlap each other. The lower and upper values can be the same.
    \value NoOverlapping The handles cannot overlap each other. The lower and upper values cannot be the same.
 */

/*!
    \enum QxtSpanSlider::SpanHandle

    This enum describes the available span handles.

    \omitvalue NoHandle \omit Internal only (for now). \endomit
    \value LowerHandle The lower boundary handle.
    \value UpperHandle The upper boundary handle.
 */

/*!
    \fn QxtSpanSlider::lowerValueChanged(int lower)

    This signal is emitted whenever the \a lower value has changed.
 */

/*!
    \fn QxtSpanSlider::upperValueChanged(int upper)

    This signal is emitted whenever the \a upper value has changed.
 */

/*!
    \fn QxtSpanSlider::spanChanged(int lower, int upper)

    This signal is emitted whenever both the \a lower and the \a upper
    values have changed ie. the span has changed.
 */

/*!
    \fn QxtSpanSlider::lowerPositionChanged(int lower)

    This signal is emitted whenever the \a lower position has changed.
 */

/*!
    \fn QxtSpanSlider::upperPositionChanged(int upper)

    This signal is emitted whenever the \a upper position has changed.
 */

/*!
    \fn QxtSpanSlider::sliderPressed(SpanHandle handle)

    This signal is emitted whenever the \a handle has been pressed.
 */

/*!
    Constructs a new QxtSpanSlider with \a parent.
 */
QxtSpanSlider::QxtSpanSlider(QWidget* parent) : QSlider(parent)
{
    pvt = new QxtSpanSliderPrivate();
    pvt->pvt = this;
    connect(this, SIGNAL(rangeChanged(int, int)), pvt, SLOT(updateRange(int, int)));
    connect(this, SIGNAL(sliderReleased()), pvt, SLOT(movePressedHandle()));
}

/*!
    Constructs a new QxtSpanSlider with \a orientation and \a parent.
 */
QxtSpanSlider::QxtSpanSlider(Qt::Orientation orientation, QWidget* parent) : QSlider(orientation, parent)
{
    pvt = new QxtSpanSliderPrivate();
    pvt->pvt = this;
    connect(this, SIGNAL(rangeChanged(int, int)), pvt, SLOT(updateRange(int, int)));
    connect(this, SIGNAL(sliderReleased()), pvt, SLOT(movePressedHandle()));
}

/*!
    Destructs the span slider.
 */
QxtSpanSlider::~QxtSpanSlider()
{
    delete pvt;
}

/*!
    \property QxtSpanSlider::handleMovementMode
    \brief the handle movement mode
 */
QxtSpanSlider::HandleMovementMode QxtSpanSlider::handleMovementMode() const
{
    return pvt->movement;
}

void QxtSpanSlider::setHandleMovementMode(QxtSpanSlider::HandleMovementMode mode)
{
    pvt->movement = mode;
}

/*!
    \property QxtSpanSlider::lowerValue
    \brief the lower value of the span
 */
int QxtSpanSlider::lowerValue() const
{
    return qMin(pvt->lower, pvt->upper);
}

void QxtSpanSlider::setLowerValue(int lower)
{
    setSpan(lower, pvt->upper);
}

/*!
    \property QxtSpanSlider::upperValue
    \brief the upper value of the span
 */
int QxtSpanSlider::upperValue() const
{
    return qMax(pvt->lower, pvt->upper);
}

void QxtSpanSlider::setUpperValue(int upper)
{
    setSpan(pvt->lower, upper);
}

/*!
    Sets the span from \a lower to \a upper.
 */
void QxtSpanSlider::setSpan(int lower, int upper)
{
    const int low = qBound(minimum(), qMin(lower, upper), maximum());
    const int upp = qBound(minimum(), qMax(lower, upper), maximum());
    if (low != pvt->lower || upp != pvt->upper)
    {
        if (low != pvt->lower)
        {
            pvt->lower = low;
            pvt->lowerPos = low;
            emit lowerValueChanged(low);
        }
        if (upp != pvt->upper)
        {
            pvt->upper = upp;
            pvt->upperPos = upp;
            emit upperValueChanged(upp);
        }
        emit spanChanged(pvt->lower, pvt->upper);
        update();
    }
}

/*!
    \property QxtSpanSlider::lowerPosition
    \brief the lower position of the span
 */
int QxtSpanSlider::lowerPosition() const
{
    return pvt->lowerPos;
}

void QxtSpanSlider::setLowerPosition(int lower)
{
    if (pvt->lowerPos != lower)
    {
        pvt->lowerPos = lower;
        if (!hasTracking())
            update();
        if (isSliderDown())
            emit lowerPositionChanged(lower);
        if (hasTracking() && !pvt->blockTracking)
        {
            bool main = (pvt->mainControl == QxtSpanSlider::LowerHandle);
            pvt->triggerAction(SliderMove, main);
        }
    }
}

/*!
    \property QxtSpanSlider::upperPosition
    \brief the upper position of the span
 */
int QxtSpanSlider::upperPosition() const
{
    return pvt->upperPos;
}

void QxtSpanSlider::setUpperPosition(int upper)
{
    if (pvt->upperPos != upper)
    {
        pvt->upperPos = upper;
        if (!hasTracking())
            update();
        if (isSliderDown())
            emit upperPositionChanged(upper);
        if (hasTracking() && !pvt->blockTracking)
        {
            bool main = (pvt->mainControl == QxtSpanSlider::UpperHandle);
            pvt->triggerAction(SliderMove, main);
        }
    }
}

/*!
    \reimp
 */
void QxtSpanSlider::keyPressEvent(QKeyEvent* event)
{
    QSlider::keyPressEvent(event);

    bool main = true;
    SliderAction action = SliderNoAction;
    switch (event->key())
    {
    case Qt::Key_Left:
        main   = (orientation() == Qt::Horizontal);
        action = !invertedAppearance() ? SliderSingleStepSub : SliderSingleStepAdd;
        break;
    case Qt::Key_Right:
        main   = (orientation() == Qt::Horizontal);
        action = !invertedAppearance() ? SliderSingleStepAdd : SliderSingleStepSub;
        break;
    case Qt::Key_Up:
        main   = (orientation() == Qt::Vertical);
        action = invertedControls() ? SliderSingleStepSub : SliderSingleStepAdd;
        break;
    case Qt::Key_Down:
        main   = (orientation() == Qt::Vertical);
        action = invertedControls() ? SliderSingleStepAdd : SliderSingleStepSub;
        break;
    case Qt::Key_Home:
        main   = (pvt->mainControl == QxtSpanSlider::LowerHandle);
        action = SliderToMinimum;
        break;
    case Qt::Key_End:
        main   = (pvt->mainControl == QxtSpanSlider::UpperHandle);
        action = SliderToMaximum;
        break;
    default:
        event->ignore();
        break;
    }

    if (action)
        pvt->triggerAction(action, main);
}

/*!
    \reimp
 */
void QxtSpanSlider::mousePressEvent(QMouseEvent* event)
{
    if (minimum() == maximum() || (event->buttons() ^ event->button()))
    {
        event->ignore();
        return;
    }

    pvt->handleMousePress(event->pos(), pvt->upperPressed, pvt->upper, QxtSpanSlider::UpperHandle);
    if (pvt->upperPressed != QStyle::SC_SliderHandle)
        pvt->handleMousePress(event->pos(), pvt->lowerPressed, pvt->lower, QxtSpanSlider::LowerHandle);

    pvt->firstMovement = true;
    event->accept();
}

/*!
    \reimp
 */
void QxtSpanSlider::mouseMoveEvent(QMouseEvent* event)
{
    if (pvt->lowerPressed != QStyle::SC_SliderHandle && pvt->upperPressed != QStyle::SC_SliderHandle)
    {
        event->ignore();
        return;
    }

    QStyleOptionSlider opt;
    pvt->initStyleOption(&opt);
    const int m = style()->pixelMetric(QStyle::PM_MaximumDragDistance, &opt, this);
    int newPosition = pvt->pixelPosToRangeValue(pvt->pick(event->pos()) - pvt->offset);
    if (m >= 0)
    {
        const QRect r = rect().adjusted(-m, -m, m, m);
        if (!r.contains(event->pos()))
        {
            newPosition = pvt->position;
        }
    }

    // pick the preferred handle on the first movement
    if (pvt->firstMovement)
    {
        if (pvt->lower == pvt->upper)
        {
            if (newPosition < lowerValue())
            {
                pvt->swapControls();
                pvt->firstMovement = false;
            }
        }
        else
        {
            pvt->firstMovement = false;
        }
    }

    if (pvt->lowerPressed == QStyle::SC_SliderHandle)
    {
        if (pvt->movement == NoCrossing)
            newPosition = qMin(newPosition, upperValue());
        else if (pvt->movement == NoOverlapping)
            newPosition = qMin(newPosition, upperValue() - 1);

        if (pvt->movement == FreeMovement && newPosition > pvt->upper)
        {
            pvt->swapControls();
            setUpperPosition(newPosition);
        }
        else
        {
            setLowerPosition(newPosition);
        }
    }
    else if (pvt->upperPressed == QStyle::SC_SliderHandle)
    {
        if (pvt->movement == NoCrossing)
            newPosition = qMax(newPosition, lowerValue());
        else if (pvt->movement == NoOverlapping)
            newPosition = qMax(newPosition, lowerValue() + 1);

        if (pvt->movement == FreeMovement && newPosition < pvt->lower)
        {
            pvt->swapControls();
            setLowerPosition(newPosition);
        }
        else
        {
            setUpperPosition(newPosition);
        }
    }
    event->accept();
}

/*!
    \reimp
 */
void QxtSpanSlider::mouseReleaseEvent(QMouseEvent* event)
{
    QSlider::mouseReleaseEvent(event);
    setSliderDown(false);
    pvt->lowerPressed = QStyle::SC_None;
    pvt->upperPressed = QStyle::SC_None;
    update();
}

/*!
    \reimp
 */
void QxtSpanSlider::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    QStylePainter painter(this);

    // groove & ticks
    QStyleOptionSlider opt;
    pvt->initStyleOption(&opt);
    opt.sliderValue = 0;
    opt.sliderPosition = 0;
    opt.subControls = QStyle::SC_SliderGroove | QStyle::SC_SliderTickmarks;
    painter.drawComplexControl(QStyle::CC_Slider, opt);

    // handle rects
    opt.sliderPosition = pvt->lowerPos;
    const QRect lr = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
    const int lrv  = pvt->pick(lr.center());
    opt.sliderPosition = pvt->upperPos;
    const QRect ur = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
    const int urv  = pvt->pick(ur.center());

    // span
    const int minv = qMin(lrv, urv);
    const int maxv = qMax(lrv, urv);
    const QPoint c = QRect(lr.center(), ur.center()).center();
    QRect spanRect;
    if (orientation() == Qt::Horizontal)
        spanRect = QRect(QPoint(minv, c.y() - 2), QPoint(maxv, c.y() + 1));
    else
        spanRect = QRect(QPoint(c.x() - 2, minv), QPoint(c.x() + 1, maxv));
    pvt->drawSpan(&painter, spanRect);

    // handles
    switch (pvt->lastPressed)
    {
    case QxtSpanSlider::LowerHandle:
        pvt->drawHandle(&painter, QxtSpanSlider::UpperHandle);
        pvt->drawHandle(&painter, QxtSpanSlider::LowerHandle);
        break;
    case QxtSpanSlider::UpperHandle:
    default:
        pvt->drawHandle(&painter, QxtSpanSlider::LowerHandle);
        pvt->drawHandle(&painter, QxtSpanSlider::UpperHandle);
        break;
    }
}
