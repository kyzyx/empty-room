#ifndef QXTSPANSLIDER_H
#define QXTSPANSLIDER_H

#include <QSlider>
#include <QStylePainter>

class QxtSpanSliderPrivate;

class QxtSpanSlider : public QSlider
{
    Q_OBJECT

    Q_PROPERTY(int lowerValue READ lowerValue WRITE setLowerValue)
    Q_PROPERTY(int upperValue READ upperValue WRITE setUpperValue)
    Q_PROPERTY(int lowerPosition READ lowerPosition WRITE setLowerPosition)
    Q_PROPERTY(int upperPosition READ upperPosition WRITE setUpperPosition)
    Q_PROPERTY(HandleMovementMode handleMovementMode READ handleMovementMode WRITE setHandleMovementMode)
    Q_ENUMS(HandleMovementMode)

public:
    explicit QxtSpanSlider(QWidget* parent = 0);
    explicit QxtSpanSlider(Qt::Orientation orientation, QWidget* parent = 0);
    virtual ~QxtSpanSlider();

    enum HandleMovementMode
    {
        FreeMovement,
        NoCrossing,
        NoOverlapping
    };

    enum SpanHandle
    {
        NoHandle,
        LowerHandle,
        UpperHandle
    };

    HandleMovementMode handleMovementMode() const;
    void setHandleMovementMode(HandleMovementMode mode);

    int lowerValue() const;
    int upperValue() const;

    int lowerPosition() const;
    int upperPosition() const;

public slots:
    void setLowerValue(int lower);
    void setUpperValue(int upper);
    void setSpan(int lower, int upper);

    void setLowerPosition(int lower);
    void setUpperPosition(int upper);

signals:
    void spanChanged(int lower, int upper);
    void lowerValueChanged(int lower);
    void upperValueChanged(int upper);

    void lowerPositionChanged(int lower);
    void upperPositionChanged(int upper);

    void sliderPressed(SpanHandle handle);

protected:
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void paintEvent(QPaintEvent* event);

private:
    friend class QxtSpanSliderPrivate;
    QxtSpanSliderPrivate* pvt;
};

class QxtSpanSliderPrivate : public QObject
{
    Q_OBJECT

public:
    QxtSpanSliderPrivate();
    void initStyleOption(QStyleOptionSlider* option, QxtSpanSlider::SpanHandle handle = QxtSpanSlider::UpperHandle) const;
    int pick(const QPoint& pt) const
    {
        return pvt->orientation() == Qt::Horizontal ? pt.x() : pt.y();
    }
    int pixelPosToRangeValue(int pos) const;
    void handleMousePress(const QPoint& pos, QStyle::SubControl& control, int value, QxtSpanSlider::SpanHandle handle);
    void drawHandle(QStylePainter* painter, QxtSpanSlider::SpanHandle handle) const;
    void setupPainter(QPainter* painter, Qt::Orientation orientation, qreal x1, qreal y1, qreal x2, qreal y2) const;
    void drawSpan(QStylePainter* painter, const QRect& rect) const;
    void triggerAction(QAbstractSlider::SliderAction action, bool main);
    void swapControls();

    int lower;
    int upper;
    int lowerPos;
    int upperPos;
    int offset;
    int position;
    QxtSpanSlider::SpanHandle lastPressed;
    QxtSpanSlider::SpanHandle mainControl;
    QStyle::SubControl lowerPressed;
    QStyle::SubControl upperPressed;
    QxtSpanSlider::HandleMovementMode movement;
    bool firstMovement;
    bool blockTracking;

public Q_SLOTS:
    void updateRange(int min, int max);
    void movePressedHandle();
private:
    friend class QxtSpanSlider;
    QxtSpanSlider* pvt;
};

#endif // QXTSPANSLIDER_H
