#ifndef SUBPROCESSWORKER_H
#define SUBPROCESSWORKER_H

#include <QObject>

class SubprocessWorker : public QObject
{
    Q_OBJECT
public:
    explicit SubprocessWorker(QObject *parent = 0);
    SubprocessWorker(QObject *parent, QString cmd);
protected:
    QString command;
signals:
    void percentChanged(int percent);
    void done();

public slots:
    void run();
};

#endif // SUBPROCESSWORKER_H
