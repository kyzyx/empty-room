#ifndef SUBPROCESSWORKER_H
#define SUBPROCESSWORKER_H

#include <QObject>
#include <cstdio>

class SubprocessWorker : public QObject
{
    Q_OBJECT
public:
    explicit SubprocessWorker(QObject *parent = 0);
    SubprocessWorker(QObject *parent, QString cmd);
protected:
    QString command;
    pid_t pid;
signals:
    void percentChanged(int percent);
    void done();
    void error(QString errstring);

public slots:
    void run();
    void terminate();
};

#endif // SUBPROCESSWORKER_H
