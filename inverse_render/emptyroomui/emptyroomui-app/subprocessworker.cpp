#include "subprocessworker.h"
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <csignal>
#include <errno.h>
#include <sys/wait.h>
#include <QStringList>

SubprocessWorker::SubprocessWorker(QObject *parent) :
    QObject(parent), pid(0)
{
}

SubprocessWorker::SubprocessWorker(QObject *parent, QString cmd) :
    QObject(parent), command(cmd), pid(0)
{
}


pid_t popen2(char * const command[], int *outfp)
{
    int p_stdout[2];
    pid_t pid;

    if (pipe(p_stdout) != 0)
        return -1;

    pid = fork();

    if (pid < 0)
        return pid;
    else if (pid == 0)
    {
        if (p_stdout[1] != fileno(stdout)) {
            dup2(p_stdout[1], fileno(stdout));
            close(p_stdout[1]);
        }
        close(p_stdout[0]);
        execvp(command[0], command);
        //execl("/bin/sh", "sh", "-c", command, NULL);
        perror("execl");
        exit(1);
    }

    close(p_stdout[1]);
    *outfp = p_stdout[0];
    return pid;
}
int pclose2(pid_t pid) {
    sigset_t nmask, omask;
    union wait pstat;
    sigemptyset(&nmask);
    sigaddset(&nmask, SIGINT);
    sigaddset(&nmask, SIGQUIT);
    sigaddset(&nmask, SIGHUP);
    sigprocmask(SIG_BLOCK, &nmask, &omask);
    register int p;
    do {
        p = waitpid(pid, (int*) &pstat, 0);
    } while (p == -1  && errno == EINTR);
    sigprocmask(SIG_SETMASK, &omask, NULL);
    return p==-1?-1:pstat.w_status;
}

void SubprocessWorker::terminate() {
    if (pid) {
        int r = kill(pid, SIGTERM);
        pid = 0;
    }
}

const int SZ = 256;
void SubprocessWorker::run() {
    int output;
    qDebug(command.toStdString().c_str());
    QStringList args = command.split(" ");
    char** arguments = new char*[args.size()+1];
    for (int i = 0; i < args.size(); ++i) {
        arguments[i] = new char [args[i].length()+1];
        strcpy(arguments[i], args[i].toStdString().c_str());
    }
    arguments[args.size()] = NULL;
    pid = popen2(arguments, &output);
    delete [] arguments;
    FILE* fp = fdopen(output, "r");
    char buf[SZ];
    bool isdone = false;
    while (fgets(buf, SZ, fp) != NULL) {
        qDebug(buf);
        if (buf[0] == '>') {
            if (buf[1] == '>') {
                if (strncmp(buf+2, "done", 4) == 0) {
                    if (!isdone) {
                        emit done();
                        isdone = true;
                    }
                } else if (strncmp(buf+2, "error:", 6) == 0) {
                    emit error(QString(buf+2));
                } else if (strncmp(buf+2, "data:", 5) == 0) {
                    emit data(QString(buf+7));
                }
            } else {
                int n = atoi(buf+1);
                emit percentChanged(n);
            }
        }
    }
    fclose(fp);
    pclose2(pid);
    pid = 0;
}
