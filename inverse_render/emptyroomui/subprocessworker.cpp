#include "subprocessworker.h"
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <csignal>
#include <errno.h>
#include <sys/wait.h>

SubprocessWorker::SubprocessWorker(QObject *parent) :
    QObject(parent)
{
}

SubprocessWorker::SubprocessWorker(QObject *parent, QString cmd) :
    QObject(parent), command(cmd)
{
}


pid_t popen2(const char *command, int *outfp)
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

        execl("/bin/sh", "sh", "-c", command, NULL);
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

const int SZ = 256;
void SubprocessWorker::run() {
    int output;
    //qDebug(command.toStdString().c_str());
    pid_t pid = popen2(command.toStdString().c_str(), &output);
    FILE* fp = fdopen(output, "r");
    char buf[SZ];
    while (fgets(buf, SZ, fp) != NULL) {
        //qDebug(buf);
        if (buf[0] == '>') {
            int n = atoi(buf+1);
            emit percentChanged(n);
            if (n == 100) {
                emit done();
                break;
            }
        }
    }
    fclose(fp);
    pclose2(pid);
}
