#ifndef _INVRENDER_APP_H
#define _INVRENDER_APP_H
#include "datamanager/imagemanager.h"
#include "datamanager/meshmanager.h"
#include "roommodel/roommodel.h"

class InvrenderApp {
    public:
        InvrenderApp() :
            mmgr(NULL), imgr(NULL), room(NULL),
            emit_progress(false), noshm(false) {;}
        ~InvrenderApp() { if (room) delete room; }
        int parseargs(int argc, char** argv);
        virtual int run() {;}

    protected:
        virtual void printargs() {;}
        virtual int _parseargs(int argc, char** argv) { return 1; }

        void emitDone();
        void progressfn(int percent, int doneparts, int totalparts);
        boost::function<void(int)> getProgressFunction(int partnum, int totalparts);
        MeshManager* mmgr;
        ImageManager* imgr;
        roommodel::RoomModel* room;

        bool emit_progress;
        bool noshm;
};

#endif
