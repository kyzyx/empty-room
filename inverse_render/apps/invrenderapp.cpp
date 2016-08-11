#include "invrenderapp.h"
#include "datamanager/fileimageserver.h"

#include <unistd.h>
#include <cstdio>
#include <csignal>

#include <string>
#include <iostream>
#include <pcl/console/parse.h>
#include <boost/bind.hpp>

using namespace std;

int InvrenderApp::parseargs(int argc, char** argv) {
    if (argc < 3) {
        cout <<
         "Usage: " << argv[0] <<
                      " -meshfile mesh.ply -camfile camera.cam -roommodel room.json [args]\n" \
         "   Arguments:\n" \
         "       -p: Emit progress in percent\n" \
         "   Camfile Arguments:\n" \
         "       -noshm: Load images lazily from disk\n" \
         "       -flip_x: Mirror the input images horizontally (with noshm)\n"\
         "       -flip_y: Mirror the input images vertically (with noshm)\n" << endl;
        printargs();
        return false;
    }
    string meshfile, camfile, roomfile;
    bool image_flip_x = false;
    bool image_flip_y = false;
    if (pcl::console::find_switch(argc, argv, "-p")) emit_progress = true;
    if (pcl::console::find_switch(argc, argv, "-noshm")) noshm = true;
    if (pcl::console::find_switch(argc, argv, "-flip_x")) image_flip_x = true;
    if (pcl::console::find_switch(argc, argv, "-flip_y")) image_flip_y = true;
    if (pcl::console::find_argument(argc, argv, "-meshfile") >= 0) {
        pcl::console::parse_argument(argc, argv, "-meshfile", meshfile);
        mmgr = new MeshManager(meshfile);
    }
    if (pcl::console::find_argument(argc, argv, "-roommodel") >= 0) {
        pcl::console::parse_argument(argc, argv, "-roommodel", roomfile);
        room = new roommodel::RoomModel(roomfile);
    }
    if (pcl::console::find_argument(argc, argv, "-camfile") >= 0) {
        pcl::console::parse_argument(argc, argv, "-camfile", camfile);
        if (noshm) {
            imgr = new FileImageServer(camfile, image_flip_x, image_flip_y);
        }
        else {
            imgr = new ImageManager(camfile);
        }
    }
    return _parseargs(argc, argv);
}

void InvrenderApp::emitDone() {
    if (emit_progress) {
        cout << ">100" << endl;
        cout << ">>done" << endl;
    }
}
void InvrenderApp::progressfn(int percent, int doneparts, int totalparts) {
    if (emit_progress) cout << ">" << (100*doneparts+percent)/totalparts << endl;
}

boost::function<void(int)> InvrenderApp::getProgressFunction(int partnum, int totalparts) {
    return boost::bind(&InvrenderApp::progressfn, this, _1, partnum, totalparts);
}
