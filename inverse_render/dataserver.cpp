#include "imageserver.h"
#include "meshserver.h"
#include <unistd.h>
#include <string>
#include <cstdio>
#include <iostream>
#include <pcl/console/parse.h>
#include <csignal>

using namespace std;
bool ccw = false;
bool image_flip_x = false;
bool image_flip_y = false;
bool emit_progress = false;

// TODO: Configure imagetypes to load
// TODO: Load labels/types/samples from file

string camfile, meshfile;

bool parseargs(int argc, char** argv) {
    if (argc < 3) {
        cout <<
         "Usage: dataserver -meshfile mesh.ply -camfile camera.cam [args] &\n" \
         "   Arguments:\n" \
         "       -ccw: Face vertices in counterclockwise direction, i.e.\n" \
         "             flip normals (default off, i.e. clockwise)\n" \
         "       -flip_x: Mirror the input images horizontally\n"\
         "       -flip_y: Mirror the input images vertically\n"\
         "       -p: Emit progress in percent\n" << endl;
        return false;
    }
    if (pcl::console::find_switch(argc, argv, "-ccw")) ccw = true;
    if (pcl::console::find_switch(argc, argv, "-flip_x")) image_flip_x = true;
    if (pcl::console::find_switch(argc, argv, "-flip_y")) image_flip_x = true;
    if (pcl::console::find_switch(argc, argv, "-p")) emit_progress = true;

    if (pcl::console::find_argument(argc, argv, "-meshfile") >= 0) {
        pcl::console::parse_argument(argc, argv, "-meshfile", meshfile);
    }
    if (pcl::console::find_argument(argc, argv, "-camfile") >= 0) {
        pcl::console::parse_argument(argc, argv, "-camfile", camfile);
    }
    if (meshfile.empty() && camfile.empty()) return false;
    return true;
}

void progress(int n) {
    if (emit_progress) cout << ">" << n << endl;
}
void progress1(int n) {
    if (emit_progress) cout << ">" << (n/2) << endl;
}
void progress2(int n) {
    if (emit_progress) cout << ">" << (50+n/2) << endl;
}
ImageServer* imgr;
MeshServer* mmgr;
bool run = true;

void cleanup(int n) {
    run = false;
}

int main(int argc, char* argv[]) {
    if (!parseargs(argc, argv)) return 1;
    signal(SIGQUIT, &cleanup);
    signal(SIGINT, &cleanup);
    signal(SIGTERM, &cleanup);

    if (!camfile.empty()) {
        cout << "Loading images..." << endl;
        imgr = new ImageServer(camfile, image_flip_x, image_flip_y, meshfile.empty()?&progress:&progress1);
        cout << "Done loading images." << endl;
    }
    if (!meshfile.empty()) {
        cout << "Loading geometry..." << endl;
        mmgr = new MeshServer(meshfile, ccw, camfile.empty()?&progress:&progress2);
        cout << "Done loading geometry." << endl;
    }
    progress(100);
    while(run) {
        sleep(1);
    }
    cout << endl << "Exiting..." << endl;
    delete imgr;
    delete mmgr;
    return 0;
}
