#include "datamanager/imageserver.h"
#include "datamanager/meshserver.h"
#include <unistd.h>
#include <string>
#include <cstdio>
#include <iostream>
#include <pcl/console/parse.h>
#include <csignal>
#include <boost/bind.hpp>

using namespace std;
bool ccw = false;
bool image_flip_x = false;
bool image_flip_y = false;
bool emit_progress = false;

bool savelabelimages = false;
bool saveedgeimages = false;
string savereprojectfile, savelabelfile;

// TODO: Configure imagetypes to load

string camfile, meshfile, samplesfile, datafile;
bool data_only = false;

int sampleflags = MeshManager::READ_SAMPLES_ONLY;
int labelflags = MeshManager::READ_ALL_CHANNELS;

int parseargs(int argc, char** argv) {
    if (argc < 3) {
        cout <<
         "Usage: dataserver -meshfile mesh.ply -camfile camera.cam [args] &\n" \
         "   Arguments:\n" \
         "       -labelfile f: Load mesh per-vertex data from file f\n" \
         "       -samplesfile f: Load mesh per-vertex color samples from file f\n" \
         "       -ccw: Face vertices in counterclockwise direction, i.e.\n" \
         "             flip normals (default off, i.e. clockwise)\n" \
         "       -flip_x: Mirror the input images horizontally\n"\
         "       -flip_y: Mirror the input images vertically\n"\
         "       -data_only: load per-vertex data and exit; don't persist as\n"\
         "                   server\n"\
         "       -[no]load_labels: With samplesfile or labelfile, [don't] load \n"\
         "                         the per-vertex label field (channel 0)\n"\
         "       -[no]load_types: With samplesfile or labelfile, [don't] load \n"\
         "                        the per-vertex types field  (channel 1)\n"\
         "       -[no]load_data: With samplesfile or labelfile, [don't] load\n"\
         "                       the per-vertex data field (channel 2)\n"\
         "       -save_label_images: Save generated label images\n"\
         "       -save_edge_images: Save generated edge images\n"\
         "       -save_labels f: Save per-vertex labels to file f\n"\
         "       -save_samples f: Save per-vertex color samples to file f\n"\
         "       -p: Emit progress in percent\n" << endl;
        return false;
    }
    int numfilestoload = 0;
    if (pcl::console::find_switch(argc, argv, "-ccw")) ccw = true;
    if (pcl::console::find_switch(argc, argv, "-flip_x")) image_flip_x = true;
    if (pcl::console::find_switch(argc, argv, "-flip_y")) image_flip_y = true;
    if (pcl::console::find_switch(argc, argv, "-save_label_images")) savelabelimages = true;
    if (pcl::console::find_switch(argc, argv, "-save_edge_images")) saveedgeimages = true;
    if (pcl::console::find_switch(argc, argv, "-p")) emit_progress = true;

    if (pcl::console::find_switch(argc, argv, "-load_labels")) {
        sampleflags += MeshManager::READ_LABEL_CHANNEL;
    }
    if (pcl::console::find_switch(argc, argv, "-load_types")) {
        sampleflags += MeshManager::READ_TYPE_CHANNEL;
    }
    if (pcl::console::find_switch(argc, argv, "-load_data")) {
        sampleflags += MeshManager::READ_DATA_CHANNEL;
    }
    if (pcl::console::find_switch(argc, argv, "-noload_labels")) {
        labelflags -= MeshManager::READ_LABEL_CHANNEL;
    }
    if (pcl::console::find_switch(argc, argv, "-noload_types")) {
        labelflags -= MeshManager::READ_TYPE_CHANNEL;
    }
    if (pcl::console::find_switch(argc, argv, "-noload_data")) {
        labelflags -= MeshManager::READ_DATA_CHANNEL;
    }
    if (pcl::console::find_switch(argc, argv, "-data_only")) {
        data_only = true;
        --numfilestoload;
    }

    if (pcl::console::find_argument(argc, argv, "-meshfile") >= 0) {
        pcl::console::parse_argument(argc, argv, "-meshfile", meshfile);
        ++numfilestoload;
    }
    if (pcl::console::find_argument(argc, argv, "-camfile") >= 0) {
        pcl::console::parse_argument(argc, argv, "-camfile", camfile);
        ++numfilestoload;
    }
    if (pcl::console::find_argument(argc, argv, "-labelfile") >= 0) {
        pcl::console::parse_argument(argc, argv, "-labelfile", datafile);
        ++numfilestoload;
    }
    if (pcl::console::find_argument(argc, argv, "-samplesfile") >= 0) {
        pcl::console::parse_argument(argc, argv, "-samplesfile", samplesfile);
        ++numfilestoload;
    }
    if (pcl::console::find_argument(argc, argv, "-save_labels") >= 0) {
        pcl::console::parse_argument(argc, argv, "-save_labels", savelabelfile);
    }
    if (pcl::console::find_argument(argc, argv, "-save_samples") >= 0) {
        pcl::console::parse_argument(argc, argv, "-save_samples", savereprojectfile);
    }
    if (meshfile.empty() && (!datafile.empty() || !samplesfile.empty())) {
        cout << "Error: loading per-vertex data requires a meshfile" << endl;
        return 0;
    }
    return numfilestoload;
}

void progressfn(int n, int curr, int total) {
    if (emit_progress) cout << ">" << (100*curr+n)/total << endl;
}
ImageManager* imgr;
MeshManager* mmgr;
bool run = true;

void cleanup(int n) {
    run = false;
}

int main(int argc, char* argv[]) {
    int nload = parseargs(argc, argv);
    signal(SIGQUIT, &cleanup);
    signal(SIGINT, &cleanup);
    signal(SIGTERM, &cleanup);

    int nloaded = 0;
    if (!data_only && !camfile.empty()) {
        cout << "Loading images..." << endl;
        imgr = new ImageServer(camfile, image_flip_x, image_flip_y, boost::bind(progressfn, _1, nloaded, nload));
        cout << "Done loading images." << endl;
        nloaded++;
    }
    if (!data_only && !meshfile.empty()) {
        cout << "Loading geometry..." << endl;
        mmgr = new MeshServer(meshfile, ccw, boost::bind(progressfn, _1, nloaded, nload));
        cout << "Done loading geometry." << endl;
        nloaded++;
    }
    if (!samplesfile.empty()) {
        cout << "Loading per-vertex color sample data..." << endl;
        if (!mmgr) mmgr = new MeshManager(meshfile);
        mmgr->readSamplesFromFile(samplesfile, sampleflags, boost::bind(progressfn, _1, nloaded, nload));
        cout << "Done loading per-vertex color sample data." << endl;
        nloaded++;
    }
    if (!datafile.empty()) {
        cout << "Loading per-vertex data..." << endl;
        if (!mmgr) mmgr = new MeshManager(meshfile);
        mmgr->readLabelsFromFile(datafile, labelflags, boost::bind(progressfn, _1, nloaded, nload));
        cout << "Done loading per-vertex data." << endl;
        nloaded++;
    }
    if (nload > 0) {
        cout << ">100" << endl;
        cout << ">>done" << endl;
    }
    if (!data_only) {
        while(run) {
            sleep(1);
        }
        cout << endl << "Exiting..." << endl;
    }
    int nsave = 0;
    int nsaved = 0;
    if (!savereprojectfile.empty()) ++nsave;
    if (!savelabelfile.empty()) ++nsave;
    if (saveedgeimages) ++nsave;
    if (savelabelimages) ++nsave;

    if (!savereprojectfile.empty()) {
        if (!mmgr) mmgr = new MeshManager(meshfile);
        mmgr->loadSamples();
        mmgr->writeSamplesToFile(savereprojectfile, boost::bind(progressfn, _1, nsaved, nsave));
        nsaved++;
    }
    if (!savelabelfile.empty()) {
        if (!mmgr) mmgr = new MeshManager(meshfile);
        mmgr->writeLabelsToFile(savelabelfile, boost::bind(progressfn, _1, nsaved, nsave));
        nsaved++;
    }
    if (savelabelimages) {
        if (!imgr) imgr = new ImageManager(camfile);
        const char* type = "labels";
        for (int i = 0; i < imgr->size(); ++i) {
            progressfn(100*i/imgr->size(), nsaved, nsave);
            if (imgr->getFlags(type, i) & ImageManager::DF_INITIALIZED)
                imgr->saveImage(type, i);
        }
        progressfn(100, nsaved, nsave);
        nsaved++;
    }
    if (saveedgeimages) {
        if (!imgr) imgr = new ImageManager(camfile);
        const char* type = "edges";
        for (int i = 0; i < imgr->size(); ++i) {
            progressfn(100*i/imgr->size(), nsaved, nsave);
            if (imgr->getFlags(type, i) & ImageManager::DF_INITIALIZED)
                imgr->saveImage(type, i);
        }
        progressfn(100, nsaved, nsave);
        nsaved++;
    }
    if (nsave) {
        cout << ">100" << endl;
        cout << ">>done" << endl;
    }
    if (imgr) delete imgr;
    if (mmgr) delete mmgr;
    return 0;
}
