#include "fileimageserver.h"
#include "imageio.h"
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

inline bool fileexists(const std::string& filename) {
    struct stat buffer;
    return (stat (filename.c_str(), &buffer) == 0);
}

FileImageServer::FileImageServer(const string& camerafile, bool flipx, bool flipy, cb_type cb)
{
    currentidx = -1;
    flip_x = flipx;
    flip_y = flipy;
    progress_cb = cb;
    camfile = camerafile;

    initserver();
}

bool FileImageServer::initializeSharedMemory() {
    cameras = new CameraParams[sz];
    depth2rgb = new R4Matrix;
    flags = new unsigned char[sz*imagetypes.size()];
    memset(flags, 0, sz*imagetypes.size());
    mutexes = new shmutex[imagetypes.size()];
    currimages.resize(imagetypes.size());
    for (int i = 0; i < imagetypes.size(); ++i) {
        currimages[i] = new char[w*h*imagetypes[i].getSize()];
    }
}

FileImageServer::~FileImageServer() {
    if (cameras) delete [] cameras;
    if (depth2rgb) delete depth2rgb;
    if (flags) delete [] flags;
    for (int i = 0; i < currimages.size(); i++) {
        delete [] currimages[i];
    }
}

const void* FileImageServer::getImage(int i, int n) {
    if (!loaded(n)) loadImages(n);
    if (i < 0) return NULL;
    return currimages[i];
}

void* FileImageServer::getImageWriteable(const std::string& type, int n) {
    if (!loaded(n)) loadImages(n);
    int i = nameToIndex(type);
    if (i < 0) return NULL;
    return currimages[i];
}

bool FileImageServer::loadImages(int i) {
    int w, h;
    for (int n = 0; n < imagetypes.size(); ++n) {
        std::string f = filenames[i];
        if (imagetypes[n].getExtension() != "") {
            f = ImageIO::replaceExtension(f, imagetypes[n].getExtension());
        }
        if (fileexists(f)) {
            void* im = currimages[n];
            bool success = false;

            if (imagetypes[n].getFileFormat() == ImageType::IT_DEPTHMAP) {
                success = ImageIO::readDepthMap(f, im, w, h);
            } else if (imagetypes[n].getFileFormat() == ImageType::IT_SCALAR) {
                success = ImageIO::readScalarMap(f, im, w, h, imagetypes[n].getSize());
            } else {
                if (imagetypes[n].getType() == CV_32FC3) {
                    success = ImageIO::readFloatImage(f, im, w, h);
                    if (imagetypes[n].getFlags() & ImageType::IT_APPLYCORRECTION) {
                        float* p = (float*) im;
                        for (int j = 0; j < w*h; j++) {
                            for (int k = 0; k < 3; k++) {
                                *p = pow(*p, getCamera(i)->gamma)*getCamera(i)->exposure[k];
                                p++;
                            }
                        }
                    }
                } else if (imagetypes[n].getType() == CV_32FC1) {
                    success = ImageIO::readFloatImage(f, im, w, h, 1);
                } else if (imagetypes[n].getType() == CV_8UC3) {
                    success = ImageIO::readRGBImage(f, im, w, h);
                } else if (imagetypes[n].getType() == CV_8UC1) {
                    success = ImageIO::readRGBImage(f, im, w, h, 1);
                }
            }

            if (success) {
                if (!(imagetypes[n].getFlags() & ImageType::IT_NOFLIP)) {
                    ImageIO::flip((char*) im, w, h, imagetypes[n].getSize(), flip_x, flip_y);
                }
                flags[n*sz+i] |= DF_INITIALIZED;
            }
        }
    }
    currentidx = i;
}
