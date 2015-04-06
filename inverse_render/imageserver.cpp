#include "imageserver.h"
#include "imageio.h"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <boost/interprocess/shared_memory_object.hpp>

using namespace std;

// --------------------------------------------------------------
// Constructors and initialization
// --------------------------------------------------------------
using namespace boost::interprocess;
ImageServer::ImageServer(const string& camfile, bool flipx, bool flipy)
    : ImageManager(camfile), flip_x(flipx), flip_y(flipy)
{
    readCameraFile(camfile);

    defaultinit(camfile);
    shared_memory_object::remove(shmname.c_str());
    initializeSharedMemory();
    loadAllFiles();
}

ImageServer::~ImageServer() {
    shared_memory_object::remove(shmname.c_str());
}

// --------------------------------------------------------------
// Camera File Wrangling
// --------------------------------------------------------------
bool ImageServer::readCameraFile(const string& filename)
{
    // Format:
    //    FrameCount Width Height vfov [transformfilename]
    //    filename1 x y z Up_x Up_y Up_z Towards_x Towards_y Towards_z
    //    filename2 x y z Up_x Up_y Up_z Towards_x Towards_y Towards_z
    //    filename3 x y z Up_x Up_y Up_z Towards_x Towards_y Towards_z
    //    ...
    //    Note: angles in degrees
    try {
        ifstream in(filename.c_str());
        double hfov, vfov;
        string infoline;
        getline(in, infoline);
        size_t numparams = count(infoline.begin(), infoline.end(), ' ') + 1;
        stringstream infoin(infoline);
        if (numparams < 4) {
            cerr << "Error! Invalid camera file!" << endl;
            return false;
        }
        infoin >> sz >> w >> h >> vfov;
        double foc = h/(2*tan(vfov*M_PI/360));
        double a,b,c;
        string s;
        CameraParams* curr = cameras;
        for (int i = 0; i < sz; ++i, ++curr) {
            in >> s >> a >> b >> c;
            filenames.push_back(s);
            curr->pos.Reset(a,b,c);
            in >> a >> b >> c;
            curr->up.Reset(a,b,c);
            in >> a >> b >> c;
            curr->towards.Reset(a,b,c);
            curr->right = curr->towards%curr->up;
            curr->width = w;
            curr->height = h;
            curr->focal_length = foc;
            curr->fov = vfov;
        }
        if (numparams > 4) {
            string camxform;
            infoin >> camxform;
            ifstream xformin(camxform.c_str());
            double m[16];
            for (int i = 0; i < 16; ++i) xformin >> m[i];
            *depth2rgb = R4Matrix(m);
            transformAllCameras(*depth2rgb);
        } else {
            *depth2rgb = R4identity_matrix;
        }
    } catch (...) {
        return false;
    }
    return true;
}

void ImageServer::transformAllCameras(const R4Matrix& m) {
    CameraParams* cam = cameras;
    for (int i = 0; i < sz; ++i, ++cam) {
        cam->pos = m*cam->pos;
        cam->up = m*cam->up;
        cam->towards = m*cam->towards;
        cam->right = m*cam->right;
    }
}
// --------------------------------------------------------------
// Utilities
// --------------------------------------------------------------
inline bool fileexists(const string& filename) {
    struct stat buffer;
    return (stat (filename.c_str(), &buffer) == 0);
}
string replaceExtension(const string& s, string newext) {
    size_t i = s.find_last_of(".");
    if (i == string::npos) return s + newext;
    else return s.substr(0,i+1) + newext;
}
// --------------------------------------------------------------
// File load
// --------------------------------------------------------------
bool ImageServer::loadAllFiles() {
    for (int n = 0; n < imagetypes.size(); ++n) {
        for (int i = 0; i < sz; ++i) {
            string f = filenames[i];
            if (imagetypes[n].getExtension() != "") {
                f = replaceExtension(f, imagetypes[n].getExtension());
            }
            if (fileexists(f)) {
                void* im = images[n][i];
                bool success = false;

                if (imagetypes[n].getFileFormat() == ImageType::IT_DEPTHMAP) {
                    success = ImageIO::readDepthMap(f, im, w, h);
                } else if (imagetypes[n].getFileFormat() == ImageType::IT_SCALAR) {
                    success = ImageIO::readScalarMap(f, im, w, h, imagetypes[n].getSize());
                } else {
                    if (imagetypes[n].getType() == CV_32FC3) {
                        success = ImageIO::readFloatImage(f, im, w, h);
                    } else if (imagetypes[n].getType() == CV_32FC1) {
                        success = ImageIO::readFloatImage(f, im, w, h, 1);
                    } else if (imagetypes[n].getType() == CV_8UC3) {
                        success = ImageIO::readRGBImage(f, im, w, h);
                    } else if (imagetypes[n].getType() == CV_8UC1) {
                        success = ImageIO::readRGBImage(f, im, w, h, 1);
                    }
                }

                if (success) {
                    flip((char*) im, w, h, imagetypes[n].getSize());
                    flags[n*sz+i] |= DF_INITIALIZED;
                }
            }
        }
    }
}
void ImageServer::flip(char* a, int w, int h, size_t bytes) {
    char* tmp = new char[bytes];
    if (flip_x) {
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w/2; ++j) {
                memcpy(tmp, a+(i*w+j)*bytes, bytes);
                memcpy(a+(i*w+j)*bytes, a+(i*w+w-j-1)*bytes, bytes);
                memcpy(a+(i*w+w-j-1)*bytes, tmp, bytes);
            }
        }
    }
    delete tmp;
    tmp = new char[bytes*w];
    if (flip_y) {
        for (int i = 0; i < h/2; ++i) {
            memcpy(tmp, a+i*w*bytes, bytes*w);
            memcpy(a+i*w*bytes, a+(h-i-1)*w*bytes, bytes*w);
            memcpy(a+(h-i-1)*w*bytes, tmp, bytes*w);
        }
    }
    delete tmp;
}
