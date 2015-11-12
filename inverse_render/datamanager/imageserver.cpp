#include "imageserver.h"
#include "imageio.h"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/filesystem.hpp>

using namespace std;

// --------------------------------------------------------------
// Constructors and initialization
// --------------------------------------------------------------
using namespace boost::interprocess;
ImageServer::ImageServer(const string& camerafile, bool flipx, bool flipy, cb_type cb)
    : flip_x(flipx), flip_y(flipy), progress_cb(cb), camfile(camerafile)
{
    initserver();
}

void ImageServer::initserver() {
    ifstream in(camfile.c_str());
    if (!readCameraFileHeader(in)) return;
    in.close();

    defaultinit(camfile);
    shared_memory_object::remove(shmname.c_str());
    initializeSharedMemory();
    for (int i = 0; i < imagetypes.size(); ++i) {
        new (&(mutexes[i])) shmutex();
    }
    if (progress_cb) progress_cb(2);
    readCameraFile(camfile);
    if (progress_cb) progress_cb(5);
    loadAllFiles();
}

ImageServer::~ImageServer() {
    shared_memory_object::remove(shmname.c_str());
}

// --------------------------------------------------------------
// Camera File Wrangling
// --------------------------------------------------------------
typedef boost::filesystem::path bfpath;
bool ImageServer::readCameraFile(const string& filename)
{
    // Format:
    //    FrameCount Width Height vfov
    //    filename1 x y z Up_x Up_y Up_z Towards_x Towards_y Towards_z [exp_r exp_g exp_b]
    //    filename2 x y z Up_x Up_y Up_z Towards_x Towards_y Towards_z [exp_r exp_g exp_b]
    //    filename3 x y z Up_x Up_y Up_z Towards_x Towards_y Towards_z [exp_r exp_g exp_b]
    //    ...
    //    Note: angles in degrees
    try {
        ifstream in(filename.c_str());
        map<string, string> vars;
        if (!readCameraFileHeader(in, &vars)) return false;
        double hfov, vfov, foc;
        double gamma = 1;
        if (vars.count("yfov")) {
            vfov = atof(vars["yfov"].c_str());
            foc = h/(2*tan(vfov*M_PI/360));
        } else if (vars.count("vfov")) {
            vfov = atof(vars["vfov"].c_str());
            foc = h/(2*tan(vfov*M_PI/360));
        } else if (vars.count("xfov")) {
            hfov = atof(vars["xfov"].c_str());
            foc = w/(2*tan(hfov*M_PI/360));
            vfov = atan(h/(2*foc))*360/M_PI;
        } else if (vars.count("hfov")) {
            hfov = atof(vars["hfov"].c_str());
            foc = w/(2*tan(hfov*M_PI/360));
            vfov = atan(h/(2*foc))*360/M_PI;
        } else {
            cerr << "Missing field of view" << endl;
            return false;
        }
        double a,b,c;
        string s, line;
        CameraParams* curr = cameras;
        for (int i = 0; i < sz; ++i, ++curr) {
            getline(in, line);
            int ntoks = count(line.begin(), line.end(), ' ') + 1;
            bool hasExposures = (ntoks >= 13);
            stringstream imagein(line);
            imagein >> s >> a >> b >> c;
            bfpath filepath = bfpath(filename).parent_path();
            filepath /= bfpath(s);
            string fullpath = boost::filesystem::canonical(filepath).string();
            filenames.push_back(fullpath);
            curr->pos.Reset(a,b,c);
            imagein >> a >> b >> c;
            curr->up.Reset(a,b,c);
            imagein >> a >> b >> c;
            curr->towards.Reset(a,b,c);
            curr->right = curr->towards%curr->up;
            curr->width = w;
            curr->height = h;
            curr->focal_length = foc;
            curr->fov = vfov;
            if (hasExposures) {
                imagein >> a >> b >> c;
                curr->exposure.Reset(a,b,c);
            } else {
                curr->exposure.Reset(1,1,1);
            }
        }
        if (vars.count("camxform")) {
            bfpath filepath = bfpath(filename).parent_path();
            filepath /= bfpath(vars["camxform"]);
            string fullpath = boost::filesystem::canonical(filepath).string();
            ifstream xformin(fullpath);
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
// --------------------------------------------------------------
// File load
// --------------------------------------------------------------
bool ImageServer::loadAllFiles() {
    for (int i = 0; i < sz; ++i) {
        for (int n = 0; n < imagetypes.size(); ++n) {
            string f = filenames[i];
            if (imagetypes[n].getExtension() != "") {
                f = ImageIO::replaceExtension(f, imagetypes[n].getExtension());
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
                        if (imagetypes[n].getFlags() & ImageType::IT_APPLYEXPOSURE) {
                            float* p = (float*) im;
                            for (int j = 0; j < w*h; j++) {
                                for (int k = 0; k < 3; k++) {
                                    *p++ *= getCamera(i)->exposure[k];
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
            if (progress_cb) {
                float prevprop = (i*imagetypes.size()+n-1)/(float) (sz*imagetypes.size());
                float prop = (i*imagetypes.size()+n)/(float) (sz*imagetypes.size());
                //float prevprop = (n*sz+i-1)/(float) (sz*imagetypes.size());
                //float prop = (n*sz+i)/(float) (sz*imagetypes.size());
                int pp = 5+(int)(95*prevprop);
                int p = 5+(int)(95*prop);
                if (p > pp) progress_cb(p);
            }
        }
    }
}
