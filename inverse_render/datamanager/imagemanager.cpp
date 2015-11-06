#include "imagemanager.h"
#include "imageio.h"
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/interprocess/sync/upgradable_lock.hpp>
#include <boost/filesystem.hpp>

#include <functional>
#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

#define SHM_IMAGEDATA_ID "SHM_IMAGEDATA_ID"
// TODO: Read imagetypes from config
// TODO: Proper locking mechanism

/*
 * Memory layout:
 * All mutexes
 * All initialization flags
 * Depth2RGB, if any
 * All CameraParams
 * All images for imagetypes[0]
 * All images for imagetypes[1]
 * All images for imagetypes[2]
 * ...
 */

// --------------------------------------------------------------
// Constructors and initialization
// --------------------------------------------------------------
ImageManager::ImageManager(int width, int height, int numImages)
    : w(width), h(height), sz(numImages)
{
    defaultinit("");
    initializeSharedMemory();
}
ImageManager::ImageManager(const string& camfile)
{
    ifstream in(camfile.c_str());
    in >> sz >> w >> h;
    in.close();

    defaultinit(camfile);
    initializeSharedMemory();
    readCameraFile(camfile);
}

void ImageManager::defaultinit(const string& camfile) {
    initializeImageTypes();

    string fullpath = boost::filesystem::canonical(boost::filesystem::path(camfile)).string();
    hash<string> str_hash;
    stringstream ss;
    ss << setbase(16) << SHM_IMAGEDATA_ID << str_hash(fullpath);
    shmname = ss.str();
}

// Sets up what types of image
void ImageManager::initializeImageTypes() {
    // Raw input data
    imagetypes.push_back(ImageType("color",      "",     CV_32FC3, ImageType::IT_APPLYEXPOSURE));
    // Custom types begin here
    imagetypes.push_back(ImageType("confidence", "conf", CV_32FC1, 0, ImageType::IT_SCALAR));
    imagetypes.push_back(ImageType("depth",      "pcd",  CV_32FC1, 0, ImageType::IT_DEPTHMAP));
    // Processed results
    imagetypes.push_back(ImageType("edges",      "edges.exr", CV_32FC3, ImageType::IT_COMPUTED));
    imagetypes.push_back(ImageType("labels",     "lbl.png",  CV_8UC1, ImageType::IT_COMPUTED));

    images.resize(imagetypes.size());
}

using namespace boost::interprocess;
bool ImageManager::initializeSharedMemory() {
    try {
        shared_memory_object shm(open_or_create, shmname.c_str(), read_write);
        shm.truncate(computeSize());
        mregion = mapped_region(shm, read_write);
    } catch(...) {
        return false;
    }

    char* s = static_cast<char*>(mregion.get_address());
    mutexes = (shmutex*)(s);
    s += imagetypes.size()*sizeof(shmutex);
    flags = (unsigned char*)(s);
    s += sz*imagetypes.size();
    depth2rgb = (R4Matrix*)(s);
    s += sizeof(R4Matrix);
    cameras = (CameraParams*)(s);
    s += sz*sizeof(CameraParams);
    for (int i = 0; i < imagetypes.size(); ++i) {
        images[i].resize(sz);
        for (int j = 0; j < sz; ++j) {
            images[i][j] = s;
            s += w*h*imagetypes[i].getSize();
        }
    }
    return true;
}

typedef boost::filesystem::path bfpath;
bool ImageManager::readCameraFile(const std::string& filename)
{
    try {
        ifstream in(filename.c_str());
        string infoline;
        getline(in, infoline);
        size_t numparams = count(infoline.begin(), infoline.end(), ' ') + 1;
        if (numparams < 4) {
            cerr << "Error! Invalid camera file!" << endl;
            return false;
        }
        double a;
        string s;
        for (int i = 0; i < sz; ++i) {
            in >> s;
            for (int j = 0; j < 9; ++j) in >> a;
            bfpath filepath = bfpath(filename).parent_path();
            filepath /= bfpath(s);
            string fullpath = boost::filesystem::canonical(filepath).string();
            filenames.push_back(fullpath);
        }
    } catch (...) {
        return false;
    }
    return true;
}

void ImageManager::saveImage(const string& type, int n) {
    int i = nameToIndex(type);
    if (i < 0) return;
    saveImage(i,n);
}

void ImageManager::saveImage(int i, int n) {
    saveImage(i, n, false, false);
}
void ImageManager::saveImage(int i, int n, bool flip_x, bool flip_y) {
    string f = filenames[n];
    if (imagetypes[i].getExtension() != "") {
        f = ImageIO::replaceExtension(f, imagetypes[i].getExtension());
    }
    if (imagetypes[i].getFileFormat() == ImageType::IT_DEPTHMAP) {
        // TODO: Unimplemented
    } else if (imagetypes[i].getFileFormat() == ImageType::IT_SCALAR) {
        char* tmp = new char[w*h*imagetypes[i].getSize()];
        memcpy(tmp, getImage(i,n), w*h*imagetypes[i].getSize());
        ImageIO::flip(tmp, w, h, imagetypes[i].getSize(), flip_x, flip_y);
        ImageIO::writeScalarMap(f, tmp, w, h, imagetypes[i].getSize());
        delete tmp;
    } else {
        char* tmp = new char[w*h*imagetypes[i].getSize()];
        memcpy(tmp, getImage(i,n), w*h*imagetypes[i].getSize());
        ImageIO::flip(tmp, w, h, imagetypes[i].getSize(), flip_x, flip_y);
        if (imagetypes[i].getType() == CV_32FC3) {
            ImageIO::writeExrImage(f, (const float*) tmp, w, h);
        } else if (imagetypes[i].getType() == CV_32FC1) {
            ImageIO::writeExrImage(f, (const float*) tmp, w, h, 1);
        } else if (imagetypes[i].getType() == CV_8UC3) {
            ImageIO::writePngImage(f, (const unsigned char*) tmp, w, h);
        } else if (imagetypes[i].getType() == CV_8UC1) {
            ImageIO::writePngImage(f, (const unsigned char*) tmp, w, h, 1);
        }
        delete tmp;
    }

}

// --------------------------------------------------------------
// Utilities
// --------------------------------------------------------------
size_t ImageManager::computeSize() const {
    size_t sum = 0;
    for (int i = 0; i < imagetypes.size(); ++i) {
        sum += imagetypes[i].getSize();
    }
    return imagetypes.size()*sizeof(shmutex)      // Mutexes
         + sz*imagetypes.size()                   // Initialization flags
         + sizeof(R4Matrix)                       // Depth to RGB Transform
         + sz*sizeof(CameraParams)                // Camera parameters
         + sz*w*h*sum;                            // Image data
}

int ImageManager::nameToIndex(const string& type) const {
    for (int i = 0; i < imagetypes.size(); ++i) {
        if (imagetypes[i].getName() == type) return i;
    }
    return -1;
}

ImageManager::shmutex* ImageManager::getMutex(int t, int n) {
    return &mutexes[t];
}

// --------------------------------------------------------------
// Accessors
// --------------------------------------------------------------
const void* ImageManager::getImage(const string& type, int n) {
    int i = nameToIndex(type);
    return getImage(i, n);
}
const void* ImageManager::getImage(int i, int n) {
    if (i < 0) return NULL;
    //boost::interprocess::shareable_lock<shmutex> lock(*getMutex(i,n));
    if (flags[i*sz+n] & DF_INITIALIZED) return images[i][n];
    else return NULL;
}
const void* ImageManager::getImage(int n) {
    return getImage(0, n);
}

void* ImageManager::getImageWriteable(const string& type, int n) {
    int i = nameToIndex(type);
    if (i < 0) return NULL;
    //boost::interprocess::scoped_lock<shmutex> lock(*getMutex(i,n));
    //memcpy(images[i][n], ptr, w*h*imagetypes[i].getSize());
    return images[i][n];
}

const CameraParams* ImageManager::getCamera(int n) const {
    if (n < 0 || n >= sz) return NULL;
    return &cameras[n];
}

unsigned char ImageManager::getFlags(const string& type, int n) const {
    int i = nameToIndex(type);
    if (i < 0) return DF_ERROR;
    if (n < 0 || n >= sz) return DF_ERROR;
    //boost::interprocess::shareable_lock<shmutex> lock(*getMutex(i,n));
    return flags[i*sz + n];
}
void ImageManager::setFlags(const string& type, int n, unsigned char value) {
    int i = nameToIndex(type);
    if (i < 0) return;
    if (n < 0 || n >= sz) return;
    //boost::interprocess::scoped_lock<shmutex> lock(*getMutex(i,n));
    flags[i*sz + n] = value;
}
const R4Matrix& ImageManager::getDepthToRgbTransform() const {
    return *depth2rgb;
}
