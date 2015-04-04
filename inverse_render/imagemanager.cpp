#include "imagemanager.h"

#define SHM_IMAGEDATA_ID "SHM_IMAGEDATA_ID"

// Sets up what types of image
void ImageManager::initializeImageTypes() {
    // Raw input data
    imagetypes.push_back(ImageType("color",      sizeof(float)*3));
    imagetypes.push_back(ImageType("confidence", sizeof(float)));
    imagetypes.push_back(ImageType("depth",      sizeof(float)));
    // Processed results
    imagetypes.push_back(ImageType("edges",      sizeof(char)*3));
    imagetypes.push_back(ImageType("labels",     sizeof(char)));
}

int ImageManager::computeSize() {
    int sum = 0;
    for (int i = 0; i < imagetypes.size(); ++i) {
        sum += imagetypes[i].getSize();
    }
    return sz*(w*h*sum + sizeof(CameraParams));
}

int ImageManager::nameToIndex(const string& type) {
    for (int i = 0; i < imagetypes.size(); ++i) {
        if (imagetypes[i].getName() == type) return i;
    }
    return -1;
}
const void* ImageManager::getImage(const string& type, int n) {
    int i = nameToIndex(type);
    if (i < 0) return NULL;
    return images[i][n];
}

void ImageManager::setImage(const string& type, int n, void* ptr) {
    int i = nameToIndex(type);
    if (i < 0) return NULL;
    // FIXME: Lock image
    memcpy(images[i][n], ptr, w*h*imagetypes[i].getSize());
}

const CameraParams* ImageManager::getCamera(int n) {
    return cameras[i];
}

ImageManager::ImageManager(int width, int height, int numImages)
    : w(width), h(height), sz(numImages)
{
    initializeImageTypes();
    initializeSharedMemory();
}

bool ImageManager::initializeSharedMemory() {
    // FIXME
}
