#ifndef _IMAGE_MANAGER_H
#define _IMAGE_MANAGER_H

#include "R3Shapes/R3Shapes.h"
#include <string>
#include <vector>

struct CameraParams {
    R3Point pos;
    R3Vector up;
    R3Vector towards;
    R3Vector right;

    int width;
    int height;
    double focal_length;
    double fov;
};

/*
 * Class describing image data that exists for each camera,
 * for example RGB images, depth data, confidence values.
 * It holds an identifier used to refer to the image type
 * and the bytes per-pixel of the image.
 */
class ImageType {
    public:
        ImageType(std::string name, int size)
            : n(name), s(size) {;}
        const std::string getName() const { return n; }
        int getSize() const { return s; }
    private:
        std::string n;
        int s;
};

class ImageManager {
    public:
        ImageManager(int width, int height, int numImages);

        const void* getImage(const std::string& type, int n) const;
        const CameraParams* getCamera(int n) const;
        void setImage(const std::string& type, int n, void* ptr);

        int width() const { return w; }
        int height() const { return h; }
        int size() const { return sz; }
    private:
        bool initializeSharedMemory();
        void initializeImageTypes();
        int computeSize() const;
        int nameToIndex(const std::string& type) const;

        std::vector<ImageType> imagetypes;
        std::vector<std::vector<void*> > images;
        std::vector<CameraParams*> cameras;
        int w, h, sz;
        // FIXME: Locks?
        // FIXME: initialized flags?
};
#endif
