#ifndef _IMAGE_MANAGER_H
#define _IMAGE_MANAGER_H

#include <string>
#include <vector>
#include <opencv2/core/types_c.h>

#include <boost/interprocess/sync/interprocess_upgradable_mutex.hpp>

#include "camparams.h"

/*
 * Class describing image data that exists for each camera,
 * for example RGB images, depth data, confidence values.
 * It holds an identifier used to refer to the image type
 * and the bytes per-pixel of the image.
 */
class ImageType {
    public:
        ImageType(const std::string& name, const std::string& extension, int sizetype, int fileformat=IT_IMAGE)
            : n(name), ext(extension), s(sizetype), img(fileformat){;}
        const std::string& getName() const { return n; }
        const std::string& getExtension() const { return ext; }
        int getSize() const {
            int cn = 1+(s >> CV_CN_SHIFT);
            int sz = 1 << (CV_MAT_DEPTH(s) >> 1);
            return cn*sz;
        }
        int getType() const { return s; }
        int getFileFormat() const { return img; }
        enum {
            IT_IMAGE,
            IT_SCALAR,
            IT_DEPTHMAP,
        };
    private:
        std::string n;
        std::string ext;
        int s;
        int img;
};

class ImageManager {
    public:
        typedef boost::interprocess::interprocess_upgradable_mutex shmutex;

        ImageManager(int width, int height, int numImages);
        ImageManager(const std::string& camfile);

        const CameraParams* getCamera(int n) const;
        unsigned char getFlags(const std::string& type, int n) const;
        void setFlags(const std::string& type, int n, unsigned char value);
        const void* getImage(const std::string& type, int n) const;
        const void* getImage(int n) const;
        void* getImageWriteable(const std::string& type, int n);

        int width() const { return w; }
        int height() const { return h; }
        int size() const { return sz; }

        enum {
           DF_NONE=0,
           DF_INITIALIZED=1,
           DF_ERROR=128,
        };
    protected:
        void initializeImageTypes();

        void defaultinit(const std::string& camfile);
        bool initializeSharedMemory();

        int computeSize() const;
        int nameToIndex(const std::string& type) const;
        shmutex* getMutex(int t, int n);

        std::vector<ImageType> imagetypes;

        int w, h, sz;
        std::vector<std::vector<void*> > images;

        CameraParams* cameras;
        unsigned char* flags;
        shmutex* mutexes;
        std::string shmname;
};
#endif
