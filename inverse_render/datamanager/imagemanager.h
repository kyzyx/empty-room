#ifndef _IMAGE_MANAGER_H
#define _IMAGE_MANAGER_H

#include <string>
#include <vector>
#include <opencv2/core/types_c.h>

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_sharable_mutex.hpp>

#include "camparams.h"

/*
 * Class describing image data that exists for each camera,
 * for example RGB images, depth data, confidence values.
 * It holds an identifier used to refer to the image type
 * and the bytes per-pixel of the image.
 */
class ImageType {
    public:
        ImageType(const std::string& name, const std::string& extension, int sizetype, int flags=0, int fileformat=IT_IMAGE)
            : n(name), ext(extension), s(sizetype), f(flags), img(fileformat){;}
        const std::string& getName() const { return n; }
        const std::string& getExtension() const { return ext; }
        int getSize() const {
            int cn = 1+(s >> CV_CN_SHIFT);
            int sz = 1 << (CV_MAT_DEPTH(s) >> 1);
            return cn*sz;
        }
        int getType() const { return s; }
        int getFileFormat() const { return img; }
        int getFlags() const { return f; }
        enum {
            IT_IMAGE,
            IT_SCALAR,
            IT_DEPTHMAP,
        };
        enum {
            IT_COMPUTED=1,
            IT_NOFLIP=2,
            IT_APPLYEXPOSURE=4,
        };
    private:
        std::string n;
        std::string ext;
        int s;
        int f;
        int img;
};

class ImageManager {
    public:
        typedef boost::interprocess::interprocess_sharable_mutex shmutex;

        ImageManager(int width, int height, int numImages);
        ImageManager(const std::string& camfile);
        virtual ~ImageManager() {;}

        const CameraParams* getCamera(int n) const;
        virtual unsigned char getFlags(const std::string& type, int n) const;
        virtual void setFlags(const std::string& type, int n, unsigned char value);
        const void* getImage(const std::string& type, int n);
        virtual const void* getImage(int i, int n);
        const void* getImage(int n);
        virtual void* getImageWriteable(const std::string& type, int n);
        const R4Matrix& getDepthToRgbTransform() const;

        virtual void saveImage(int i, int n);
        void saveImage(const std::string& type, int n);

        ImageType getImageType(int n) const { return imagetypes[n]; }
        int getNumImageTypes() const { return imagetypes.size(); }

        int width() const { return w; }
        int height() const { return h; }
        int size() const { return sz; }

        enum {
           DF_NONE=0,
           DF_INITIALIZED=1,
           DF_ERROR=128,
        };
    protected:
        ImageManager() { ; }
        void initializeImageTypes();

        void defaultinit(const std::string& camfile);
        virtual bool initializeSharedMemory();

        virtual bool readCameraFile(const std::string& filename);

        size_t computeSize() const;
        int nameToIndex(const std::string& type) const;
        shmutex* getMutex(int t, int n);

        std::vector<ImageType> imagetypes;

        int w, h, sz;
        std::vector<std::vector<void*> > images;

        R4Matrix* depth2rgb;
        CameraParams* cameras;
        unsigned char* flags;
        shmutex* mutexes;
        std::string shmname;
        std::vector<std::string> filenames;

        boost::interprocess::mapped_region mregion;
};
#endif
