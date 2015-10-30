#ifndef _FILE_IMAGE_SERVER_H
#define _FILE_IMAGE_SERVER_H
#include "imageserver.h"

class FileImageServer : public ImageServer {
    public:
        FileImageServer(const std::string& camfile,
                bool flipx=false,
                bool flipy=false,
                cb_type cb=NULL);
        ~FileImageServer();

        virtual unsigned char getFlags(const std::string& type, int n) const {
            if (loaded(n)) return ImageManager::getFlags(type, n);
            else return DF_ERROR;
        }
        virtual void setFlags(const std::string& type, int n, unsigned char value) {
            if (loaded(n)) ImageManager::setFlags(type, n, value);
        }

        virtual const void* getImage(int i, int n);
        virtual void* getImageWriteable(const std::string& type, int n);

        virtual void saveImage(int i, int n) {
            if (loaded(n)) ImageManager::saveImage(i, n);
        }
    protected:
        virtual bool initializeSharedMemory();
        virtual bool loadAllFiles() { loadImages(0); }

        bool loadImages(int i);
        bool loaded(int n) const { return n == currentidx; }

        int currentidx;
        std::vector<char*> currimages;
};
#endif
