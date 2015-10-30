#ifndef _IMAGE_SERVER_H
#define _IMAGE_SERVER_H
#include "imagemanager.h"
#include <boost/function.hpp>

class ImageServer : public ImageManager {
    public:
        typedef boost::function<void(int)> cb_type;
        ImageServer(const std::string& camfile,
                bool flipx=false,
                bool flipy=false,
                cb_type cb=NULL);
        ~ImageServer();

    protected:
        ImageServer() {}
        virtual bool readCameraFile(const std::string& filename);
        virtual void initserver();
        void transformAllCameras(const R4Matrix& m);
        void flip(char* a, int w, int h, size_t bytes) const;

        virtual bool loadAllFiles();

        bool flip_x, flip_y;
        std::string camfile;

        cb_type progress_cb;
};
#endif
