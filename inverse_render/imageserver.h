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
        bool readCameraFile(const std::string& filename);
        void transformAllCameras(const R4Matrix& m);
        void flip(char* a, int w, int h, size_t bytes);

        bool loadAllFiles();

        std::vector<std::string> filenames;

        bool flip_x, flip_y;

        cb_type progress_cb;
};
#endif
