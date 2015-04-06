#ifndef _IMAGE_SERVER_H
#define _IMAGE_SERVER_H
#include "imagemanager.h"

class ImageServer : public ImageManager {
    public:
        ImageServer(const std::string& camfile, bool flipx=false, bool flipy=false);
        ~ImageServer();

    protected:
        bool readCameraFile(const std::string& filename);
        void transformAllCameras(const R4Matrix& m);
        void flip(char* a, int w, int h, size_t bytes);

        bool loadAllFiles();

        std::vector<std::string> filenames;

        bool flip_x, flip_y;
};
#endif
