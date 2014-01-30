#include <iostream>
#include <string>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/image_viewer.h>

#define BUFFER_SIZE 640*480

using namespace std;
using namespace pcl;
using namespace openni_wrapper;

class DataGrabber {
    public:
        DataGrabber () :  path("data/"), rgbviewer("RGB Image") , depthviewer("Depth Image") {
            rgbviewer.setSize(640, 480);
            depthviewer.setSize(640, 480);
            rgbviewer.setPosition(640,0);
            depthbuffer = new unsigned short[BUFFER_SIZE];
            rgbbuffer = new unsigned char[3*BUFFER_SIZE];
        }
        DataGrabber (string outputpath) : path(outputpath), rgbviewer("RGB Image") , depthviewer("Depth Image") {
            rgbviewer.setSize(640, 480);
            depthviewer.setSize(640, 480);
            rgbviewer.setPosition(640,0);
            depthbuffer = new unsigned short[BUFFER_SIZE];
            rgbbuffer = new unsigned char[3*BUFFER_SIZE];
        }
        ~DataGrabber() {
            delete [] depthbuffer;
            delete [] rgbbuffer;
        }

        void data_cb_(const Image::Ptr& rgb, const DepthImage::Ptr& depth, float constant) {
            boost::mutex::scoped_lock lock(image_mutex_);
            image_ = rgb;
            depthimage_ = depth;
            if (rgb->getEncoding() != Image::RGB) {
                image_->fillRGB(rgb->getWidth(), rgb->getHeight(), rgbbuffer);
            }
            depthimage_->fillDepthImageRaw(depth->getWidth(), depth->getHeight(), depthbuffer);
        }

        void run() {
            OpenNIGrabber* grabber = new OpenNIGrabber();

            boost::function<void(const Image::Ptr&, const DepthImage::Ptr&, float)> f =
                boost::bind(&DataGrabber::data_cb_, this, _1, _2, _3);
            grabber->registerCallback(f);
            grabber->getDevice()->setExposure(900);
            grabber->start();
            while(!rgbviewer.wasStopped() && !depthviewer.wasStopped()) {
                Image::Ptr image;
                DepthImage::Ptr depthimage;
                if (image_mutex_.try_lock()) {
                    image_.swap(image);
                    depthimage_.swap(depthimage);
                    image_mutex_.unlock();
                }
                if (image && depthimage) {
                    if (image->getEncoding() == Image::RGB) {
                        rgbviewer.addRGBImage((const unsigned char*) image->getMetaData().getData(), image->getWidth(), image->getHeight());
                    } else {
                        rgbviewer.addRGBImage(rgbbuffer, image->getWidth(), image->getHeight());
                    }
                    depthviewer.addShortImage(depthbuffer, image->getWidth(), image->getHeight(),0,7000);
                    depthviewer.spinOnce();
                    rgbviewer.spinOnce();
                }
            }
            if (!rgbviewer.wasStopped()) rgbviewer.close();
            if (!depthviewer.wasStopped()) depthviewer.close();
            grabber->stop();
        }

    private:
        unsigned short* depthbuffer;
        unsigned char* rgbbuffer;
        string path;
        visualization::ImageViewer depthviewer;
        visualization::ImageViewer rgbviewer;

        Image::Ptr image_;
        DepthImage::Ptr depthimage_;
        boost::mutex image_mutex_;
};

int main(int argc, char** argv) {
    string path = "data/";
    if (argc > 1) {
        path = argv[1];
    }
    DataGrabber dg(path);
    dg.run();
    return 0;
}
