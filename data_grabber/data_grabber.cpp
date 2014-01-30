#include <iostream>
#include <string>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/image_viewer.h>

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

using namespace std;
using namespace pcl;
using namespace openni_wrapper;
using namespace io;

void savepoints(const string& filename, PointCloud<PointXYZ>::Ptr cloud) {
    savePLYFileBinary(filename, *cloud);
}
class DataGrabber {
    public:
        DataGrabber ()
            : save(false), continuous(false), framecount(0), savedImages(0),
            path("data/"), rgbviewer("RGB Image") , depthviewer("Depth Image")
        {
            rgbviewer.setSize(IMG_WIDTH, IMG_HEIGHT);
            depthviewer.setSize(IMG_WIDTH, IMG_HEIGHT);
            rgbviewer.setPosition(IMG_WIDTH,0);
            depthbuffer = new unsigned short[IMG_WIDTH*IMG_HEIGHT];
            rgbbuffer = new unsigned char[3*IMG_WIDTH*IMG_HEIGHT];
        }
        DataGrabber (string outputpath)
            : save(false), continuous(false), framecount(0), savedImages(0),
            path(outputpath), rgbviewer("RGB Image") , depthviewer("Depth Image")
        {
            rgbviewer.setSize(IMG_WIDTH, IMG_HEIGHT);
            depthviewer.setSize(IMG_WIDTH, IMG_HEIGHT);
            rgbviewer.setPosition(IMG_WIDTH,0);
            depthbuffer = new unsigned short[IMG_WIDTH*IMG_HEIGHT];
            rgbbuffer = new unsigned char[3*IMG_WIDTH*IMG_HEIGHT];
        }
        ~DataGrabber() {
            delete [] depthbuffer;
            delete [] rgbbuffer;
        }

        void data_cb_(const Image::Ptr& rgb, const PointCloud<PointXYZ>::ConstPtr& cloud, const DepthImage::Ptr& depth, float constant) {
            boost::mutex::scoped_lock lock(image_mutex_);
            image_ = rgb;
            depthimage_ = depth;
            cloud_ = cloud;
            if (continuous) {
                framecount++;
                if (framecount%5 == 4) {
                    save = true;
                }
            }
            if (rgb->getEncoding() != Image::RGB) {
                image_->fillRGB(rgb->getWidth(), rgb->getHeight(), rgbbuffer);
            }
            depthimage_->fillDepthImageRaw(depth->getWidth(), depth->getHeight(), depthbuffer);
        }

        void savefiles(Image::Ptr image, PointCloud<PointXYZ>::Ptr cloud) {
            char framename[100];
            sprintf(framename, "frame%.4d", savedImages++);
            string filename(framename);
            filename = path + filename;
            boost::thread plywriter(&savepoints, filename + ".depth.ply", cloud);
            int exposure = grabber->getDevice()->getExposure();
            int gain = grabber->getDevice()->getGain();
            filename += "." + boost::lexical_cast<string>(exposure) + "." + boost::lexical_cast<string>(gain);
            if (image && image->getEncoding() != Image::RGB) {
                saveRgbPNGFile(filename +  ".png", rgbbuffer, IMG_WIDTH, IMG_HEIGHT);
            } else if (image) {
                saveRgbPNGFile(filename + ".png", (const unsigned char*) image->getMetaData().getData(), IMG_WIDTH, IMG_HEIGHT);
            }
            cout << "Saved " << filename << endl;
        }

        void keyboard_callback(const visualization::KeyboardEvent& event, void*) {
            if (event.keyDown()) {
                int exposure = grabber->getDevice()->getExposure();
                int gain = grabber->getDevice()->getGain();
                if (event.getKeyCode() == ' ') {
                    save = true;
                } else if (event.getKeyCode() == ',') {
                    if (exposure > 10) grabber->getDevice()->setExposure(exposure - 10);
                    cout << "Exposure is " << exposure-10 << endl;
                } else if (event.getKeyCode() == '.') {
                    if (exposure < 200) grabber->getDevice()->setExposure(exposure + 10);
                    cout << "Exposure is " << exposure+10 << endl;
                } else if (event.getKeyCode() == '[') {
                    if (gain > 50) grabber->getDevice()->setExposure(gain - 50);
                    cout << "Gain is " << gain-50 << endl;
                } else if (event.getKeyCode() == ']') {
                    if (gain < 1000) grabber->getDevice()->setGain(gain + 50);
                    cout << "Gain is " << gain+50 << endl;
                } else if (event.getKeyCode() == 'z') {
                    continuous = !continuous;
                }
            }
        }

        void run() {
            grabber = new OpenNIGrabber();

            rgbviewer.registerKeyboardCallback(&DataGrabber::keyboard_callback, *this);
            depthviewer.registerKeyboardCallback(&DataGrabber::keyboard_callback, *this);

            boost::function<void(const Image::Ptr&, const PointCloud<PointXYZ>::ConstPtr&, const DepthImage::Ptr&, float)> f =
                boost::bind(&DataGrabber::data_cb_, this, _1, _2, _3, _4);
            grabber->registerCallback(f);
            grabber->getDevice()->setExposure(70);
            grabber->start();
            grabber->getDevice()->setExposure(70);
            grabber->getDevice()->setGain(900);
            while(!rgbviewer.wasStopped() && !depthviewer.wasStopped()) {
                Image::Ptr image;
                DepthImage::Ptr depthimage;
                PointCloud<PointXYZ>::ConstPtr cloud;
                if (image_mutex_.try_lock()) {
                    image_.swap(image);
                    depthimage_.swap(depthimage);
                    cloud_.swap(cloud);
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
                if (save && image && cloud) {
                    PointCloud<PointXYZ>::Ptr cloudcopy(new PointCloud<PointXYZ>(*cloud));
                    savefiles(image, cloudcopy);
                    save = false;
                }
            }
            if (!rgbviewer.wasStopped()) rgbviewer.close();
            if (!depthviewer.wasStopped()) depthviewer.close();
            grabber->stop();
        }

    private:
        OpenNIGrabber* grabber;
        unsigned short* depthbuffer;
        unsigned char* rgbbuffer;
        string path;
        visualization::ImageViewer depthviewer;
        visualization::ImageViewer rgbviewer;

        PointCloud<PointXYZ>::ConstPtr cloud_;
        Image::Ptr image_;
        DepthImage::Ptr depthimage_;
        boost::mutex image_mutex_;

        bool save;
        bool continuous;
        int savedImages;
        int framecount;
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
