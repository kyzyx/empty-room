#include <iostream>
#include <string>
#include <pcl/io/grabber.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/image_viewer.h>

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

using namespace std;
using namespace pcl;
using namespace io;

void savepoints(const string& filename, PointCloud<PointXYZ>::Ptr cloud) {
    PLYWriter w;
    w.write<PointXYZ>(filename, *cloud, true, false);
}
class DataGrabber {
    private:
        void init() {
            rgbviewer.setSize(IMG_WIDTH, IMG_HEIGHT);
            depthviewer.setSize(IMG_WIDTH, IMG_HEIGHT);
            rgbviewer.setPosition(IMG_WIDTH,0);
            depthbuffer = new unsigned short[IMG_WIDTH*IMG_HEIGHT];
            rgbbuffer = new unsigned char[3*IMG_WIDTH*IMG_HEIGHT];
            exposuremax = 0;
            exposuremin = 0;
            exposureincrement = -10;
            state = STATE_START;
        }
    public:
        DataGrabber ()
            : save(0), continuous(false), savedImages(0),
            path("data/"), rgbviewer("RGB Image") , depthviewer("Depth Image")
        {
            init();
        }
        DataGrabber (string outputpath)
            : save(0), continuous(false), savedImages(0),
            path(outputpath), rgbviewer("RGB Image") , depthviewer("Depth Image")
        {
            init();
        }
        DataGrabber (string outputpath, int freq)
            : save(0), continuous(false), savedImages(0),
            path(outputpath), rgbviewer("RGB Image") , depthviewer("Depth Image")
        {
            init();
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
            adjustExposure();
            if (rgb->getEncoding() != Image::RGB) {
                image_->fillRGB(rgb->getWidth(), rgb->getHeight(), rgbbuffer);
            }
            depthimage_->fillDepthImageRaw(depth->getWidth(), depth->getHeight(), depthbuffer);
        }

        void savefiles(Image::Ptr image, PointCloud<PointXYZ>::Ptr cloud, int exposure) {
            char framename[100];
            sprintf(framename, "frame%.4d.%.2d", savedImages++, exposure);
            string filename(framename);
            filename = path + filename;
            boost::thread plywriter(&savepoints, filename + ".ply", cloud);
            if (image && image->getEncoding() != Image::RGB) {
                saveRgbPNGFile(filename +  ".png", rgbbuffer, IMG_WIDTH, IMG_HEIGHT);
            } else if (image) {
                saveRgbPNGFile(filename + ".png", (const unsigned char*) image->getData(), IMG_WIDTH, IMG_HEIGHT);
            }
            cout << "Saved " << filename << endl;
        }

        double meanLuminance() {
            unsigned char* pixels = (unsigned char*) image_->getData();
            double tot = 0;
            for (int i = 0; i < IMG_WIDTH*IMG_HEIGHT; ++i) {
                tot += pixels[i*3] + pixels[i*3+1] + pixels[i*3+2];
            }
            tot /= IMG_WIDTH*IMG_HEIGHT*3;
        }

        bool isOverExposed() {
            const int maxpixelval = 240;
            int numoverexposed = 0;
            unsigned char* pixels = (unsigned char*) image_->getData();
            for (int i = 0; i < IMG_WIDTH*IMG_HEIGHT; ++i) {
                if (pixels[i*3] > maxpixelval && pixels[i*3+1] > maxpixelval && pixels[i*3+2] > maxpixelval) {
                    numoverexposed++;
                }
            }
            return numoverexposed > IMG_WIDTH;
        }
        bool isUnderExposed() {
            const int minpixelval = 15;
            int numunderexposed = 0;
            unsigned char* pixels = (unsigned char*) image_->getData();
            for (int i = 0; i < IMG_WIDTH*IMG_HEIGHT; ++i) {
                if (pixels[i*3] < minpixelval && pixels[i*3+1] < minpixelval && pixels[i*3+2] < minpixelval) {
                    numunderexposed++;
                }
            }
            return numunderexposed > IMG_WIDTH;
        }

        void adjustExposure() {
            if (state == STATE_BRACKET1 || state == STATE_BRACKET2 || state == STATE_CAPTURING) {
                double currlum = meanLuminance();
                cout << abs(currlum - meanlum) << endl;
                if (abs(currlum - meanlum) > 10) {
                    if (state == STATE_BRACKET1) {
                        if (isOverExposed()) {
                            if (currexposure <= 2 && currgain <= 100) {
                                cerr << "Note: Brightest object is too bright for this camera!" << endl;
                            }
                            cerr << "Set lower bracket to " << prevexposure << " at gain " << currgain << endl;
                            cout << "Please aim camera at darkest region of scene" << endl;
                            exposuremin = prevexposure;
                            state = STATE_BRACKETED1;
                        } else {
                            prevexposure = currexposure;
                            if (currexposure < 10) currexposure += 2;
                            else if (currexposure < 20) currexposure += 5;
                            else {
                                if (currgain < 1000) {
                                    currgain += 100;
                                    currexposure = 5;
                                    // Increase gain
                                    grabber->getDevice()->setGain(currgain);
                                    grabber->getDevice()->setExposure(currexposure);
                                } else {
                                    cerr << "Note: No significant light source detected!" << endl;
                                    state = STATE_ERROR;
                                }
                            }
                            grabber->getDevice()->setExposure(currexposure);
                        }
                    } else if (state == STATE_BRACKET2) {
                        if (isUnderExposed()) {
                            if (prevexposure == currexposure && currgain < 1000) {
                                currgain += 100;
                                currexposure = 50;
                                grabber->getDevice()->setGain(currgain);
                                grabber->getDevice()->setExposure(currexposure);
                            } else {
                                exposuremax = prevexposure;
                                cerr << "Set upper bracket to " << prevexposure << " and gain " << currgain << endl;
                                state = STATE_BRACKETED2;
                            }
                        } else {
                            prevexposure = currexposure;
                            if (currexposure < 2) {
                                if (currgain <= 100) {
                                    cerr << "Scene too bright for camera!" << endl;
                                    state = STATE_ERROR;
                                } else {
                                    cerr << "Something went wrong here!" << endl;
                                    state = STATE_ERROR;
                                }
                            }
                            else if (currexposure < 10) currexposure -= 2;
                            else if (currexposure < 50) currexposure -= 5;
                            grabber->getDevice()->setExposure(currexposure);
                        }
                    } else {
                        save = currexposure;
                        prevexposure = currexposure;
                        currexposure += exposureincrement;
                        if (currexposure == exposuremax || currexposure == exposuremin) {
                            exposureincrement = -exposureincrement;
                        }
                        grabber->getDevice()->setExposure(currexposure);
                    }
                }
                meanlum = currlum;
            }
        }

        void statechange() {
            if (state == STATE_START) {
                // Capture brightest area
                state = STATE_BRACKET1;
                currexposure = prevexposure = exposuremin = 2;
                currgain = 100;
                meanlum = meanLuminance();
                grabber->getDevice()->setExposure(currexposure);
                grabber->getDevice()->setGain(currgain);
            } else if (state == STATE_BRACKETED1) {
                // Capture darkest area
                state = STATE_BRACKET2;
                currexposure = prevexposure = exposuremax = 50;
                meanlum = meanLuminance();
                grabber->getDevice()->setExposure(currexposure);
            } else if (state == STATE_BRACKETED2 || state == STATE_CAPTURING) {
                // Toggle capture
                continuous = !continuous;
            }
        }
        void keyboard_callback(const visualization::KeyboardEvent& event, void*) {
            if (event.keyDown()) {
                if (event.getKeyCode() == ' ') {
                    statechange();
                }
            }
        }

        void run() {
            grabber = new io::OpenNI2Grabber();

            rgbviewer.registerKeyboardCallback(&DataGrabber::keyboard_callback, *this);
            depthviewer.registerKeyboardCallback(&DataGrabber::keyboard_callback, *this);

            boost::function<void(const Image::Ptr&, const PointCloud<PointXYZ>::ConstPtr&, const DepthImage::Ptr&, float)> f =
                boost::bind(&DataGrabber::data_cb_, this, _1, _2, _3, _4);
            grabber->registerCallback(f);
            grabber->start();
            grabber->getDevice()->setExposure(2);
            grabber->getDevice()->setGain(400);
            cout << "Please point camera at brightest light emitter in scene, then press [Space]" << endl;
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
                        rgbviewer.addRGBImage((const unsigned char*) image->getData(), image->getWidth(), image->getHeight());
                    } else {
                        rgbviewer.addRGBImage(rgbbuffer, image->getWidth(), image->getHeight());
                    }
                    depthviewer.addShortImage(depthbuffer, image->getWidth(), image->getHeight(),0,7000);
                    depthviewer.spinOnce();
                    rgbviewer.spinOnce();
                }
                if (save && image && cloud) {
                    PointCloud<PointXYZ>::Ptr cloudcopy(new PointCloud<PointXYZ>(*cloud));
                    savefiles(image, cloudcopy, save);
                    save = 0;
                }
            }
            if (!rgbviewer.wasStopped()) rgbviewer.close();
            if (!depthviewer.wasStopped()) depthviewer.close();
            grabber->stop();
        }

    private:
        io::OpenNI2Grabber* grabber;
        unsigned short* depthbuffer;
        unsigned char* rgbbuffer;
        string path;
        visualization::ImageViewer depthviewer;
        visualization::ImageViewer rgbviewer;

        PointCloud<PointXYZ>::ConstPtr cloud_;
        Image::Ptr image_;
        DepthImage::Ptr depthimage_;
        boost::mutex image_mutex_;

        int save;
        bool continuous;
        int savedImages;

        enum {STATE_START, STATE_BRACKET1, STATE_BRACKETED1, STATE_BRACKET2, STATE_BRACKETED2, STATE_CAPTURING, STATE_ERROR};
        double meanlum;
        int state;
        int exposureincrement;
        int currgain;
        int currexposure;
        int prevexposure;
        int exposuremax;
        int exposuremin;
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
