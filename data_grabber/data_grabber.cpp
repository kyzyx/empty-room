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
            exposuremax = 50;
            exposuremin = 2;
            exposureincrement = -10;
            state = STATE_START;
        }
    public:
        DataGrabber ()
            : save(0), savedImages(0), direction(0),
            path("data/"), rgbviewer("RGB Image") , depthviewer("Depth Image")
        {
            init();
        }
        DataGrabber (string outputpath)
            : save(0), savedImages(0), direction(0),
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
            if (rgb->getEncoding() != Image::RGB) {
                image_->fillRGB(rgb->getWidth(), rgb->getHeight(), rgbbuffer);
            }
            adjustExposure();
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

        bool isOverExposed() {
            const int maxpixelval = 240;
            int numoverexposed = 0;
            unsigned char* pixels = (unsigned char*) image_->getData();
            for (int i = 0; i < IMG_WIDTH*IMG_HEIGHT; ++i) {
                if (pixels[i*3] > maxpixelval && pixels[i*3+1] > maxpixelval && pixels[i*3+2] > maxpixelval) {
                    numoverexposed++;
                }
            }
            return numoverexposed > 10;
        }
        bool isUnderExposed() {
            const int minpixelval = 10;
            int numunderexposed = 0;
            unsigned char* pixels = (unsigned char*) image_->getData();
            for (int i = 0; i < IMG_WIDTH*IMG_HEIGHT; ++i) {
                if (pixels[i*3] < minpixelval && pixels[i*3+1] < minpixelval && pixels[i*3+2] < minpixelval) {
                    numunderexposed++;
                }
            }
            return numunderexposed > 2*IMG_WIDTH;
        }

        void setExposureAndGain(int newexposure) {
            prevexposure = currexposure;
            currexposure = newexposure;
            int currgain = currexposure*20;
            if (currexposure < 10) currgain = (currexposure-2)*12.5 + 100;
            grabber->getDevice()->setExposure(currexposure);
            grabber->getDevice()->setGain(currgain);
        }

        bool exposureReady() {
            if (prevexposure < 20) return framewait == 5;
            else if (prevexposure < 40) return framewait == 4;
            else return framewait == 3;
        }

        bool resetDirection() {
            direction = (currexposure == exposuremin)?1:-1;
        }
        int nextExposure() {
            if (direction == 0) resetDirection();
            if (currexposure < 10-direction) {
                if (currexposure + direction*2 == exposuremin) {
                    direction = 1;
                    return exposuremin;
                }
                return currexposure + direction*2;
            }
            else if (currexposure <= exposuremax) {
                if (currexposure + direction*5 == exposuremax) {
                    direction = -1;
                    return exposuremax;
                }
                return currexposure + direction*5;
            }
        }

        void adjustExposure() {
            if (state == STATE_BRACKET1 || state == STATE_BRACKET2 || state == STATE_CAPTURING) {
                if (exposureReady()) {
                    if (state == STATE_BRACKET1) {
                        if (isOverExposed()) {
                            if (currexposure <= 2) {
                                cerr << "Note: Brightest object is too bright for this camera!" << endl;
                            }
                            cerr << "Set lower bracket to " << prevexposure << endl;
                            cout << "Please aim camera at darkest region of scene" << endl;
                            exposuremin = prevexposure;
                            state = STATE_BRACKETED1;
                        } else {
                            if (currexposure == 50) {
                                cerr << "Note: No significant light source detected!" << endl;
                                state = STATE_ERROR;
                            } else {
                                setExposureAndGain(nextExposure());
                            }
                        }
                    } else if (state == STATE_BRACKET2) {
                        if (isUnderExposed()) {
                            exposuremax = prevexposure;
                            cerr << "Set upper bracket to " << prevexposure << endl;
                            if (exposuremax == 50) cerr << "Note: Your scene is very dark" << endl;
                            state = STATE_BRACKETED2;
                        } else {
                            if (currexposure == 2) {
                                cerr << "Something strange happened..." << endl;
                                state = STATE_ERROR;
                            } else {
                                setExposureAndGain(nextExposure());
                            }
                        }
                    } else {
                        save = currexposure;
                        setExposureAndGain(nextExposure());
                    }
                    framewait = 0;
                }
                framewait++;
            }
        }

        void statechange() {
            if (state == STATE_START) {
                // Capture brightest area
                state = STATE_BRACKET1;
                framewait = 0;
                setExposureAndGain(exposuremin);
                prevexposure = exposuremin;
                resetDirection();
            } else if (state == STATE_BRACKETED1) {
                // Capture darkest area
                state = STATE_BRACKET2;
                framewait = 0;
                setExposureAndGain(exposuremax);
                prevexposure = exposuremax;
                resetDirection();
            } else if (state == STATE_BRACKETED2) {
                // Toggle capture
                state = STATE_CAPTURING;
                framewait = 0;
                setExposureAndGain(exposuremin);
                resetDirection();
            } else if (state == STATE_CAPTURING) {
                state = STATE_BRACKETED2;
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
            setExposureAndGain(2);
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
        int savedImages;

        enum {STATE_START, STATE_BRACKET1, STATE_BRACKETED1, STATE_BRACKET2, STATE_BRACKETED2, STATE_CAPTURING, STATE_ERROR};
        int state;
        int exposureincrement;
        int currexposure;
        int prevexposure;
        int exposuremax;
        int exposuremin;
        int direction;

        int framewait;
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
