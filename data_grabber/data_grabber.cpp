#include <iostream>
#include <string>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

class DataGrabber {
    public:
        DataGrabber () :  path("data/"), viewer("Empty Room Data Grabber") {}
        DataGrabber (string outputpath) : path(outputpath), viewer("Empty Room Data Grabber") {}

        void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
            if (!viewer.wasStopped()) {
                viewer.showCloud(cloud);
            }
        }

        void run() {
            pcl::Grabber* grabber = new pcl::OpenNIGrabber();

            boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
                boost::bind(&DataGrabber::cloud_cb_, this, _1);
            grabber->registerCallback(f);
            grabber->start();
            while(!viewer.wasStopped()) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            }
            grabber->stop();
        }

    private:
        string path;
        pcl::visualization::CloudViewer viewer;
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
