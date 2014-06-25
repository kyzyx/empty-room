#include "pairwise.h"
#include "findplanes.h"
#include <pcl/io/io.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace std;
using namespace Eigen;
visualization::PCLVisualizer* viewer;

bool freeze = false;
PointCloud<PointXYZ>::ConstPtr lastcloud;

void showColoredCloud(PointCloud<PointXYZ>::ConstPtr cloud, vector<int>& ids, bool add=true) {
    static char name[] = {'M','e','s','h','0','\0'};
    static bool added = false;
    PointCloud<PointXYZRGB>::Ptr c(new PointCloud<PointXYZRGB>);
    copyPointCloud(*cloud, *c);
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] == -1) {
            c->at(i).r = 255;
            c->at(i).g = 255;
            c->at(i).b = 255;
        } else if (ids[i] == 0) {
            c->at(i).r = 255;
            c->at(i).g = 0;
            c->at(i).b = 0;
        } else if (ids[i] == 1) {
            c->at(i).r = 0;
            c->at(i).g = 255;
            c->at(i).b = 0;
        } else if (ids[i] == 2) {
            c->at(i).r = 0;
            c->at(i).g = 0;
            c->at(i).b = 255;
        } else if (ids[i] == 3) {
            c->at(i).r = 255;
            c->at(i).g = 0;
            c->at(i).b = 255;
        } else if (ids[i] == 4) {
            c->at(i).r = 0;
            c->at(i).g = 255;
            c->at(i).b = 255;
        } else if (ids[i] == 5) {
            c->at(i).r = 255;
            c->at(i).g = 255;
            c->at(i).b = 0;
        } else {
            c->at(i).r = 127;
            c->at(i).g = 127;
            c->at(i).b = 127;
        }
    }
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(c);
    if (!added) {
        viewer->addPointCloud<PointXYZRGB>(c, rgb, name);
        added = true;
    }
    if (add) {
        name[4]++;
        added = false;
    } else {
        viewer->updatePointCloud<PointXYZRGB>(c, rgb, name);
    }
}

void processCloud(const PointCloud<PointXYZ>::ConstPtr& cloud) {
    vector<Vector4d> planes;
    vector<int> ids;
    findPlanes(cloud, planes, ids);
    showColoredCloud(cloud, ids, false);
}

void cloud_cb(const PointCloud<PointXYZ>::ConstPtr& cloud) {
    if (!viewer->wasStopped() && !freeze) {
        processCloud(cloud);
        lastcloud = cloud;
    }
}

void kbd_cb(const visualization::KeyboardEvent& event, void*) {
    if (event.keyDown()) {
        if (event.getKeyCode() == ' ') {
            freeze = !freeze;
        }
    }
}

int main(int argc, char** argv) {
    Grabber* g = NULL;
    viewer = new visualization::PCLVisualizer("Cloud viewer");

    PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);

    if (argc > 1) {
        cout << "Loading..." << endl;
        io::loadPCDFile<PointXYZ>(argv[1], *cloud1);
        if (argc > 2) {
            io::loadPCDFile<PointXYZ>(argv[2], *cloud2);
        }
        cout << "Done loading" << endl;

        vector<Vector4d> srcplanes, tgtplanes;
        vector<int> srcids, tgtids;
        //srcids.resize(cloud1->size(),-1);
        //tgtids.resize(cloud2->size(),-1);

        cout << "Finding planes..." << endl;
        findPlanes(cloud1, srcplanes, srcids);
        if (argc > 2) {
            cout << "Done finding frame 1 planes" << endl;
            findPlanes(cloud2, tgtplanes, tgtids);
        }
        cout << "Done finding planes" << endl;

        showColoredCloud(cloud1, srcids);
    } else {
        cout << "Initializing OpenNI2 grabber" << endl;
        g = new io::OpenNI2Grabber();
        cout << "Initialized" << endl;
        boost::function<void (const PointCloud<PointXYZ>::ConstPtr&)> f =
            boost::bind(&cloud_cb, _1);
        viewer->registerKeyboardCallback(kbd_cb, NULL);
        g->registerCallback(f);
        cout << "Starting" << endl;
        g->start();
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    if (g) {
        g->stop();
    }

    return 0;
}
