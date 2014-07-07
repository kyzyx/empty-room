#include "pairwise.h"
#include "findplanes.h"
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <fstream>
#include <vector>

using namespace pcl;
using namespace std;
using namespace Eigen;

typedef PointXYZ Pt;
vector<PointCloud<Pt>::Ptr > clouds;
vector<Matrix4d> xforms;
bool displaypairs = false;
int displayindex = 0;
bool ready = false;
Matrix4f currTransform = Matrix4f::Identity();
Matrix4f lastTransform = Matrix4f::Identity();
PointCloud<Pt>::Ptr lastcopy(new PointCloud<Pt>);
PointCloud<Pt>::Ptr complete(new PointCloud<Pt>);

void generateComplete() {
    complete->clear();
    for (int i = 0; i < clouds.size(); ++i) {
        *complete += *clouds[i];
    }
    VoxelGrid<Pt> vgf;
    vgf.setInputCloud(complete);
    vgf.setLeafSize(0.01f,0.01f,0.01f);
    vgf.filter(*complete);
}

void keyboard_callback(const visualization::KeyboardEvent& event, void* viewer) {
    if (event.keyDown()) {
        char cloudname[20];
        visualization::PCLVisualizer* v = (visualization::PCLVisualizer*) viewer;
        if (event.getKeyCode() == ' ') {
            displaypairs = !displaypairs;
            if (!displaypairs) {
                generateComplete();
            }
        }
        else if (event.getKeyCode() == ',') {
            displayindex--;
            if (displayindex < 0) displayindex = clouds.size() - 1;
        }
        else if (event.getKeyCode() == '.') {
            displayindex++;
            if (displayindex == clouds.size()) displayindex = 0;
        }
        int n = (displayindex+1)%clouds.size();
        if (displaypairs) {
            for (int i = 0; i < clouds.size(); ++i) {
                sprintf(cloudname, "cloud%03d", i);
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, cloudname);
            }
            v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, "Complete");
            if (n == 0) {
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, "Copy");
                sprintf(cloudname, "cloud%03d", 0);
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, cloudname);
            }
            else {
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, "Copy");
                sprintf(cloudname, "cloud%03d", displayindex);
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, cloudname);
                sprintf(cloudname, "cloud%03d", (int)((displayindex+1)%clouds.size()));
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, cloudname);
            }
        } else {
            v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, "Complete");
            for (int i = 0; i < clouds.size(); ++i) {
                sprintf(cloudname, "cloud%03d", i);
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, cloudname);
            }
            v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, "Copy");
        }
    }
}

void loadFile(string fname, PointCloud<PointXYZ>::Ptr cloud) {
    io::loadPCDFile<PointXYZ>(fname, *cloud);
    FastBilateralFilter<PointXYZ> fbf;
    fbf.setSigmaS(9);
    fbf.setSigmaR(0.05);
    fbf.setInputCloud(cloud);
    fbf.applyFilter(*cloud);
    for (int i = 0; i < cloud->size(); ++i) {
        if (isnan(cloud->at(i).x)) cloud->at(i).z = cloud->at(i).x;
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: globalreg MANIFEST prefix" << endl;
        return 0;
    }
    string prefix = argv[2];
    bool subsample = false;
    bool display = true;
    if (console::find_switch(argc, argv, "-subsample")) subsample = true;
    if (console::find_switch(argc, argv, "-nodisplay")) display = false;
    try {
        ifstream in(argv[1]);
        string s;
        getline(in,s);
        int n = 0;
        while (!in.eof()) {
            PointCloud<Pt>::Ptr cloud(new PointCloud<Pt>);
            //io::loadPCDFile<Pt>(s, *cloud);
            loadFile(s, cloud);
            cout << "Read point cloud " << n << endl;
            clouds.push_back(cloud);
            getline(in,s);
            ++n;
        }
    } catch(...) {
        cerr << "Error reading manifest!" << endl;
        return 1;
    }
    cout << "Done reading point clouds" << endl;
    xforms.push_back(Matrix4d::Identity());
    vector<Vector4d> srcplanes, tgtplanes;
    vector<int> srcids, tgtids;
    for (int i = 1; i < clouds.size(); ++i) {
        srcplanes.clear(); tgtplanes.clear();
        srcids.clear(); tgtids.clear();
        Matrix4d t = align(clouds[i], clouds[i-1], srcplanes, srcids, tgtplanes, tgtids);
        xforms.push_back(t*xforms.back());
        cout << "Done alignment " << i << endl;
    }

    if (display) {
        visualization::PCLVisualizer viewer("Cloud viewer");
        char cloudname[10];
        for (int i = 0; i < clouds.size(); ++i) {
            sprintf(cloudname, "cloud%03d", i);
            transformPointCloud(*clouds[i], *clouds[i], xforms[i]);
            viewer.addPointCloud<Pt>(clouds[i], cloudname);
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, i%3==0?1:0, i%3==1?1:0, i%3==2?1:0, cloudname);
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, cloudname);
        }
        copyPointCloud(*clouds[clouds.size()-1], *lastcopy);
        viewer.addPointCloud<Pt>(lastcopy, "Copy");
        viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1.f,1.f,1.f,"Copy");
        generateComplete();
        viewer.addPointCloud<Pt>(complete, "Complete");
        viewer.registerKeyboardCallback(&keyboard_callback, &viewer);
        while (!viewer.wasStopped()) {
            viewer.spinOnce(100);
        }
    }
    PLYWriter w;
    char fname[20];
    for (int i = 0; i < clouds.size(); ++i) {
        sprintf(fname, (prefix + "%04d.ply").c_str(), i);
        w.write<Pt>(fname, *clouds[i], true, false);
        //*complete += *clouds[i];
        //clouds[i].reset(); // Free up memory(?)
    }
    //PLYWriter w;
    //w.write<Pt>(argv[2], *complete, true, false);

    return 0;
}
