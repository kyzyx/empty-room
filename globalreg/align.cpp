#include "pairwise.h"
#include "findplanes.h"
#include "plane2plane.h"
#include <pcl/point_representation.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace std;
using namespace Eigen;

int colormode = 0;
visualization::PCLVisualizer* viewer;
vector<Vector4d> srcplanes, tgtplanes;
vector<int> srcids, tgtids;

PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
PointCloud<PointXYZRGB>::Ptr colored1(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr colored2(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr prevcolored(new PointCloud<PointXYZRGB>);
visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb1(colored1);
visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb2(colored2);
visualization::PointCloudColorHandlerRGBField<PointXYZRGB> prevrgb(prevcolored);

void colorPlaneCorrespondences() {
    vector<int> planecorrespondences;
    int numcorrespondences = findPlaneCorrespondences(cloud1, cloud2, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
    cout << numcorrespondences << " plane correspondences found" << endl;

    for (int i = 0; i < srcids.size(); ++i) {
        if (srcids[i] == -1) {
            colored1->at(i).r = 255;
            colored1->at(i).g = 255;
            colored1->at(i).b = 255;
        } else {
            colored1->at(i).r = 0;
            colored1->at(i).g = 0;
            colored1->at(i).b = 0;
            if (planecorrespondences[srcids[i]] == -1) {
                colored1->at(i).b = 255;
                if ((srcids[i]+1)&1) colored1->at(i).r = 127;
                if ((srcids[i]+1)&2) colored1->at(i).g = 127;
            } else {
                if ((planecorrespondences[srcids[i]]+1)&1) colored1->at(i).r = 255;
                if ((planecorrespondences[srcids[i]]+1)&2) colored1->at(i).g = 255;
                if ((planecorrespondences[srcids[i]]+1)&4) colored1->at(i).b = 255;
            }
        }
    }
    for (int i = 0; i < tgtids.size(); ++i) {
        if (tgtids[i] == -1) {
            colored2->at(i).r = 127;
            colored2->at(i).g = 127;
            colored2->at(i).b = 127;
        } else {
            if ((tgtids[i]+1)&1) colored2->at(i).r = 127;
            if ((tgtids[i]+1)&2) colored2->at(i).g = 127;
            colored2->at(i).b = 0;
        }
    }
}

void colorPlanes() {
    for (int i = 0; i < srcids.size(); ++i) {
        if (srcids[i] == -1) {
            colored1->at(i).r = 255;
            colored1->at(i).g = 255;
            colored1->at(i).b = 255;
        } else if (srcids[i] == 0) {
            colored1->at(i).r = 255;
            colored1->at(i).g = 0;
            colored1->at(i).b = 0;
        } else if (srcids[i] == 1) {
            colored1->at(i).r = 0;
            colored1->at(i).g = 255;
            colored1->at(i).b = 0;
        } else if (srcids[i] == 2) {
            colored1->at(i).r = 0;
            colored1->at(i).g = 0;
            colored1->at(i).b = 255;
        } else if (srcids[i] == 3) {
            colored1->at(i).r = 255;
            colored1->at(i).g = 0;
            colored1->at(i).b = 255;
        } else if (srcids[i] == 4) {
            colored1->at(i).r = 0;
            colored1->at(i).g = 255;
            colored1->at(i).b = 255;
        } else if (srcids[i] == 5) {
            colored1->at(i).r = 255;
            colored1->at(i).g = 255;
            colored1->at(i).b = 0;
        } else {
            colored1->at(i).r = 127;
            colored1->at(i).g = 127;
            colored1->at(i).b = 127;
        }
    }
    for (int i = 0; i < tgtids.size(); ++i) {
        if (tgtids[i] == -1) {
            colored2->at(i).r = 127;
            colored2->at(i).g = 127;
            colored2->at(i).b = 127;
        } else if (tgtids[i] == 0) {
            colored2->at(i).r = 127;
            colored2->at(i).g = 0;
            colored2->at(i).b = 0;
        } else if (tgtids[i] == 1) {
            colored2->at(i).r = 0;
            colored2->at(i).g = 127;
            colored2->at(i).b = 0;
        } else if (tgtids[i] == 2) {
            colored2->at(i).r = 0;
            colored2->at(i).g = 0;
            colored2->at(i).b = 127;
        } else if (tgtids[i] == 3) {
            colored2->at(i).r = 127;
            colored2->at(i).g = 0;
            colored2->at(i).b = 127;
        } else if (tgtids[i] == 4) {
            colored2->at(i).r = 0;
            colored2->at(i).g = 127;
            colored2->at(i).b = 127;
        } else if (tgtids[i] == 5) {
            colored2->at(i).r = 127;
            colored2->at(i).g = 127;
            colored2->at(i).b = 0;
        } else {
            colored2->at(i).r = 63;
            colored2->at(i).g = 63;
            colored2->at(i).b = 63;
        }
    }
}

void colorUnfilteredPoints() {
    vector<int> planecorrespondences;
    findPlaneCorrespondences(cloud1, cloud2, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
    int srcid;
    for (srcid = 0; srcid < planecorrespondences.size(); ++srcid) {
        if (planecorrespondences[srcid] > -1) break;
    }
    int tgtid = planecorrespondences[srcid];
    vector<int> prunesrc(srcids);
    vector<int> prunetgt(tgtids);
    int MAGIC = 42;
    markDepthDiscontinuities(cloud1, 0.1, prunesrc, MAGIC);
    markDepthDiscontinuities(cloud2, 0.1, prunetgt, MAGIC);
    for (int i = 0; i < cloud1->size(); ++i) {
        if (prunesrc[i] == srcid) {
            colored1->at(i).r = 0;
            colored1->at(i).g = 0;
            colored1->at(i).b = 0;
        } else if (prunesrc[i] == MAGIC) {
            colored1->at(i).r = 63;
            colored1->at(i).g = 0;
            colored1->at(i).b = 0;
        } else {
            colored1->at(i).r = 255;
            colored1->at(i).g = 255;
            colored1->at(i).b = 255;
        }
    }
    for (int i = 0; i < cloud2->size(); ++i) {
        if (prunetgt[i] == tgtid) {
            colored2->at(i).r = 0;
            colored2->at(i).g = 0;
            colored2->at(i).b = 0;
        } else if (prunetgt[i] == MAGIC) {
            colored2->at(i).r = 63;
            colored2->at(i).g = 0;
            colored2->at(i).b = 0;
        } else {
            colored2->at(i).r = 255;
            colored2->at(i).g = 255;
            colored2->at(i).b = 255;
        }
    }
}

void kbd_cb(const visualization::KeyboardEvent& event, void*) {
    static int step = 0;
    if (event.keyDown()) {
        if (event.getKeyCode() == ' ') {
            colormode = (colormode+1)%4;
            if (colormode == 0) {           // Planes only
                cout << "Coloring detected planes" << endl;
                colorPlanes();
            } else if (colormode == 1) {    // Plane Correspondences
                cout << "Coloring detected plane correspondences" << endl;
                colorPlaneCorrespondences();
            } else if (colormode == 2) {    // Show points to align
                cout << "Coloring relevant points for constrained ICP" << endl;
                colorUnfilteredPoints();
            } else if (colormode == 3) {
            }
            viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
            viewer->updatePointCloud<PointXYZRGB>(colored2, rgb2, "Mesh2");
        } else if (event.getKeyCode() == ',') {
            static bool view1 = true;
            view1 = !view1;
            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, view1?1:0, "Mesh1");
        } else if (event.getKeyCode() == '.') {
            static bool view2 = true;
            view2 = !view2;
            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, view2?1:0, "Mesh2");
        } else if (event.getKeyCode() == '/') {
            if (step >= 1) {
                static bool viewprev = false;
                viewprev = !viewprev;
                viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, viewprev?1:0, "prev");
            }
        } else if (event.getKeyCode() == 'z') {
            if (step == 0) {
                vector<int> planecorrespondences;
                findPlaneCorrespondences(cloud1, cloud2, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
                for (int i = 0; i < planecorrespondences.size(); ++i) {
                    if (planecorrespondences[i] > -1) {
                        Matrix4d t = overlapPlanes(srcplanes[i], tgtplanes[planecorrespondences[i]]);
                        copyPointCloud(*colored1, *prevcolored);
                        viewer->addPointCloud<PointXYZRGB>(prevcolored, prevrgb, "prev");
                        viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0, "prev");
                        transformPointCloud(*colored1, *colored1, t);
                        viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
                        cout << "Moved clouds to overlap planes" << endl;
                        break;
                    }
                }
                ++step;
            } else if (step == 1) {
                vector<int> planecorrespondences;
                findPlaneCorrespondences(cloud1, cloud2, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
                vector<PointXYZ> pointcorrespondences;
                partialAlignPlaneToPlane(cloud1, cloud2, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences, pointcorrespondences, 100);
                char linename[30];
                for (int i = 0; i < pointcorrespondences.size(); i+=2) {
                    sprintf(linename, "line%03d", i/2);
                    cout << pointcorrespondences[i] << " corresponding to " << pointcorrespondences[i+1] << endl;
                    viewer->addLine(pointcorrespondences[i], pointcorrespondences[i+1],linename);
                }
                cout << "Computed correspondences" << endl;
                ++step;
            }
        }
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: align cloud1.pcd cloud2.pcd" << endl;
        return 0;
    }
    io::loadPCDFile<PointXYZ>(argv[1], *cloud1);
    io::loadPCDFile<PointXYZ>(argv[2], *cloud2);
    cout << "Finding planes... ";
    findPlanes(cloud1, srcplanes, srcids);
    cout << "Found frame 1 planes... ";
    findPlanes(cloud2, tgtplanes, tgtids);
    cout << "Foud frame 2 planes" << endl;
    vector<int> planecorrespondences;
    viewer = new visualization::PCLVisualizer("Cloud viewer");

    copyPointCloud(*cloud1, *colored1);
    copyPointCloud(*cloud2, *colored2);
    colorPlanes();
    viewer->addPointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
    viewer->addPointCloud<PointXYZRGB>(colored2, rgb2, "Mesh2");
    viewer->registerKeyboardCallback(kbd_cb, NULL);

    cout << "Press [Space] to toggle coloring" << endl;
    cout << "Press , and . to toggle individual frame display, / to toggle pre-transform display" << endl;
    cout << "Press z to advance to next stage of algorithm" << endl;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    return 0;
}
