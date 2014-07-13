#include "pairwise.h"
#include "findplanes.h"
#include "util.h"
#include "plane2plane.h"
#include <pcl/point_representation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/fast_bilateral.h>
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
vector<int> srcids, tgtids, fsrcids, ftgtids;

PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
PointCloud<PointXYZRGB>::Ptr colored1(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr colored2(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr prevcolored(new PointCloud<PointXYZRGB>);
visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb1(colored1);
visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb2(colored2);
visualization::PointCloudColorHandlerRGBField<PointXYZRGB> prevrgb(prevcolored);
vector<int> planecorrespondences;
int numcorrespondences;
double normalsmoothing = 10;

#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/integral_image_normal.h>
void colorNormals() {
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    IntegralImageNormalEstimation<PointXYZRGB, Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(normalsmoothing);

    ne.setInputCloud(colored1);
    ne.compute(*normals);
    for (int i = 0; i < colored1->size(); ++i) {
        colored1->at(i).r = normals->at(i).normal_x*128+127;
        colored1->at(i).g = normals->at(i).normal_y*128+127;
        colored1->at(i).b = normals->at(i).normal_z*128+127;
    }
    ne.setInputCloud(colored2);
    ne.compute(*normals);
    for (int i = 0; i < colored2->size(); ++i) {
        colored2->at(i).r = normals->at(i).normal_x*128+127;
        colored2->at(i).g = normals->at(i).normal_y*128+127;
        colored2->at(i).b = normals->at(i).normal_z*128+127;
    }
}
void smooth(double s, double r) {
    FastBilateralFilter<PointXYZ> fbf;
    PointCloud<PointXYZ>::Ptr tmp(new PointCloud<PointXYZ>);
    fbf.setSigmaS(s);
    fbf.setSigmaR(r);
    fbf.setInputCloud(cloud1);
    fbf.applyFilter(*tmp);
    copyPointCloud(*tmp, *colored1);
    fbf.setInputCloud(cloud2);
    fbf.applyFilter(*tmp);
    copyPointCloud(*tmp, *colored2);
}

void colorPlaneCorrespondences() {
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
                colored2->at(i).r = 200;
                colored2->at(i).g = 200;
                colored2->at(i).b = 200;
            } else {
                if ((planecorrespondences[srcids[i]]+1)&1) colored1->at(i).r = 255;
                if ((planecorrespondences[srcids[i]]+1)&2) colored1->at(i).g = 255;
                if ((planecorrespondences[srcids[i]]+1)&4) colored1->at(i).b = 255;
            }
        }
    }
    for (int i = 0; i < tgtids.size(); ++i) {
        colored2->at(i).r = 0;
        colored2->at(i).g = 0;
        colored2->at(i).b = 0;
        if (tgtids[i] == -1) {
            colored2->at(i).r = 127;
            colored2->at(i).g = 127;
            colored2->at(i).b = 127;
        } else {
            if ((tgtids[i]+1)&1) colored2->at(i).r = 127;
            if ((tgtids[i]+1)&2) colored2->at(i).g = 127;
            if ((tgtids[i]+1)&4) colored2->at(i).b = 127;
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
    int srcid;
    for (srcid = 0; srcid < planecorrespondences.size(); ++srcid) {
        if (planecorrespondences[srcid] > -1) break;
    }
    int tgtid = planecorrespondences[srcid];
    if (fsrcids.size() == 0) fsrcids = srcids;
    if (ftgtids.size() == 0) ftgtids = tgtids;
    for (int i = 0; i < cloud1->size(); ++i) {
        if (fsrcids[i] == srcid) {
            colored1->at(i).r = 0;
            colored1->at(i).g = 0;
            colored1->at(i).b = 0;
        } else if (fsrcids[i] == -2) {
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
        if (ftgtids[i] == tgtid) {
            colored2->at(i).r = 0;
            colored2->at(i).g = 0;
            colored2->at(i).b = 0;
        } else if (ftgtids[i] == -2) {
            colored2->at(i).r = 63;
            colored2->at(i).g = 0;
            colored2->at(i).b = 0;
        } else {
            colored2->at(i).r = 127;
            colored2->at(i).g = 127;
            colored2->at(i).b = 127;
        }
    }
}

void updateColormode() {
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
        cout << "Coloring normals" << endl;
        colorNormals();
    }
    viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
    viewer->updatePointCloud<PointXYZRGB>(colored2, rgb2, "Mesh2");
}

vector<PointXYZ> pointcorrespondences;
void kbd_cb(const visualization::KeyboardEvent& event, void*) {
    static int step = 0;
    static double ss = 9;
    static double sr = 0.05;
    static PointCloud<PointXYZ>::Ptr aligned(new PointCloud<PointXYZ>);
    if (event.keyDown()) {
        if (event.getKeyCode() == ' ') {
            colormode = (colormode+1)%4;
            updateColormode();
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
        } else if (event.getKeyCode() == 'v') {
            copyPointCloud(*cloud1, *aligned);
            char linename[30];
            for (int i = 0; i < pointcorrespondences.size(); i+=2) {
                sprintf(linename, "line%03d", i/2);
                viewer->removeShape(linename);
            }
            Matrix4d t = align(aligned, cloud2, srcplanes, srcids, tgtplanes, tgtids).transform;
            copyPointCloud(*colored1, *prevcolored);
            viewer->addPointCloud<PointXYZRGB>(prevcolored, prevrgb, "prev");
            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0, "prev");
            transformPointCloud(*aligned, *aligned, t);
            copyPointCloud(*aligned, *colored1);
            viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
            cout << "Computed best transform" << endl;
            srcplanes.clear();
            srcids.clear();
            findPlanes(aligned, srcplanes, srcids);
        } else if (event.getKeyCode() == 'b') {
            normalsmoothing -= 1;
            cout << "Normal smoothing: " << normalsmoothing << endl;
            updateColormode();
            viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
            viewer->updatePointCloud<PointXYZRGB>(colored2, rgb2, "Mesh2");
        } else if (event.getKeyCode() == 'B') {
            normalsmoothing += 1;
            cout << "Normal smoothing: " << normalsmoothing << endl;
            updateColormode();
            viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
            viewer->updatePointCloud<PointXYZRGB>(colored2, rgb2, "Mesh2");
        } else if (event.getKeyCode() == 'n') {
            ss-=2;
            cout << ss << " " << sr << endl;
            smooth(ss,sr);
            updateColormode();
            viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
            viewer->updatePointCloud<PointXYZRGB>(colored2, rgb2, "Mesh2");
        } else if (event.getKeyCode() == 'N') {
            ss+=2;
            cout << ss << " " << sr << endl;
            smooth(ss,sr);
            updateColormode();
            viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
            viewer->updatePointCloud<PointXYZRGB>(colored2, rgb2, "Mesh2");
        } else if (event.getKeyCode() == 'm') {
            sr-=0.01;
            cout << ss << " " << sr << endl;
            smooth(ss,sr);
            updateColormode();
            viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
            viewer->updatePointCloud<PointXYZRGB>(colored2, rgb2, "Mesh2");
        } else if (event.getKeyCode() == 'M') {
            sr+=0.01;
            cout << ss << " " << sr << endl;
            smooth(ss,sr);
            updateColormode();
            viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
            viewer->updatePointCloud<PointXYZRGB>(colored2, rgb2, "Mesh2");
        } else if (event.getKeyCode() == 'z') {
            if (step == 0) {
                if (numcorrespondences == 1) {
                    for (int i = 0; i < planecorrespondences.size(); ++i) {
                        if (planecorrespondences[i] > -1) {
                            Matrix4d t = overlapPlanes(srcplanes[i], tgtplanes[planecorrespondences[i]]);
                            copyPointCloud(*colored1, *prevcolored);
                            viewer->addPointCloud<PointXYZRGB>(prevcolored, prevrgb, "prev");
                            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0, "prev");
                            transformPointCloud(*colored1, *colored1, t);
                            viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
                            cout << "Moved clouds to overlap planes" << endl;
                            copyPointCloud(*cloud1, *aligned);
                            ++step;
                            break;
                        }
                    }
                } else if (numcorrespondences >= 2) {
                    vector<int> ids;
                    for (int i = 0; i < planecorrespondences.size(); ++i) {
                        if (planecorrespondences[i] > -1) ids.push_back(i);
                    }
                    Matrix4d t = overlapEdge(srcplanes[ids[0]], srcplanes[ids[1]], tgtplanes[planecorrespondences[ids[0]]], tgtplanes[planecorrespondences[ids[1]]]);
                    copyPointCloud(*colored1, *prevcolored);
                    viewer->addPointCloud<PointXYZRGB>(prevcolored, prevrgb, "prev");
                    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0, "prev");
                    transformPointCloud(*colored1, *colored1, t);
                    viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
                    cout << "Moved clouds to overlap 2 planes" << endl;
                    copyPointCloud(*cloud1, *aligned);
                    ++step;
                }
            } else if (step&1) {
                updateColormode();
                pointcorrespondences.clear();
                double pprop = 3 - (step+1)*0.05;
                if (pprop < 0.5) pprop = 0.5;
                if (numcorrespondences == 1) {
                    partialAlignPlaneToPlane(aligned, cloud2, srcplanes, srcids, fsrcids, tgtplanes, tgtids, ftgtids, planecorrespondences, pointcorrespondences, 2000, pprop);
                } else if (numcorrespondences >= 2) {
                    partialAlignEdgeToEdge(aligned, cloud2, srcplanes, srcids, fsrcids, tgtplanes, tgtids, ftgtids, planecorrespondences, pointcorrespondences, 800, pprop);
                }
                char linename[30];
                for (int i = 0; i < pointcorrespondences.size(); i+=2) {
                    sprintf(linename, "line%03d", i/2);
                    int dx = pointcorrespondences[i].x - pointcorrespondences[i+1].x>0?255:80;
                    int dy = pointcorrespondences[i].y - pointcorrespondences[i+1].y>0?255:80;
                    int dz = pointcorrespondences[i].z - pointcorrespondences[i+1].z>0?255:80;
                    viewer->addArrow(pointcorrespondences[i+1],
                                     pointcorrespondences[i],
                                     dx, dy, dz, false, linename);
                }
                cout << "Computed correspondences" << endl;
                ++step;
            } else {
                char linename[30];
                for (int i = 0; i < pointcorrespondences.size(); i+=2) {
                    sprintf(linename, "line%03d", i/2);
                    viewer->removeShape(linename);
                }

                double pprop = 3 - step*0.05;
                if (pprop < 0.5) pprop = 0.5;
                pointcorrespondences.clear();
                Matrix4d t;
                if (numcorrespondences == 1) {
                    t = partialAlignPlaneToPlane(aligned, cloud2, srcplanes, srcids, fsrcids, tgtplanes, tgtids, ftgtids, planecorrespondences, pointcorrespondences, 6000, pprop).transform;
                } else if (numcorrespondences >= 2) {
                    t = partialAlignEdgeToEdge(aligned, cloud2, srcplanes, srcids, fsrcids, tgtplanes, tgtids, ftgtids, planecorrespondences, pointcorrespondences, 6000, pprop).transform;
                }
                pointcorrespondences.clear();
                copyPointCloud(*colored1, *prevcolored);
                viewer->updatePointCloud<PointXYZRGB>(prevcolored, prevrgb, "prev");
                viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0, "prev");
                transformPointCloud(*aligned, *aligned, t);
                copyPointCloud(*aligned, *colored1);
                viewer->updatePointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
                cout << "Computed best transform" << endl;
                for (int i = 0; i < srcplanes.size(); ++i) {
                    srcplanes[i] = transformPlane(srcplanes[i], t);
                }
                ++step;
            }
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
        cout << "Usage: align cloud1.pcd cloud2.pcd" << endl;
        return 0;
    }
    loadFile(argv[1], cloud1);
    loadFile(argv[2], cloud2);
    cout << "Finding planes... ";
    findPlanes(cloud1, srcplanes, srcids);
    cout << "Found frame 1 planes... ";
    findPlanes(cloud2, tgtplanes, tgtids);
    cout << "Found frame 2 planes" << endl;
    numcorrespondences = findPlaneCorrespondences(cloud1, cloud2, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        printf("%d(%.3f, %.3f, %.3f, %.3f): %d",
                i, srcplanes[i](0), srcplanes[i](1), srcplanes[i](2), srcplanes[i](3), planecorrespondences[i]);
        if (planecorrespondences[i] > -1) {
            int j = planecorrespondences[i];
            printf("(%.3f, %.3f, %.3f, %.3f)", tgtplanes[j](0), tgtplanes[j](1), tgtplanes[j](2), tgtplanes[j](3));
        }
        printf("\n");
    }
    for (int i = 0; i < tgtplanes.size(); ++i) {
            printf("%d: (%.3f, %.3f, %.3f, %.3f)\n", i, tgtplanes[i](0), tgtplanes[i](1), tgtplanes[i](2), tgtplanes[i](3));
    }

    copyPointCloud(*cloud1, *colored1);
    copyPointCloud(*cloud2, *colored2);
    colorPlanes();
    viewer = new visualization::PCLVisualizer("Cloud viewer");
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
