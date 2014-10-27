#include "pairwise.h"
#include "findplanes.h"
#include "roommodel.h"
#include "util.h"
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

// TODO: Allow switching which edge to align
using namespace pcl;
using namespace std;
using namespace Eigen;

typedef PointXYZ Pt;
vector<PointCloud<Pt>::Ptr > clouds;
vector<PointCloud<Pt>::Ptr> xclouds;
vector<Matrix4d> xforms, cumxforms;
bool displaypairs = false;
int displayindex = 0;
bool ready = false;
bool alignmode = false;
bool nowrite = false;
bool labelwall = false;
int startframe = 0;
int endframe = -1;
Matrix4f currTransform = Matrix4f::Identity();
Matrix4f lastTransform = Matrix4f::Identity();
PointCloud<Pt>::Ptr lastcopy(new PointCloud<Pt>);
PointCloud<Pt>::Ptr complete(new PointCloud<Pt>);
bool track[6];

vector<int> numalignedto;
string outputprefix = "";
string xformspattern = "";
int planeinliers = 8000;

// Variables for interactive realignment
vector<int> currsrcids, currtgtids, currplanecorrespondences;
vector<Vector4d> currsrcplanes, currtgtplanes;
PointCloud<PointXYZRGB>::Ptr coloredsrc(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr coloredtgt(new PointCloud<PointXYZRGB>);
visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgbsrc(coloredsrc);
visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgbtgt(coloredtgt);
int currnumcorrespondences;
AlignmentResult currres;

void colorPlaneCorrespondences(
        PointCloud<PointXYZRGB>::Ptr colored1,
        PointCloud<PointXYZRGB>::Ptr colored2,
        vector<int>& srcids,
        vector<int>& tgtids,
        vector<int>& planecorrespondences)
{
    vector<bool> matched(planecorrespondences.size(), false);
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (planecorrespondences[i] >= 0) {
            if (planecorrespondences[i] >= matched.size()) matched.resize(planecorrespondences[i], false);
            matched[planecorrespondences[i]] = true;
        }
    }
    for (int i = 0; i < srcids.size(); ++i) {
            colored1->at(i).r = 0;
            colored1->at(i).g = 0;
            colored1->at(i).b = 0;
        if (srcids[i] == -1 || planecorrespondences[srcids[i]] == -1) {
            colored1->at(i).r = 255;
            colored1->at(i).g = 255;
            colored1->at(i).b = 255;
        } else {
            if ((planecorrespondences[srcids[i]]+1)&1) colored1->at(i).r = 255;
            if ((planecorrespondences[srcids[i]]+1)&2) colored1->at(i).g = 255;
            if ((planecorrespondences[srcids[i]]+1)&4) colored1->at(i).b = 255;
        }
    }
    for (int i = 0; i < tgtids.size(); ++i) {
        colored2->at(i).r = 0;
        colored2->at(i).g = 0;
        colored2->at(i).b = 0;
        if (tgtids[i] == -1 || !matched[tgtids[i]]) {
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

void generateComplete() {
    PointCloud<Pt>::Ptr tmp(new PointCloud<Pt>);
    for (int i = 0; i < xclouds.size(); ++i) {
        *tmp += *xclouds[i];
    }
    VoxelGrid<Pt> vgf;
    vgf.setInputCloud(tmp);
    vgf.setLeafSize(0.01f,0.01f,0.01f);
    vgf.filter(*complete);
}
void showPlanes(visualization::PCLVisualizer* v) {
    v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, "Mesh1");
    v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, "Mesh2");
    // Find planes and plane labels (on original clouds)
    currsrcplanes.clear();
    currtgtplanes.clear();
    currsrcids.clear();
    currtgtids.clear();
    currplanecorrespondences.clear();
    findPlanes(clouds[displayindex+1], currsrcplanes, currsrcids);
    findPlanes(clouds[displayindex], currtgtplanes, currtgtids);
    currnumcorrespondences = findPlaneCorrespondences(clouds[displayindex+1], clouds[displayindex], currsrcplanes, currsrcids, currtgtplanes, currtgtids, currplanecorrespondences);

    // Show colors (on current transform clouds)
    copyPointCloud(*xclouds[displayindex+1], *coloredsrc);
    copyPointCloud(*xclouds[displayindex], *coloredtgt);
    colorPlaneCorrespondences(coloredsrc, coloredtgt, currsrcids, currtgtids, currplanecorrespondences);
    cout << displayindex << ": Planes displayed with " << numalignedto[displayindex+1] << "/" << currnumcorrespondences << " correspondences" << endl;
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
        } else if (event.getKeyCode() == 'p') {
            labelwall = !labelwall;
            cout << "Label wall: " << labelwall << endl;
        } else if (event.getKeyCode() == ',') {
            displayindex--;
            if (displayindex < startframe) displayindex = endframe - 1;
            if (alignmode) {
                for (int i = 0; i < 6; ++i) track[i] = false;
                showPlanes(v);
            }
            planeinliers = setMinPlaneInliers();
        } else if (event.getKeyCode() == '.') {
            displayindex++;
            if (displayindex == endframe) displayindex = startframe;
            if (alignmode) {
                for (int i = 0; i < 6; ++i) track[i] = false;
                showPlanes(v);
            }
            planeinliers = setMinPlaneInliers();
        } else if (event.getKeyCode() == '[') {
            planeinliers -= 500;
            setMinPlaneInliers(planeinliers);
        } else if (event.getKeyCode() == ']') {
            planeinliers += 500;
            setMinPlaneInliers(planeinliers);
        } else if (displaypairs && event.getKeyCode() == 'a') {
            alignmode = true;
            cout << "Entering alignment mode...";
            for (int i = 0; i < 6; ++i) track[i] = false;
            showPlanes(v);
        } else if (alignmode) {
            if (labelwall) {
                if (event.getKeyCode() == '1') track[0] = true;
                else if (event.getKeyCode() == '2') track[1] = true;
                else if (event.getKeyCode() == '3') track[2] = true;
                else if (event.getKeyCode() == '4') track[3] = true;
                else if (event.getKeyCode() == '5') track[4] = true;
                else if (event.getKeyCode() == '6') track[5] = true;
            } else if (event.getKeyCode() == '3' && currnumcorrespondences >= 3) {
                cout << "Aligning corner...";
                currres = alignCornerToCorner(clouds[displayindex+1], clouds[displayindex], currsrcplanes, currsrcids, currtgtplanes, currtgtids, currplanecorrespondences);
                currres.type = AlignmentResult::ALIGNED_CORNER;
                Matrix4d currxform = cumxforms[displayindex]*currres.transform;
                copyPointCloud(*clouds[displayindex+1], *coloredsrc);
                transformPointCloud(*coloredsrc, *coloredsrc, currxform);
                cout << "Aligned" << endl;
            } else if (event.getKeyCode() == '2' && currnumcorrespondences >= 2) {
                cout << "Aligning edge...";
                currres = alignEdgeToEdge(clouds[displayindex+1], clouds[displayindex], currsrcplanes, currsrcids, currtgtplanes, currtgtids, currplanecorrespondences, 50);
                currres.type = AlignmentResult::ALIGNED_EDGE;
                Matrix4d currxform = cumxforms[displayindex]*currres.transform;
                copyPointCloud(*clouds[displayindex+1], *coloredsrc);
                transformPointCloud(*coloredsrc, *coloredsrc, currxform);
                cout << "Aligned" << endl;
            } else if (event.getKeyCode() == '1' && currnumcorrespondences >= 1) {
                cout << "Aligning plane...";
                currres = alignPlaneToPlane(clouds[displayindex+1], clouds[displayindex], currsrcplanes, currsrcids, currtgtplanes, currtgtids, currplanecorrespondences, 50);
                currres.type = AlignmentResult::ALIGNED_PLANE;
                Matrix4d currxform = cumxforms[displayindex]*currres.transform;
                copyPointCloud(*clouds[displayindex+1], *coloredsrc);
                transformPointCloud(*coloredsrc, *coloredsrc, currxform);
                cout << "Aligned" << endl;
            }
            if (event.getKeyCode() == 'w') {
                // Confirm current transformation
                int n = 0;
                vector<bool> alignedto(currsrcplanes.size(), false);
                if (labelwall) {
                    int ntracked = 0;
                    for (int i = 0; i < currplanecorrespondences.size(); ++i) {
                        if (currplanecorrespondences[i] > -1 && track[currplanecorrespondences[i]]) {
                            alignedto[i] = true;
                            ntracked++;
                        }
                    }
                    cout << "Aligned and tracked " << ntracked << " planes" << endl;
                } else if (currres.type != AlignmentResult::ALIGNED_NONE) {
                    switch (currres.type) {
                        case AlignmentResult::ALIGNED_CORNER:
                            n = 3;
                            break;
                        case AlignmentResult::ALIGNED_EDGE:
                            n = 2;
                            break;
                        case AlignmentResult::ALIGNED_PLANE:
                            n = 1;
                            break;
                    }
                    xforms[displayindex+1] = currres.transform;
                    char cloudname[10];
                    for (int i = displayindex+1; i < clouds.size(); ++i) {
                        cumxforms[i] = cumxforms[i-1]*xforms[i];
                        sprintf(cloudname, "cloud%03d", i);
                        transformPointCloud(*clouds[i], *xclouds[i], cumxforms[i]);
                        v->updatePointCloud<Pt>(xclouds[i], cloudname);
                    }
                    for (int j = 0; j < currplanecorrespondences.size(); ++j) {
                        if (currplanecorrespondences[j] > -1) {
                            alignedto[j] = true;
                            --n;
                            if (!n) break;
                        }
                    }
                    planeinliers = setMinPlaneInliers();
                }
                if (!nowrite && (outputprefix.length() > 0 || xformspattern.length() > 0)) {
                    string prefix;
                    if (outputprefix.length() == 0) prefix = xformspattern;
                    else prefix = outputprefix;
                    char fname[20];
                    sprintf(fname, (prefix + ".planes").c_str(), displayindex+1);
                    ofstream out(fname);
                    out << currsrcplanes.size() << " " << currres.error << endl;
                    for (int k = 0; k < currsrcplanes.size(); ++k) {
                        for (int j = 0; j < 4; ++j) out << currsrcplanes[k](j) << " ";
                        out << (alignedto[k]?1:0) << endl;
                    }
                }
                displayindex++;
                if (displayindex == endframe) displayindex = startframe;
                if (alignmode) {
                    for (int i = 0; i < 6; ++i) track[i] = false;
                    showPlanes(v);
                }
            }
        }
        int n = (displayindex+1)%clouds.size();
        if (displaypairs) {
            for (int i = startframe; i < endframe; ++i) {
                sprintf(cloudname, "cloud%03d", i);
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, cloudname);
            }
            v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, "Complete");
            if (alignmode) {
                v->updatePointCloud<PointXYZRGB>(coloredsrc, rgbsrc, "Mesh1");
                v->updatePointCloud<PointXYZRGB>(coloredtgt, rgbtgt, "Mesh2");
            } else {
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, "Mesh1");
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, "Mesh2");
                if (n == 0) {
                    v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, "Copy");
                    sprintf(cloudname, "cloud%03d", 0);
                    v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, cloudname);
                } else {
                    v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, "Copy");
                    sprintf(cloudname, "cloud%03d", displayindex);
                    v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, cloudname);
                    sprintf(cloudname, "cloud%03d", (int)((displayindex+1)%clouds.size()));
                    v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, cloudname);
                }
            }
        } else {
            v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 1.0, "Complete");
            for (int i = startframe; i < endframe; ++i) {
                sprintf(cloudname, "cloud%03d", i);
                v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, cloudname);
            }
            v->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, "Copy");
        }
    }
}

void loadFile(string fname, PointCloud<PointXYZ>::Ptr cloud, bool smoothing) {
    io::loadPCDFile<PointXYZ>(fname, *cloud);
    if (smoothing) {
        FastBilateralFilter<PointXYZ> fbf;
        fbf.setSigmaS(5);
        fbf.setSigmaR(0.04);
        fbf.setInputCloud(cloud);
        fbf.applyFilter(*cloud);
    }
    for (int i = 0; i < cloud->size(); ++i) {
        if (isnan(cloud->at(i).x)) cloud->at(i).z = cloud->at(i).x;
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "Usage: globalreg MANIFEST" << endl;
        cout << "    Options:" << endl;
        cout << "       -xforms XFORMPATTERN: Read in transforms from a file" << endl;
        cout << "       -outputprefix PREFIX: Write aligned point clouds to ply files" << endl;
        cout << "       -nodisplay: Exit after performing alignment" << endl;
        cout << "       -nowrite: Don't output new transforms" << endl;
        cout << "       -noloopclosure: Write pairwise transforms, without loop closure" << endl;
        cout << "       -nosmoothing: Don't smooth point clouds before processing" << endl;
        cout << "       -start n: Only load frames starting from n (default 0)" << endl;
        cout << "       -end n: Only load frames up to but not including n (default length-1)" << endl;
        return 0;
    }
    bool display = true;
    bool smoothing = true;
    bool doloopclosure = true;
    bool domanhattan = true;
    if (console::find_switch(argc, argv, "-nodisplay")) display = false;
    if (console::find_switch(argc, argv, "-nowrite")) nowrite = true;
    if (console::find_switch(argc, argv, "-nosmoothing")) smoothing = false;
    if (console::find_switch(argc, argv, "-noloopclosure")) {
        doloopclosure = false;
        domanhattan = false;
    }
    if (console::find_switch(argc, argv, "-manhattanonly")) {
        doloopclosure = false;
    }
    if (console::find_argument(argc, argv, "-outputprefix") > -1) {
        console::parse_argument(argc, argv, "-outputprefix", outputprefix);
    }
    if (console::find_argument(argc, argv, "-xforms") > -1) {
        console::parse_argument(argc, argv, "-xforms", xformspattern);
    }
    if (console::find_argument(argc, argv, "-start") > -1) {
        console::parse_argument(argc, argv, "-start", startframe);
    }
    if (console::find_argument(argc, argv, "-end") > -1) {
        console::parse_argument(argc, argv, "-end", endframe);
    }
    try {
        ifstream in(argv[1]);
        string s;
        getline(in,s);
        int n = 0;
        while (!in.eof()) {
            PointCloud<Pt>::Ptr cloud(new PointCloud<Pt>);
            //io::loadPCDFile<Pt>(s, *cloud);
            if (n >= startframe && (endframe < 0 || n < endframe)) {
                loadFile(s, cloud, smoothing);
                cout << "Read point cloud " << n << endl;
            }
            clouds.push_back(cloud);
            xclouds.push_back(PointCloud<Pt>::Ptr(new PointCloud<Pt>));
            getline(in,s);
            ++n;
        }
    } catch(...) {
        cerr << "Error reading manifest!" << endl;
        return 1;
    }
    if (endframe < 0) endframe = clouds.size();
    displayindex = startframe;
    cout << "Done reading point clouds" << endl;
    xforms.push_back(Matrix4d::Identity());
    cumxforms.push_back(Matrix4d::Identity());
    vector<Vector4d> srcplanes, tgtplanes;
    vector<int> srcids, tgtids;
    vector<bool> alignedto;
    RoomModel rm;
    for (int i = 1; i < clouds.size(); ++i) {
        srcplanes.clear();
        srcids.clear();
        alignedto.clear();
        double error;
        Matrix4d t;
        if (xformspattern.length() > 0) {
            char fname[20];
            sprintf(fname, (xformspattern + ".xform").c_str(), i);
            ifstream in(fname);
            for (int j = 0; j < 4; ++j) {
                for (int k = 0; k < 4; ++k) {
                    in >> t(j,k);
                }
            }
            in.close();
            if (i == 1) {
                sprintf(fname, (xformspattern + ".planes").c_str(), 0);
                in.open(fname);
                int numplanes;
                in >> numplanes >> error;
                for (int j = 0; j < numplanes; ++j) {
                    Vector4d p;
                    for (int k = 0; k < 4; ++k) in >> p(k);
                    tgtplanes.push_back(p);
                    int n;
                    in >> n;
                }
                numalignedto.push_back(0);
            }
            in.close();
            sprintf(fname, (xformspattern + ".planes").c_str(), i);
            in.open(fname);
            int numplanes;
            in >> numplanes >> error;
            int curralignedto = 0;
            for (int j = 0; j < numplanes; ++j) {
                Vector4d p;
                for (int k = 0; k < 4; ++k) in >> p(k);
                srcplanes.push_back(p);
                int n;
                in >> n;
                if (n) alignedto.push_back(true);
                else alignedto.push_back(false);
                curralignedto += n;
            }
            numalignedto.push_back(curralignedto);
            in.close();
        } else {
            vector<int> planecorrespondences;
            AlignmentResult res;
            if (i == clouds.size()) {
                res = align(clouds[0], clouds[i-1], srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
            } else {
                res = align(clouds[i], clouds[i-1], srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
            }
            alignedto.resize(srcplanes.size(), false);
            t = res.transform;
            int n = 0;
            switch (res.type) {
                case AlignmentResult::ALIGNED_CORNER:
                    n = 3;
                    break;
                case AlignmentResult::ALIGNED_EDGE:
                    n = 2;
                    break;
                case AlignmentResult::ALIGNED_PLANE:
                    n = 1;
                    break;
            }
            for (int j = 0; j < planecorrespondences.size(); ++j) {
                if (planecorrespondences[j] > -1) {
                    alignedto[j] = true;
                    --n;
                    if (!n) break;
                }
            }
            error = res.error;
            cout << "Done alignment " << i << endl;
        }
        if (!nowrite && outputprefix.length() > 0) {
            char fname[20];
            sprintf(fname, (outputprefix + ".planes").c_str(), i);
            ofstream out(fname);
            out << srcplanes.size() << " " << error << endl;
            for (int k = 0; k < srcplanes.size(); ++k) {
                for (int j = 0; j < 4; ++j) out << srcplanes[k](j) << " ";
                out << (alignedto[k]?1:0) << endl;
            }
            if (i == 1) {
                sprintf(fname, (outputprefix + ".planes").c_str(), 0);
                ofstream out(fname);
                out << tgtplanes.size() << " " << error << endl;
                for (int k = 0; k < tgtplanes.size(); ++k) {
                    for (int j = 0; j < 4; ++j) out << tgtplanes[k](j) << " ";
                    out << 0 << endl;
                }
            }
        }

        if (doloopclosure || domanhattan) {
            if (i == 1) {
                vector<Vector3d> alignedVectors;
                for (int j  = 0; j < alignedto.size(); ++j) {
                    if (alignedto[j]) {
                        Vector4d p = transformPlane(srcplanes[j], t);
                        alignedVectors.push_back(p.head(3));
                    }
                }
                rm.setAxes(alignedVectors[0], alignedVectors[1]);
                rm.addCloud(clouds[0], tgtplanes, alignedto, Matrix4d::Identity(), 0);
            }
            else if (domanhattan) {
                rm.addCloud(clouds[i], srcplanes, alignedto, t, error);
            }
        }

        cumxforms.push_back(cumxforms.back()*t);
        xforms.push_back(t);
        swap(srcplanes, tgtplanes);
        swap(srcids, tgtids);
    }

    if (doloopclosure) {
        //rm.closeLoop(t.inverse()*rm.getCumulativeTransform(i-1).inverse(), error);
        cout << "Closing all loops" << endl;
        rm.makeWallsConsistent();
    }
    if (doloopclosure || domanhattan) {
        for (int i = 0; i < clouds.size(); ++i) {
            xforms[i] = rm.getTransform(i);
            cumxforms[i] = rm.getCumulativeTransform(i);
        }
    }

    for (int i = startframe; i < endframe; ++i) {
        transformPointCloud(*clouds[i], *xclouds[i], cumxforms[i]);
        if (!display) {
            clouds[i].reset();
        }
    }
    if (display) {
        int tmp = 0;
        visualization::PCLVisualizer viewer(tmp, NULL, "Cloud viewer", new InteractorStyle());
        char cloudname[10];
        viewer.addPointCloud<PointXYZRGB>(coloredsrc, rgbsrc, "Mesh1");
        viewer.addPointCloud<PointXYZRGB>(coloredtgt, rgbtgt, "Mesh2");
        viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, "Mesh1");
        viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, "Mesh2");
        for (int i = startframe; i < endframe; ++i) {
            sprintf(cloudname, "cloud%03d", i);
            viewer.addPointCloud<Pt>(xclouds[i], cloudname);
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, i%3==0?1:0, i%3==1?1:0, i%3==2?1:0, cloudname);
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.0, cloudname);
        }
        if (endframe == xclouds.size()) {
            copyPointCloud(*xclouds[xforms.size()-1], *lastcopy);
            viewer.addPointCloud<Pt>(lastcopy, "Copy");
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1.f,1.f,1.f,"Copy");
        }
        generateComplete();
        viewer.addPointCloud<Pt>(complete, "Complete");
        viewer.registerKeyboardCallback(&keyboard_callback, &viewer);
        while (!viewer.wasStopped()) {
            viewer.spinOnce(100);
        }
    }
    if (!nowrite && (outputprefix.length() > 0 || xformspattern.length() > 0)) {
        if (outputprefix.length() == 0) outputprefix = xformspattern;
        PLYWriter w;
        char fname[20];
        for (int i = startframe; i < xclouds.size(); ++i) {
            if (i < endframe) {
                sprintf(fname, (outputprefix + ".ply").c_str(), i);
                w.write<Pt>(fname, *xclouds[i], true, false);
            }
            sprintf(fname, (outputprefix + ".xform").c_str(), i);
            ofstream out(fname);
            out << xforms[i] << endl;
            out.close();
            //*complete += *clouds[i];
            //clouds[i].reset(); // Free up memory(?)
        }
    }
    //PLYWriter w;
    //w.write<Pt>(argv[2], *complete, true, false);

    return 0;
}
