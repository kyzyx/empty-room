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

using namespace pcl;
using namespace std;
using namespace Eigen;

typedef PointXYZ Pt;
vector<PointCloud<Pt>::Ptr > clouds;
vector<Matrix4d> xforms, cumxforms;
bool displaypairs = false;
int displayindex = 0;
bool ready = false;
Matrix4f currTransform = Matrix4f::Identity();
Matrix4f lastTransform = Matrix4f::Identity();
PointCloud<Pt>::Ptr lastcopy(new PointCloud<Pt>);
PointCloud<Pt>::Ptr complete(new PointCloud<Pt>);

void generateComplete() {
    PointCloud<Pt>::Ptr tmp(new PointCloud<Pt>);
    for (int i = 0; i < clouds.size(); ++i) {
        *tmp += *clouds[i];
    }
    VoxelGrid<Pt> vgf;
    vgf.setInputCloud(tmp);
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
        cout << "       -noloopclosure: Write pairwise transforms, without loop closure" << endl;
        cout << "       -nosmoothing: Don't smooth point clouds before processing" << endl;
        return 0;
    }
    string outputprefix = "";
    string xformspattern = "";
    bool display = true;
    bool smoothing = true;
    bool doloopclosure = true;
    if (console::find_switch(argc, argv, "-nodisplay")) display = false;
    if (console::find_switch(argc, argv, "-nosmoothing")) smoothing = false;
    if (console::find_switch(argc, argv, "-noloopclosure")) doloopclosure = false;
    if (console::find_argument(argc, argv, "-outputprefix") > -1) {
        console::parse_argument(argc, argv, "-outputprefix", outputprefix);
    }
    if (console::find_argument(argc, argv, "-xforms") > -1) {
        console::parse_argument(argc, argv, "-xforms", xformspattern);
    }
    try {
        ifstream in(argv[1]);
        string s;
        getline(in,s);
        int n = 0;
        while (!in.eof()) {
            PointCloud<Pt>::Ptr cloud(new PointCloud<Pt>);
            //io::loadPCDFile<Pt>(s, *cloud);
            loadFile(s, cloud, smoothing);
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
    cumxforms.push_back(Matrix4d::Identity());
    vector<Vector4d> srcplanes, tgtplanes;
    vector<int> srcids, tgtids;
    vector<bool> alignedto;
    RoomModel rm;
    for (int i = 1; i <= clouds.size(); ++i) {
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
            }
            in.close();
            sprintf(fname, (xformspattern + ".planes").c_str(), i);
            in.open(fname);
            int numplanes;
            in >> numplanes >> error;
            for (int j = 0; j < numplanes; ++j) {
                Vector4d p;
                for (int k = 0; k < 4; ++k) in >> p(k);
                srcplanes.push_back(p);
                int n;
                in >> n;
                if (n) alignedto.push_back(true);
                else alignedto.push_back(false);
            }
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
            if (outputprefix.length() > 0) {
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
        }

        if (doloopclosure) {
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
            if (i == clouds.size()) {
                rm.closeLoop(t.inverse()*rm.getCumulativeTransform(i-1).inverse(), error);
            }
            else {
                rm.addCloud(clouds[i], srcplanes, alignedto, t, error);
            }
        }
        cout << "Done alignment " << i << endl;

        cumxforms.push_back(cumxforms.back()*t);
        xforms.push_back(t);
        swap(srcplanes, tgtplanes);
        swap(srcids, tgtids);
    }

    if (doloopclosure) {
        for (int i = 0; i < clouds.size(); ++i) {
            xforms[i] = rm.getTransform(i);
            cumxforms[i] = rm.getCumulativeTransform(i);
        }
    }

    if (display) {
        visualization::PCLVisualizer viewer("Cloud viewer");
        char cloudname[10];
        for (int i = 0; i < clouds.size(); ++i) {
            sprintf(cloudname, "cloud%03d", i);
            transformPointCloud(*clouds[i], *clouds[i], cumxforms[i]);
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
    if (outputprefix.length() > 0) {
        PLYWriter w;
        char fname[20];
        for (int i = 0; i < clouds.size(); ++i) {
            sprintf(fname, (outputprefix + ".ply").c_str(), i);
            w.write<Pt>(fname, *clouds[i], true, false);
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
