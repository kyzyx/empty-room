#include "display.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
using namespace std;
using namespace pcl;

void VisualizeCamera(const CameraParams* cam, visualization::PCLVisualizer& viewer,
        string name, double f=0, bool axes=true);

void visualize(Mesh& m, PointCloud<PointXYZRGB>::Ptr cloud, ColorHelper& loader,
        bool show_frustrum, bool prune, bool all_cameras,
        int labeltype, int cameraid) {
    PointIndices::Ptr nonnull(new PointIndices());
    for (int i = 0; i < cloud->size(); ++i) {
        if (labeltype == LABEL_REPROJECT_DEBUG) {
            if (m.labels[i] == 3) {
                cloud->at(i).r = 255;
                cloud->at(i).g = 0;
                cloud->at(i).b = 255;
            } else if (m.labels[i] == 2) {
                cloud->at(i).r = 255;
                cloud->at(i).g = 0;
                cloud->at(i).b = 0;
            } else if (m.labels[i] == 4) {
                cloud->at(i).r = 255;
                cloud->at(i).g = 255;
                cloud->at(i).b = 255;
            } else if (m.labels[i] == 1) {
                cloud->at(i).r = 255;
                cloud->at(i).g = 255;
                cloud->at(i).b = 0;
            }
        } else {
            if (m.samples[i].size()) {
                if (labeltype == LABEL_LIGHTS && m.labels[i] > 0) {
                    cloud->at(i).r = 0;
                    cloud->at(i).g = 0;
                    cloud->at(i).b = 0;
                    if (m.labels[i]&1) cloud->at(i).r = 255;
                    if (m.labels[i]&2) cloud->at(i).g = 255;
                    if (m.labels[i]&4) cloud->at(i).b = 255;
                } else {
                    int r = 0;
                    int g = 0;
                    int b = 0;
                    for (int j = 0; j < m.samples[i].size(); ++j) {
                        r += m.samples[i][j].r;
                        g += m.samples[i][j].g;
                        b += m.samples[i][j].b;
                    }
                    cloud->at(i).r = r/m.samples[i].size();
                    cloud->at(i).g = g/m.samples[i].size();
                    cloud->at(i).b = b/m.samples[i].size();
                }
                nonnull->indices.push_back(i);
            } else {
                cloud->at(i).r = 100;
                cloud->at(i).g = 100;
                cloud->at(i).b = 100;
            }
        }
    }
    if (prune) {
        ExtractIndices<PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(nonnull);
        extract.setNegative(false);
        extract.filter(*cloud);
    }
    visualization::PCLVisualizer viewer("Cloud viewer");
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<PointXYZRGB>(cloud, rgb, "Mesh");
    if (all_cameras) {
        char n[] = {'A', '0'};
        for (int i = 0; i < loader.size(); i++) {
            VisualizeCamera(loader.getCamera(i), viewer, n);
            n[1]++;
        }
    } else if (cameraid >= 0) {
        VisualizeCamera(loader.getCamera(cameraid), viewer, "cam", show_frustrum?3:0);
    }

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::seconds(0.5));
    }
}

void VisualizeCamera(const CameraParams* cam, visualization::PCLVisualizer& viewer,
        string name, double f, bool axes)
{
    R3Point p = cam->pos;
    PointXYZ p1(cam->pos[0], cam->pos[1], cam->pos[2]);
    if (axes) {
        R3Point v2 = p + cam->towards*0.2;
        R3Point v3 = p + cam->up*0.2;
        PointXYZ p2(v2[0], v2[1], v2[2]);
        PointXYZ p3(v3[0], v3[1], v3[2]);
        viewer.addLine(p2, p1, 1, 0, 0, name+"towards");
        viewer.addLine(p3, p1, 0, 0, 1, name+"up");
    }
    if (f > 0) {
        PointCloud<PointXYZ>::Ptr left(new PointCloud<PointXYZ>());
        PointCloud<PointXYZ>::Ptr right(new PointCloud<PointXYZ>());
        PointCloud<PointXYZ>::Ptr up(new PointCloud<PointXYZ>());
        PointCloud<PointXYZ>::Ptr down(new PointCloud<PointXYZ>());
        double w = cam->width/(2*cam->focal_length);
        double h = cam->height/(2*cam->focal_length);

        R3Point ul = p + cam->towards*f - cam->right*f*w + cam->up*f*h;
        R3Point ur = p + cam->towards*f + cam->right*f*w + cam->up*f*h;
        R3Point ll = p + cam->towards*f - cam->right*f*w - cam->up*f*h;
        R3Point lr = p + cam->towards*f + cam->right*f*w - cam->up*f*h;
        PointXYZ ulp(ul[0], ul[1], ul[2]);
        PointXYZ urp(ur[0], ur[1], ur[2]);
        PointXYZ llp(ll[0], ll[1], ll[2]);
        PointXYZ lrp(lr[0], lr[1], lr[2]);
        up->push_back(p1);
        up->push_back(ulp);
        up->push_back(urp);
        down->push_back(p1);
        down->push_back(llp);
        down->push_back(lrp);
        right->push_back(p1);
        right->push_back(urp);
        right->push_back(lrp);
        left->push_back(p1);
        left->push_back(ulp);
        left->push_back(llp);
        right->push_back(p1);
        right->push_back(urp);
        right->push_back(lrp);

        viewer.addPolygon<PointXYZ>(PointCloud<PointXYZ>::ConstPtr(up), name+"up");
        viewer.addPolygon<PointXYZ>(PointCloud<PointXYZ>::ConstPtr(down), name+"down");
        viewer.addPolygon<PointXYZ>(PointCloud<PointXYZ>::ConstPtr(left), name+"left");
        viewer.addPolygon<PointXYZ>(PointCloud<PointXYZ>::ConstPtr(right), name+"right");
    }
}
