#include <iostream>
#include <sstream>
#include <queue>
#include <pcl/io/openni2_grabber.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace pcl;
using namespace openni_wrapper;
using namespace io;

const double PI = 3.1416;
const int windowsize = 20;

class Segmentation {
    protected:
        string getFocalLengthText() {
            stringstream s;
            s << "Focal Length: " << focal_length;
            return s.str();
        }
    public:
        // Constructors/destructors
        Segmentation () : total(0), focal_length(575), viewer("Point Cloud"){
            calculated.reset(new PointCloud<PointXYZRGB>());
            viewer.setBackgroundColor(0,0,0);
            viewer.addPointCloud<PointXYZRGB>(calculated, "Plane fit");
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Plane fit");
            viewer.addText("", 20, 520, "windowavg");
            viewer.addText("", 360, 20, "angles");
            viewer.addText(getFocalLengthText(), 20, 20, "focallength");
            viewer.initCameraParameters();
            viewer.setCameraPosition(0,0,0,0,0,1,0,-1,0);
        }
        ~Segmentation() {
        }
        // Focal length control
        void kbd_cb_(const visualization::KeyboardEvent& event, void*) {
            if (event.keyDown()) {
                if (event.getKeyCode() == ',') {
                    focal_length -= 2.5;
                    grabber->setDepthFocalLength(focal_length);
                    viewer.updateText(getFocalLengthText(), 20, 20, "focallength");
                } else if (event.getKeyCode() == '.') {
                    focal_length += 2.5;
                    grabber->setDepthFocalLength(focal_length);
                    viewer.updateText(getFocalLengthText(), 20, 20, "focallength");
                }
            }
        }
        void cloud_cb_(const PointCloud<PointXYZ>::ConstPtr &cloud) {
            const double dist = 0.001;
            if (!viewer.wasStopped()) {
                viewer.removeAllPointClouds();
                ModelCoefficients coefficients[6];
                pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                // Create the segmentation object
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                // Optional
                seg.setOptimizeCoefficients (true);
                // Mandatory
                seg.setModelType (pcl::SACMODEL_PLANE);
                seg.setMethodType (pcl::SAC_RANSAC);
                seg.setDistanceThreshold (0.01);
                ExtractIndices<PointXYZ> extract;
                int i = 0;
                int nr_points = cloud->points.size();
                PointCloud<PointXYZ>::Ptr segmented_cloud(new PointCloud<PointXYZ>(*cloud));
                PointCloud<PointXYZ>::Ptr swap_cloud(new PointCloud<PointXYZ>());
                float r[] = {1.,0.,0.,1.,1.,0};
                float g[] = {0.,1.,0.,1.,0.,1};
                float b[] = {0.,0.,1.,0.,1.,1};
                char* name[] = {"red", "green", "blue", "yellow", "magenta", "cyan"};
                bool err = false;
                while (segmented_cloud->points.size() > 0.2*nr_points) {
                    seg.setInputCloud (segmented_cloud);
                    seg.segment (*inliers, coefficients[i]);
                    if (inliers->indices.size () == 0) {
                        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
                        err = true;
                        break;
                    }
                    //cerr << "Model coefficients " << i << ": ";
                    //for (int j = 0; j < coefficients[i].values.size(); ++j) {
                        //cout << coefficients[i].values[j] << " ";
                    //}
                    //cout << endl;

                    extract.setInputCloud(segmented_cloud);
                    extract.setIndices(inliers);
                    extract.setNegative(false);
                    extract.filter(*swap_cloud);
                    viewer.addPointCloud(swap_cloud, name[i]);
                    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, r[i], g[i], b[i], name[i]);
                    extract.setNegative(true);
                    extract.filter(*swap_cloud);
                    segmented_cloud.swap(swap_cloud);
                    i++;
                }
                if (!err) {
                    stringstream currtxt;
                    stringstream avgtxt;
                    // Calculate angles between normals
                    for (int j  = 0; j < i-1; ++j) {
                        double dot = 0;
                        for (int k = 0; k < 3; ++k)
                            dot += coefficients[j].values[k]*coefficients[j+1].values[k];
                        double res = acos(dot);
                        if (res < 0) res += 2*PI;
                        if (res > PI/2) res = PI - res;
                        if (res < PI/4) continue;
                        angles.push(res);
                        total += res;
                        if (angles.size() > windowsize) {
                            double tmp = angles.front();
                            angles.pop();
                            total -= tmp;
                        }
                        currtxt << res << ",";
                    }
                    if (i > 1)
                        viewer.updateText(currtxt.str(), 360, 20, "angles");
                    avgtxt << "Average: " << total/angles.size();
                    viewer.updateText(avgtxt.str(), 20, 320, "windowavg");
                }

                viewer.spinOnce();
            }
        }

        void run() {
            grabber = new OpenNIGrabber();
            viewer.registerKeyboardCallback(&Segmentation::kbd_cb_, *this);

            boost::function<void (const PointCloud<PointXYZ>::ConstPtr&)> f =
                boost::bind(&Segmentation::cloud_cb_, this, _1);
            grabber->registerCallback(f);
            grabber->setDepthFocalLength(focal_length);
            grabber->start();
            grabber->getDevice()->setExposure(70);
            grabber->getDevice()->setGain(900);
            while (!viewer.wasStopped()) {
                //PointCloud<PointXYZ>::ConstPtr cloud;
                //if (image_mutex_.try_lock()) {
                boost::this_thread::sleep(boost::posix_time::seconds(1));
            }
            grabber->stop();
        }


    private:
        PointCloud<PointXYZRGB>::Ptr calculated;
        OpenNIGrabber* grabber;
        visualization::PCLVisualizer viewer;
        PointCloud<PointXYZ>::ConstPtr cloud_;
        boost::mutex cloud_mutex_;
        double focal_length;

        queue<double> angles;
        double total;
};

int main(int argc, char** argv) {
    Segmentation tp;
    tp.run();
    return 0;
}
