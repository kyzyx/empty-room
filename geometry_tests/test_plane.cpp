#include <iostream>
#include <pcl/io/openni2_grabber.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter_indices.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace pcl;
using namespace openni_wrapper;
using namespace io;

class TestPlane {
    public:
        // Constructors/destructors
        TestPlane () : dist(0.001), mult(20), focal_length(575), viewer("Point Cloud"){
            calculated.reset(new PointCloud<PointXYZRGB>());
            viewer.setBackgroundColor(0,0,0);
            viewer.addPointCloud<PointXYZRGB>(calculated, "Plane fit");
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Plane fit");
            viewer.initCameraParameters();
            viewer.setCameraPosition(0,0,0,0,0,1,0,-1,0);
        }
        TestPlane (bool pause) : dist(0.001), mult(20), focal_length(575), viewer("Point Cloud"), pause(pause){
            calculated.reset(new PointCloud<PointXYZRGB>());
            viewer.setBackgroundColor(0,0,0);
            viewer.addPointCloud<PointXYZRGB>(calculated, "Plane fit");
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Plane fit");
            viewer.initCameraParameters();
            viewer.setCameraPosition(0,0,0,0,0,1,0,-1,0);
        }
        ~TestPlane() {
        }
        // Point coordinate debugging
        void pp_cb_(const visualization::PointPickingEvent& event, void*) {
            int i = event.getPointIndex();
            if (i == -1) return;
            float x,y,z;
            event.getPoint(x,y,z);
            double distance = plane[0]*x + plane[1]*y + plane[2]*z + plane[3];
            distance /= sqrt(x*x+y*y+z*z);
            cerr << x << "," << y << "," << z << " " << distance;
            cerr << "   " << i << ":"
                               << calculated->points[i].x << ","
                               << calculated->points[i].y << ","
                               << calculated->points[i].z << ";"
                               << (int)calculated->points[i].r << ","
                               << (int)calculated->points[i].g << ","
                               << (int)calculated->points[i].b << endl;
        }
        // Focal length control
        void kbd_cb_(const visualization::KeyboardEvent& event, void*) {
            if (event.keyDown()) {
                if (event.getKeyCode() == ',') {
                    focal_length -= 2.5;
                    grabber->setDepthFocalLength(focal_length);
                    cerr << "Focal length: " << focal_length << endl;
                } else if (event.getKeyCode() == '.') {
                    focal_length += 2.5;
                    grabber->setDepthFocalLength(focal_length);
                    cerr << "Focal length: " << focal_length << endl;
                } else if (event.getKeyCode() == '[') {
                    mult -= 10;
                    cerr << "Multiplier " << mult << endl;
                } else if (event.getKeyCode() == ']') {
                    mult += 10;
                    cerr << "Multiplier " << mult << endl;
                }
            }
        }
        void cloud_cb_(const PointCloud<PointXYZ>::ConstPtr &cloud) {
            static int done = 0;
            if (!viewer.wasStopped()) {
                if (!(done&1) && (!pause || done < 5)) {
                    // Do RANSAC plane finding
                    SampleConsensusModelPlane<PointXYZ>::Ptr model(
                            new SampleConsensusModelPlane<PointXYZ>(cloud));
                    RandomSampleConsensus<PointXYZ> ransac(model);
                    ransac.setDistanceThreshold(dist);
                    ransac.computeModel();
                    // Extract point distances
                    vector<double> distances;
                    vector<int> inliers;
                    ransac.getModelCoefficients(plane);
                    if (plane[3] < 0) {
                        for (int i = 0; i < 4; ++i) plane[i] = -plane[i];
                    }
                    model->getDistancesToModel(plane, distances);
                    ransac.getInliers(inliers);
                    calculated->points.resize(cloud->size());
                    calculated->is_dense = false;
                    if (pause && done == 4)
                        cout << plane[0] << " " << plane[1] << " " << plane[2] << " " << plane[3] << endl << endl;
                    for (size_t i = 0; i < cloud->points.size(); ++i) {
                        calculated->points[i].x = cloud->points[i].x;
                        calculated->points[i].y = cloud->points[i].y;
                        calculated->points[i].z = cloud->points[i].z;
                    }
                    vector<int> nanidx;
                    removeNaNFromPointCloud(*calculated, nanidx);
                }
                // Color based on distance
                int z = 0;
                for (size_t i = 0; i < calculated->points.size(); ++i) {
                    double x = calculated->points[i].x;
                    double y = calculated->points[i].y;
                    double z = calculated->points[i].z;

                    double distance = plane[0]*x + plane[1]*y + plane[2]*z + plane[3];
                    distance /= sqrt(x*x+y*y+z*z);
                    if (pause && done == 4) cout << calculated->points[i].x << "," << calculated->points[i].y << "," << calculated->points[i].z << ": " << distance << endl;
                    if (abs(distance) > 20*dist) {
                        calculated->points[i].r = 0;
                        calculated->points[i].g = 0;
                        calculated->points[i].b = 255;
                    } else {
                        if (distance < 0) {
                            calculated->points[i].b = 255;
                        }
                        else {
                            calculated->points[i].b = 0;
                        }
                        distance = abs(distance);
                        calculated->points[i].r = ((int)(mult*distance/dist))%256;
                        calculated->points[i].g = 255. - calculated->points[i].r;
                        /*if (inliers[z] == i) {
                            z++;
                            calculated->points[i].b = 255;
                        } else {
                            calculated->points[i].b = 0;
                        }*/
                        //calculated->points[i].b = (distance/(dist/10))/256;
                        //calculated->points[i].b = (distance/(dist/10))/256;
                    }
                }
                viewer.updatePointCloud(calculated, "Plane fit");
                // draw plane
                ModelCoefficients mc;
                mc.values.resize(4);
                mc.values[0] = plane[0];
                mc.values[1] = plane[1];
                mc.values[2] = plane[2];
                mc.values[3] = plane[3];
                viewer.removeShape("plane");
                viewer.addPlane(mc, 0,0,0,"plane");
                //viewer.setShapeRenderingProperties(visualization::PCL_VISUALIZER_REPRESENTATION, visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "plane");
                viewer.spinOnce();
                if (pause) ++done;
            }
        }

        void run() {
            grabber = new OpenNIGrabber();
            viewer.registerKeyboardCallback(&TestPlane::kbd_cb_, *this);

            boost::function<void (const PointCloud<PointXYZ>::ConstPtr&)> f =
                boost::bind(&TestPlane::cloud_cb_, this, _1);
            viewer.registerPointPickingCallback(&TestPlane::pp_cb_, *this, (void*) NULL);
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
        double dist;
        double mult;
        Eigen::VectorXf plane;
        bool pause;
};

int main(int argc, char** argv) {
    bool pause = false;
    if (argc > 1) pause = true;
    TestPlane tp(pause);
    tp.run();
    return 0;
}
