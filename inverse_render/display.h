#ifndef _DISPLAY_H
#define _DISPLAY_H
#include "datamanager/meshmanager.h"
#include "datamanager/imagemanager.h"
#include "solver.h"
#include "wall_finder.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace pcl {
    namespace visualization {
    class ImageViewer;
    class PCLVisualizer;
    class KeyboardEvent;
    }
}
class InvRenderVisualizer {
    public:
        enum {
            LABEL_NONE,
            LABEL_REPROJECT_DEBUG,
            LABEL_LIGHTS,
            LABEL_AF,
        };

        InvRenderVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud,
                ImageManager& helper,
                WallFinder& wallfinder,
                InverseRender& invrender)
        {
            ch = &helper;
            wf = &wallfinder;
            ir = &invrender;
            cloud = pointcloud;
            previouscube = 0;
            currcube = 0;
            nextcubename = 0;
            x = 0;
            change = true;
            updatepointcloud = true;
            imvu = NULL;
            viewer = NULL;
            init();
        }

        void loop();
        void recalculateColors(int labeltype);
        void drawLine(int wallidx, double x, double r, double g, double b, double starty=0, double endy=1);

        void visualizeWalls();
        void addSamples(std::vector<SampleData>& data);

        void visualizeCameras(int cameraid);
        void visualizeCamera(
                const CameraParams* cam,
                std::string name, double f=0, bool axes=true);
    private:
        void init();

        // Callbacks
        void showimage(int n, int x);
        void pointcloud_kbd_cb_(const pcl::visualization::KeyboardEvent& event, void*);
        void kbd_cb_(const pcl::visualization::KeyboardEvent& event, void*);

        void VisualizeSamplePoint(MeshManager& m, SampleData& s);

        WallFinder* wf;
        InverseRender* ir;
        ImageManager* ch;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        int previouscube;
        int currcube;
        int nextcubename;
        int x;
        bool change;
        bool updatepointcloud;
        std::vector<SampleData> sampledata;
        pcl::visualization::ImageViewer* imvu;
        pcl::visualization::PCLVisualizer* viewer;
};
#endif
