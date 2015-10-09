#include <stdio.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>

float tmp[640*480*3];
int indices[640*480];

class PoseMatrix {
    public:
        PoseMatrix() {}
        float m[12];
};
std::vector<PoseMatrix> poses;

using namespace pcl;
using namespace std;

float dot(float* a, float* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
int main(int argc, char** argv) {
    if (argc < 3) {
        printf("Usage: processdepth inputpts.pts output.ply\n");
        return 0;
    }
    FILE* in = fopen(argv[1], "r");
    int n = 0;
    int w, h;
    int m;
    int framecount = 0;
    double ts;
    while (true) {
        fread(&ts, sizeof(double), 1, in);
        if (ts < 0) break;
        fread(&m, sizeof(int), 1, in);
        m /= 3;
        fread(&w, sizeof(int), 1, in);
        fread(&h, sizeof(int), 1, in);

        fread(tmp, sizeof(float), 3*m, in);
        fread(indices, sizeof(int), w*h, in);
        n += m;
        framecount++;
    }
    for (int i = 0; i < framecount; i++) {
        PoseMatrix mat;
        fread(mat.m, sizeof(float), 12, in);
        poses.push_back(mat);
    }
    rewind(in);
    printf("Writing %d points\n", n);
    PointCloud<PointNormal>::Ptr fullcloud(new PointCloud<PointNormal>);
    PointCloud<PointXYZ>::Ptr allcloud(new PointCloud<PointXYZ>);
    for (int z = 0; z < poses.size(); z++) {
        fread(&ts, sizeof(double), 1, in);
        if (ts < 0) break;
        fread(&m, sizeof(int), 1, in);
        m /= 3;
        fread(&w, sizeof(int), 1, in);
        fread(&h, sizeof(int), 1, in);
        fread(tmp, sizeof(float), 3*m, in);
        fread(indices, sizeof(int), w*h, in);

        // -------------------------------------------------------
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
        PointCloud<PointNormal>::Ptr withnormals(new PointCloud<PointNormal>);
        cloud->resize(m);
        for (int j = 0; j < m; j++) {
            float v[3];
            v[0] = dot(poses[z].m,   tmp+3*j) + poses[z].m[3];
            v[1] = dot(poses[z].m+4, tmp+3*j) + poses[z].m[7];
            v[2] = dot(poses[z].m+8, tmp+3*j) + poses[z].m[11];
            cloud->at(j) = PointXYZ(v[0], v[1], v[2]);
        }

        NormalEstimation<PointXYZ, PointNormal> ne;
        ne.setInputCloud (cloud);
        search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.1);
        ne.compute (*withnormals);
        for (int i = 0; i < m; i++) {
            withnormals->at(i).x = cloud->at(i).x;
            withnormals->at(i).y = cloud->at(i).y;
            withnormals->at(i).z = cloud->at(i).z;
        }
        *fullcloud += *withnormals;
        printf("Done %d/%d\n", z, poses.size());
        // -------------------------------------------------------
        /*
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>(w, h));
        PointCloud<PointNormal>::Ptr withnormals(new PointCloud<PointNormal>);
        cloud->is_dense = false;
        printf("Size %d x %d and %d x %d\n", w, h, cloud->width, cloud->height);
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                int idx = i*w + j;
                int pid = indices[idx];
                if (pid < 0) {
                    cloud->at(j,i) = PointXYZ(NAN, NAN, NAN);
                } else {
                    float v[3];
                    v[0] = dot(mat,   tmp+3*pid) + mat[3];
                    v[1] = dot(mat+4, tmp+3*pid) + mat[7];
                    v[2] = dot(mat+8, tmp+3*pid) + mat[11];
                    cloud->at(j,i) = PointXYZ(v[0], v[1], v[2]);
                }
            }
        }
        IntegralImageNormalEstimation<PointXYZ, PointNormal> ne;
        ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.05f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(cloud);
        ne.compute(*withnormals);
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                withnormals->at(j,i).x = cloud->at(j,i).x;
                withnormals->at(j,i).y = cloud->at(j,i).y;
                withnormals->at(j,i).z = cloud->at(j,i).z;
            }
        }
        *fullcloud += *withnormals;*/
    }
    fclose(in);
    vector<int> tmp;
    removeNaNNormalsFromPointCloud(*fullcloud, *fullcloud, tmp);
        printf("Cleaned points %d\n", fullcloud->size());

    io::savePLYFileBinary<PointNormal>(argv[2], *fullcloud);
}
