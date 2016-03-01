#include <stdio.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
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

void writePlyFile(string filename, vector<float>& points, vector<float>& normals, vector<float>& scales) {
    FILE* out = fopen(filename.c_str(), "w");
    fprintf(out, "ply\nformat binary_little_endian 1.0\n");
    fprintf(out, "element vertex %lu\n", scales.size());
    fprintf(out, "property float x\n");
    fprintf(out, "property float y\n");
    fprintf(out, "property float z\n");
    fprintf(out, "property float nx\n");
    fprintf(out, "property float ny\n");
    fprintf(out, "property float nz\n");
    fprintf(out, "property float value\n");
    fprintf(out, "end_header\n");
    for (int i = 0; i < scales.size(); i++) {
        fwrite(&points[3*i+0], sizeof(float), 1, out);
        fwrite(&points[3*i+1], sizeof(float), 1, out);
        fwrite(&points[3*i+2], sizeof(float), 1, out);
        fwrite(&normals[3*i+0], sizeof(float), 1, out);
        fwrite(&normals[3*i+1], sizeof(float), 1, out);
        fwrite(&normals[3*i+2], sizeof(float), 1, out);
        fwrite(&scales[i], sizeof(float), 1, out);
    }
    fclose(out);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printf("Usage: processdepth inputpts.pts output.ply [-separate]\n");
        return 0;
    }
    bool singlefile = true;
    if (argc > 3 && string(argv[4]) == "-separate") singlefile = false;
    FILE* in = fopen(argv[1], "r");
    int n = 0;
    int w, h;
    int m;
    int framecount = 0;
    double ts;
    int bytesread;
    while (true) {
        bytesread = fread(&ts, sizeof(double), 1, in);
        if (ts < 0) break;
        bytesread = fread(&m, sizeof(int), 1, in);
        m /= 3;
        bytesread = fread(&w, sizeof(int), 1, in);
        bytesread = fread(&h, sizeof(int), 1, in);

        bytesread = fread(tmp, sizeof(float), 3*m, in);
        bytesread = fread(indices, sizeof(int), w*h, in);
        n += m;
        framecount++;
    }
    for (int i = 0; i < framecount; i++) {
        PoseMatrix mat;
        bytesread = fread(mat.m, sizeof(float), 12, in);
        poses.push_back(mat);
    }
    rewind(in);
    printf("Processing %d points\n", n);
    vector<float> scales;
    vector<float> points;
    vector<float> normals;
    for (int z = 0; z < poses.size(); z++) {
        bytesread = fread(&ts, sizeof(double), 1, in);
        if (ts < 0) break;
        bytesread = fread(&m, sizeof(int), 1, in);
        m /= 3;
        bytesread = fread(&w, sizeof(int), 1, in);
        bytesread = fread(&h, sizeof(int), 1, in);
        bytesread = fread(tmp, sizeof(float), 3*m, in);
        bytesread = fread(indices, sizeof(int), w*h, in);

        // -------------------------------------------------------
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
        PointCloud<Normal>::Ptr withnormals(new PointCloud<Normal>);
        cloud->resize(m);
        for (int j = 0; j < m; j++) {
            cloud->at(j) = PointXYZ(tmp[3*j], tmp[3*j+1], tmp[3*j+2]);
        }

        float radius = 0.1;
        NormalEstimation<PointXYZ, Normal> ne;
        ne.setInputCloud(cloud);
        search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(radius);
        ne.compute(*withnormals);
        if (!singlefile) {
            points.clear();
            normals.clear();
            scales.clear();
        }
        for (int i = 0; i < m; i++) {
            if (isFinite(cloud->at(i)) && isFinite(withnormals->at(i))) {
                float v[3];
                v[0] = cloud->at(i).x;
                v[1] = cloud->at(i).y;
                v[2] = cloud->at(i).z;
                points.push_back(dot(poses[z].m,   v) + poses[z].m[3]);
                points.push_back(dot(poses[z].m+4, v) + poses[z].m[7]);
                points.push_back(dot(poses[z].m+8, v) + poses[z].m[11]);
                v[0] = withnormals->at(i).normal_x;
                v[1] = withnormals->at(i).normal_y;
                v[2] = withnormals->at(i).normal_z;
                normals.push_back(dot(poses[z].m,   v));
                normals.push_back(dot(poses[z].m+4, v));
                normals.push_back(dot(poses[z].m+8, v));
                float scale = 0;
                int cnt = 0;
                vector<int> knnidx;
                vector<float> knnd;
                int n = tree->nearestKSearch(cloud->at(i), 4, knnidx, knnd);
                for (int j = 0; j < knnd.size(); j++) {
                    if (knnd[j] < radius*radius) {
                        scale += sqrt(knnd[j]);
                        ++cnt;
                    }
                }
                scales.push_back(cnt?scale/cnt:0);
            }
        }
        if (!singlefile) {
            char filename[40];
            sprintf(filename, argv[2], z);
            writePlyFile(filename, points, normals, scales);
        }
        printf("Done %d/%lu\n", z, poses.size());
        // -------------------------------------------------------
    }
    if (singlefile) {
        writePlyFile(argv[2], points, normals, scales);
    }
    fclose(in);
}
