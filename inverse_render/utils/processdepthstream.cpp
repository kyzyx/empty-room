#include <stdio.h>
#include <vector>


float tmp[640*480*3];
float transformed[640*480*3];
int indices[640*480];

class PoseMatrix {
    public:
        PoseMatrix() {}
        float m[12];
};
std::vector<PoseMatrix> poses;

float dot(float* a, float* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
int main(int argc, char** argv) {
    if (argc < 3) {
        printf("Usage: processdepth inputpts.pts output.txt\n");
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
    FILE* out = fopen(argv[2], "w");
    fprintf(out, "ply\nformat binary_little_endian 1.0\n");
    fprintf(out, "element vertex %d\n", n);
    fprintf(out, "property float x\nproperty float y\nproperty float z\n");
    fprintf(out, "end_header\n");
    for (int i = 0; i < poses.size(); i++) {
        fread(&ts, sizeof(double), 1, in);
        if (ts < 0) break;
        fread(&m, sizeof(int), 1, in);
        m /= 3;
        fread(&w, sizeof(int), 1, in);
        fread(&h, sizeof(int), 1, in);
        fread(tmp, sizeof(float), 3*m, in);
        fread(indices, sizeof(int), w*h, in);
        for (int j = 0; j < m; j++) {
            transformed[3*j+0] = dot(poses[i].m,   tmp+3*j) + poses[i].m[3];
            transformed[3*j+1] = dot(poses[i].m+4, tmp+3*j) + poses[i].m[7];
            transformed[3*j+2] = dot(poses[i].m+8, tmp+3*j) + poses[i].m[11];
        }
        fwrite(transformed, sizeof(float), 3*m, out);
    }
    fclose(in);
    fclose(out);
}
