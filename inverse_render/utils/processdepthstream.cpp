#include <stdio.h>

float tmp[640*480*3];
float transformed[640*480*3];
int indices[640*480];
float mat[12];

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
    while (true) {
        fread(&m, sizeof(int), 1, in);
        m /= 3;
        if (feof(in)) break;
        fread(&w, sizeof(int), 1, in);
        fread(&h, sizeof(int), 1, in);

        fread(mat, sizeof(float), 12, in);
        fread(tmp, sizeof(float), 3*m, in);
        fread(indices, sizeof(int), w*h, in);
        n += m;
    }
    rewind(in);
    printf("Writing %d points\n", n);
    FILE* out = fopen(argv[2], "w");
    fprintf(out, "ply\nformat binary_little_endian 1.0\n");
    fprintf(out, "element vertex %d\n", n);
    fprintf(out, "property float x\nproperty float y\nproperty float z\n");
    fprintf(out, "end_header\n");
    while (true) {
        fread(&m, sizeof(int), 1, in);
        m /= 3;
        if (feof(in)) break;
        fread(&w, sizeof(int), 1, in);
        fread(&h, sizeof(int), 1, in);

        fread(mat, sizeof(float), 12, in);
        fread(tmp, sizeof(float), 3*m, in);
        fread(indices, sizeof(int), w*h, in);
        for (int i = 0; i < m; i++) {
            transformed[3*i+0] = dot(mat,   tmp+3*i) + mat[3];
            transformed[3*i+1] = dot(mat+4, tmp+3*i) + mat[7];
            transformed[3*i+2] = dot(mat+8, tmp+3*i) + mat[11];
        }
        fwrite(transformed, sizeof(float), 3*m, out);
    }
    fclose(in);
    fclose(out);
}
