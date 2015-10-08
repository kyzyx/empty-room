#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include "yuv.h"

using namespace std;
unsigned char tmp[1280*720*3];
unsigned char yuv[1280*720*3/2];
int main(int argc, char** argv) {
    if (argc < 3) {
        printf("Usage: ./split filename.bin imageposes.xforms outputfmtstring.bin\n");
        return 0;
    } else {
        ifstream in(argv[1], ios::in | ios::binary);
        ifstream posein(argv[2], ios::in | ios::binary);
        string header;
        string line;
        getline(in, line); // ImageData
        getline(in, line);

        bool bgr = false;
        int bpp = 0;
        int w, h;

        while (line != "end_header") {
            int n = line.find(' ');
            if (n != string::npos) {
                string key = line.substr(0, n);
                if (key == "width") w = atoi(line.substr(n+1).c_str());
                else if (key == "height") h = atoi(line.substr(n+1).c_str());
            }
            header = header + line + "\n";
            getline(in, line);
        }

        int count = 0;
        while(true) {
            double ts;
            in.read((char*) &ts, sizeof(double));
            if (ts < 0) break;
            char filename[40];
            sprintf(filename, argv[3], count);
            FILE* out = fopen(filename, "w");
            fprintf(out, "ImageData\ntimestamp %f\n%s", ts, header.c_str());
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) {
                    float f;
                    char ch;
                    int n;
                    posein.read((char*) &f, sizeof(float));
                    if (j == 3) {
                        ch = 'T';
                        n = i;
                    }
                    else {
                        ch = 'R';
                        n = i*3+j;
                    }
                    fprintf(out, "%c%d %f\n", ch, n, f);
                }
            }
            fprintf(out, "end_header\n");
            in.read((char*)yuv, w*h*3/2);
            nv21_to_rgb(tmp, yuv, w, h);
            memcpy(tmp, tmp+w*3, w*3);
            fwrite(tmp, 3, w*h, out);
            fclose(out);
            printf("Wrote %d x %d file %s timestamp %f \n", w, h, filename, ts);
            count++;
        }
    }
}
