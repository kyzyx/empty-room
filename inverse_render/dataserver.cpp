#include "imageserver.h"
#include "meshserver.h"
#include <unistd.h>
#include <string>
#include <cstdio>
#include <pcl/console/parse.h>

bool ccw = false;
bool image_flip_x = false;
bool image_flip_y = false;
std::string camfile, meshfile;

// TODO: Configure imagetypes to load
// TODO: Load labels/types/samples from file

bool parseargs(int argc, char** argv) {
    if (argc < 3) {
        printf(
         "Usage: dataserver mesh.ply camera.cam [args] &\n" \
         "   Arguments:\n" \
         "       -ccw: Face vertices in counterclockwise direction, i.e.\n" \
         "             flip normals (default off, i.e. clockwise)\n" \
         "       -flip_x: Mirror the input images horizontally\n"\
         "       -flip_y: Mirror the input images vertically\n"\
         );
        return false;
    }
    meshfile = argv[1];
    camfile = argv[2];
    if (pcl::console::find_switch(argc, argv, "-ccw")) ccw = true;
    if (pcl::console::find_switch(argc, argv, "-flip_x")) image_flip_x = true;
    if (pcl::console::find_switch(argc, argv, "-flip_y")) image_flip_x = true;
    return true;
}
int main(int argc, char* argv[]) {
    if (!parseargs(argc, argv)) return 1;
    printf("Loading images...\n");
    ImageServer imgr(camfile, image_flip_x, image_flip_y);
    printf("Done loading images.\n");
    printf("Loading geometry...\n");
    MeshServer mmgr(meshfile, ccw);
    printf("Done loading geometry.\n");
    while(1) {
        sleep(1);
    }
}
