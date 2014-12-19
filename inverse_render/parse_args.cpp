#include "parse_args.h"
#include <pcl/console/parse.h>

using namespace pcl;
using namespace std;

bool all_cameras = true;
bool all_project = true;
bool project_debug = false;
bool show_frustrum = false;
bool use_confidence_files = false;
bool wallinput = false;
bool write_eq = false;
bool write_matlab = false;
bool read_eq = false;
int project;
int camera;
string outfile, infile, camfile, walloutfile, wallfile, samplefile, sampleoutfile;
string coloredfile = "";
string radfile = "";
string plyfile = "";
string pbrtfile = "";
string texfile = "texture.exr";
string matlabsamplefile = "samples.m";
int wallthreshold = 200;
int numsamples = 100;
int minlightsize = 100;
double discardthreshold = 0.25;
bool output_reprojection = false;
bool output_wall = false;
bool input = false;
bool display = true;
bool prune = false;
bool ccw = false;
bool flipfloorceiling = false;
double anglethreshold = M_PI/40;
double resolution = 0.01;
double minlength = 0.2;
bool do_wallfinding = true;
bool do_reprojection = true;
bool do_sampling = true;
bool do_texture = true;
bool do_linefinding = false;
double hdr_threshold = -1.;
double displayscale = 2.;
int hemicuberesolution = 150;
bool image_flip_x = false;
bool image_flip_y = false;
int numRansacIters = 1000;
double maxPercentErr = 0.1;

bool parseargs(int argc, char** argv) {
    if (argc < 3) {
        printf(
             "Usage: invrender mesh.ply -camfile camera.cam [args]\n" \
             "  Argument types: file=filename (string), n=int, f=float\n" \
             "  File Arguments:\n" \
             "      -samplefile file: read sampled wall point equations from file\n"\
             "      -reprojectfile file: read sample info from file f\n" \
             "      -wallfile file: read wall info from file f\n" \
             "      -outputwallfile file: output wall labels and segments to\n" \
             "           the specified file f\n"
             "      -outputreprojectfile file: output sample info from reprojection\n" \
             "           to the specified file f\n"
             "      -outputsamplefile file: output sampled wall point\n" \
             "           equations to the specified file f\n" \
             "      -outputmatlabfile file: output sampled wall point\n" \
             "           equations in a format readable by matlab to the\n" \
             "           specified file f\n" \
             "      -radfile file: output geometry for RADIANCE to specified file\n" \
             "      -plyfile file: output geometry in PLY format to specified file\n" \
             "      -pbrtfile file: output geometry in PBRT format to specified file\n" \
             "      -texfile file: output floor texture as EXR file to specified file\n" \
             "      -outputcoloredmesh file: output a PLY mesh colored with \n" \
             "           the reprojected images\n" \
             "  Partial Execution Arguments:\n" \
             "      -wallfind_only: only perform wallfinding\n" \
             "      -reproject_only: only perform reprojection\n" \
             "      -solve_only: only perform solving; requires -samplefile\n" \
             "      -nosolve: do not perform solving (default off)\n" \
             "      -notexture: do not perform texture recovery (default off)\n" \
             "  Display Arguments:\n" \
             "      -nodisplay: exit immediately\n"
             "      -prune_occluded: Do not display points for which there\n" \
             "           are no samples\n" \
             "      -no_cameras: don't draw cameras (default draw all cameras)\n" \
             "      -show_camera n: draw only camera n (default draw all cameras)\n" \
             "      -show_frustrum: with show_camera, draw camera n frustrum\n" \
             "           (default off)\n"
             "      -project_status: with project, show reprojection debugging\n" \
             "           results, e.g occluded vertices and those not in view\n"\
             "      -display_scale f: Scale displayed color values by the\n"\
             "           given amount, useful for HDR viewing\n"\
             "  Reprojection Arguments:\n" \
             "      -project n: project only camera n\n"\
             "      -flip_x: Mirror the input images horizontally\n"\
             "      -flip_y: Mirror the input images vertically\n"\
             "      -use_confidence_files: Input files have a per-pixel confidence\n"\
             "           estimate, stored in a separate file. If the image filename\n"\
             "           is image.exr, the confidence is in image.exr.conf.\n"\
             "           These conf files should be binary encoded simply\n"\
             "           containing 4-byte floating point confidence values\n"\
             "      -hdr_threshold f: radiance threshold above which pixels\n"\
             "           are considered light pixels (default 10.0)\n"\
             "      -min_light_size n: Minimum number of vertices in a light\n"\
             "  Wallfinder Arguments:\n" \
             "      -ccw: Face vertices in counterclockwise direction (flip normals)\n" \
             "      -wallfinder_anglethreshold f: Angle between normals to be\n" \
             "           considered equal, for wallfinding (default PI/40)\n" \
             "      -wallfinder_min_wall_length f: Minimum length of a wall\n" \
             "           (default 0.2)\n" \
             "      -wallfinder_resolution f: maximum distance for a point to\n" \
             "           be considered on a plane for wallfinder (default 0.01)\n" \
             "      -wallfinder_flip_floor_ceiling: Reverse the floor and ceiling\n" \
             "      -wallfinder_wallthreshold n: Minimum bucket count in\n" \
             "           histogram to count as a wall; dependent on resolution!\n" \
             "           (default 200)\n" \
             "  Solver Arguments:\n" \
             "      -solver_numsamples n: number of wall points to sample for\n" \
             "           final solution (default 100) \n" \
             "      -solver_hemicuberesolution n: resolution of hemicube (default 150)\n" \
             "      -solver_threshold f: maximum unsampled proportion of\n" \
             "           hemisphere (default 0.25) \n" \
             "      -solver_ransac_iterations n: number of iterations of RANSAC to\n" \
             "           run (default 1000) \n" \
             "      -solver_ransac_percent_err f: Maximum percent error in calculated\n" \
             "           radiosity vs. measured radiosity for a point to count as an\n" \
             "           inlier (default 0.1)\n" \
                );
        return 0;
    }
    // Parse arguments
    if (console::find_switch(argc, argv, "-ccw")) ccw = true;
    if (console::find_switch(argc, argv, "-show_frustrum")) show_frustrum = true;
    if (console::find_switch(argc, argv, "-project_status")) project_debug = true;
    if (console::find_switch(argc, argv, "-nodisplay")) display = false;
    if (console::find_switch(argc, argv, "-prune_occluded")) prune = true;
    if (console::find_switch(argc, argv, "-nosolve")) do_sampling = false;
    if (console::find_switch(argc, argv, "-notexture")) do_texture = false;
    if (console::find_switch(argc, argv, "-do_linefinding")) do_linefinding = true;
    if (console::find_switch(argc, argv, "-flip_x")) image_flip_x = true;
    if (console::find_switch(argc, argv, "-flip_y")) image_flip_x = true;
    if (console::find_switch(argc, argv, "-wallfinder_flip_floor_ceiling")) flipfloorceiling = true;
    if (console::find_switch(argc, argv, "-use_confidence_files")) use_confidence_files = true;
    if (console::find_switch(argc, argv, "-wallfind_only")) {
        do_reprojection = false;
        do_wallfinding = true;
        do_sampling = false;
    }
    if (console::find_switch(argc, argv, "-reproject_only")) {
        if (!do_reprojection) {
            cerr << "Can only select on of wallfind_only and reproject_only" << endl;
            return false;
        }
        do_reprojection = true;
        do_wallfinding = false;
        do_sampling = false;
    }
    if (console::find_switch(argc, argv, "-prune_occluded")) prune = true;
    if (console::find_switch(argc, argv, "-no_cameras")) {
        all_cameras = false;
        camera = -1;
    }
    if (console::find_argument(argc, argv, "-hdr_threshold") >= 0) {
        console::parse_argument(argc, argv, "-hdr_threshold", hdr_threshold);
    }
    if (console::find_argument(argc, argv, "-min_light_size") >= 0) {
        console::parse_argument(argc, argv, "-min_light_size", minlightsize);
    }
    if (console::find_argument(argc, argv, "-show_camera") >= 0) {
        console::parse_argument(argc, argv, "-show_camera", camera);
        all_cameras = false;
    }
    if (console::find_argument(argc, argv, "-project") >= 0) {
        console::parse_argument(argc, argv, "-project", project);
        all_project = false;
    }
    if (console::find_argument(argc, argv, "-wallfile") >= 0) {
        console::parse_argument(argc, argv, "-wallfile", wallfile);
        wallinput = true;
    }
    if (console::find_argument(argc, argv, "-reprojectfile") >= 0) {
        console::parse_argument(argc, argv, "-reprojectfile", infile);
        input = true;
    }
    if (console::find_argument(argc, argv, "-outputreprojectfile") >= 0) {
        console::parse_argument(argc, argv, "-outputreprojectfile", outfile);
        if (input) cerr << "Warning: output file specified with sample input; ignoring output file" << endl;
        else if (!do_reprojection) cerr << "Error: Reprojection output file specified but reprojection not being performed!" << endl;
        else output_reprojection = true;
    }
    if (console::find_argument(argc, argv, "-outputwallfile") >= 0) {
        console::parse_argument(argc, argv, "-outputwallfile", walloutfile);
        if (!do_wallfinding) cerr << "Error: Wall output file specified but wallfinding not being performed!" << endl;
        else output_wall = true;
    }
    if (console::find_argument(argc, argv, "-outputsamplefile") >= 0) {
        console::parse_argument(argc, argv, "-outputsamplefile", sampleoutfile);
        write_eq = true;
    }
    if (console::find_argument(argc, argv, "-samplefile") >= 0) {
        console::parse_argument(argc, argv, "-samplefile", samplefile);
        read_eq = true;
    }
    if (console::find_argument(argc, argv, "-radfile") >= 0) {
        console::parse_argument(argc, argv, "-radfile", radfile);
    }
    if (console::find_argument(argc, argv, "-plyfile") >= 0) {
        console::parse_argument(argc, argv, "-plyfile", plyfile);
    }
    if (console::find_argument(argc, argv, "-pbrtfile") >= 0) {
        console::parse_argument(argc, argv, "-pbrtfile", pbrtfile);
    }
    if (console::find_argument(argc, argv, "-texfile") >= 0) {
        console::parse_argument(argc, argv, "-texfile", texfile);
    }
    if (console::find_argument(argc, argv, "-outputcoloredmesh") >= 0) {
        console::parse_argument(argc, argv, "-outputcoloredmesh", coloredfile);
    }
    if (console::find_argument(argc, argv, "-outputmatlabfile") >= 0) {
        console::parse_argument(argc, argv, "-outputmatlabfile", matlabsamplefile);
        write_matlab = true;
    }
    if (console::find_argument(argc, argv, "-camfile") >= 0) {
        console::parse_argument(argc, argv, "-camfile", camfile);
    } else {
        cerr << "Error: No camera file specified" << endl;
        return false;
    }
    if (console::find_argument(argc, argv, "-display_scale") >= 0) {
        console::parse_argument(argc, argv, "-display_scale", displayscale);
    }
    if (console::find_argument(argc, argv, "-wallfinder_anglethreshold") >= 0) {
        if (!do_wallfinding) cerr << "Warning: ignoring wallfinder parameters" << endl;
        console::parse_argument(argc, argv, "-wallfinder_anglethreshold", anglethreshold);
    }
    if (console::find_argument(argc, argv, "-wallfinder_resolution") >= 0) {
        if (!do_wallfinding) cerr << "Warning: ignoring wallfinder parameters" << endl;
        console::parse_argument(argc, argv, "-wallfinder_resolution", resolution);
    }
    if (console::find_argument(argc, argv, "-wallfinder_min_wall_length") >= 0) {
        if (!do_wallfinding) cerr << "Warning: ignoring wallfinder parameters" << endl;
        console::parse_argument(argc, argv, "-wallfinder_min_wall_length", minlength);
    }
    if (console::find_argument(argc, argv, "-wallfinder_wallthreshold") >= 0) {
        if (!do_wallfinding) cerr << "Warning: ignoring wallfinder parameters" << endl;
        console::parse_argument(argc, argv, "-wallfinder_wallthreshold", wallthreshold);
    }
    if (console::find_argument(argc, argv, "-solver_numsamples") >= 0) {
        if (!do_sampling) cerr << "Warning: ignoring solver parameters" << endl;
        console::parse_argument(argc, argv, "-solver_numsamples", numsamples);
    }
    if (console::find_argument(argc, argv, "-solver_threshold") >= 0) {
        if (!do_sampling) cerr << "Warning: ignoring solver parameters" << endl;
        console::parse_argument(argc, argv, "-solver_threshold", discardthreshold);
    }
    if (console::find_argument(argc, argv, "-solver_hemicuberesolution") >= 0) {
        if (!do_sampling) cerr << "Warning: ignoring solver parameters" << endl;
        console::parse_argument(argc, argv, "-solver_hemicuberesolution", hemicuberesolution);
    }
    if (console::find_argument(argc, argv, "-solver_ransac_iterations") >= 0) {
        if (!do_sampling) cerr << "Warning: ignoring solver parameters" << endl;
        console::parse_argument(argc, argv, "-solver_ransac_iterations", numRansacIters);
    }
    if (console::find_argument(argc, argv, "-solver_ransac_percent_err") >= 0) {
        if (!do_sampling) cerr << "Warning: ignoring solver parameters" << endl;
        console::parse_argument(argc, argv, "-solver_ransac_percent_err", maxPercentErr);
    }
    return true;
}
