#include "parse_args.h"
#include <pcl/console/parse.h>

using namespace pcl;
using namespace std;

bool all_cameras = true;
bool all_project = true;
bool project_debug = false;
bool show_frustrum = false;
bool wallinput = false;
bool write_eq = false;
bool read_eq = false;
int project;
int camera;
string outfile, infile, camfile, walloutfile, wallfile, imagelist, lightimagelist, samplefile, sampleoutfile;
int numImageListFiles = 0;
int wallthreshold = 200;
int numsamples = 100;
double discardthreshold = 0.25;
bool output_reprojection = false;
bool output_wall = false;
bool input = false;
bool display = true;
bool prune = false;
bool ccw = false;
double anglethreshold = M_PI/40;
double resolution = 0.01;
double minlength = 0.2;
bool do_wallfinding = true;
bool do_reprojection = true;
bool do_sampling = true;

bool parseargs(int argc, char** argv) {
    if (argc < 3) {
        printf(
             "Usage: invrender mesh.ply -camfile camera.cam [args]\n" \
             "  Must have (imagelist and lightimagelist) OR samplefile\n" \
             "  Argument types: file=filename (string), n=int, f=float\n" \
             "  File Arguments:\n" \
             "      -imagelist file: name of file specifying a list of color\n"\
             "           images taken from the camera positions in camfile\n" \
             "           Must be specified with lightimagelist\n" \
             "      -lightimagelist file: name of file specifying a list of light\n"\
             "           images taken from the camera positions in camfile\n" \
             "           Must be specified with imagelist\n" \
             "      -samplefile file: read sampled wall point equations from file\n"\
             "      -reprojectfile file: read sample info from file f\n" \
             "      -wallfile file: read wall info from file f\n" \
             "      -outputwallfile file: output wall labels and segments to\n" \
             "           the specified file f\n"
             "      -outputreprojectfile file: output sample info from reprojection to\n" \
             "           the specified file f\n"
             "      -outputsamplefile file: output sampled wall point\n" \
             "           equations to the specified file f\n"
             "  Partial Execution Arguments:\n" \
             "      -wallfind_only: only perform wallfinding\n" \
             "      -reproject_only: only perform reprojection\n" \
             "      -solve_only: only perform solving; requires -samplefile\n" \
             "      -nosolve: do not perform solving (default off)\n" \
             "  Display Arguments:\n" \
             "      -nodisplay: exit immediately\n"
             "      -prune_occluded: Do not display points for which there\n" \
             "           are no samples\n" \
             "      -no_cameras: don't draw cameras (default draw all cameras)\n" \
             "      -show_camera n: draw only camera n (default draw all cameras)\n" \
             "      -show_frustrum: with show_camera, draw camera n frustrum\n" \
             "           (default off)\n"
             "      -project_status: with project, show reprojection debugging\n" \
             "           results, e.g occluded vertices and those not in view\n"
             "  Reprojection Arguments:\n" \
             "      -project n: project only camera n\n"
             "  Wallfinder Arguments:\n" \
             "      -ccw: Faces in counterclockwise direction (flip normals)\n" \
             "      -wallfinder_anglethreshold f: Angle between normals to be\n" \
             "           considered equal, for wallfinding(default PI/40)\n" \
             "      -wallfinder_min_wall_length f: Minimum length of a wall\n" \
             "           (default 0.2)\n" \
             "      -wallfinder_resolution f: maximum distance for a point to\n" \
             "           be considered on a plane for wallfinder (default 0.01)\n" \
             "      -wallfinder_wallthreshold n: Minimum bucket count in\n" \
             "           histogram to count as a wall; dependent on resolution!\n" \
             "           (default 200)\n" \
             "  Solver Arguments:\n" \
             "      -solver_numsamples n: number of wall points to sample for\n" \
             "           final solution (default 100) \n" \
             "      -solver_threshold f: maximum unsampled proportion of\n" \
             "           hemisphere (default 0.25) \n" \
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
    if (console::find_switch(argc, argv, "-wallfind_only")) {
        do_reprojection = false;
        do_wallfinding = true;
    }
    if (console::find_switch(argc, argv, "-reproject_only")) {
        if (!do_reprojection) {
            cerr << "Can only select on of wallfind_only and reproject_only" << endl;
            return false;
        }
        do_reprojection = true;
        do_wallfinding = false;
    }
    if (console::find_switch(argc, argv, "-prune_occluded")) prune = true;
    if (console::find_switch(argc, argv, "-no_cameras")) {
        all_cameras = false;
        camera = -1;
    }
    if (console::find_argument(argc, argv, "-show_camera") >= 0) {
        console::parse_argument(argc, argv, "-show_camera", camera);
        all_cameras = false;
    }
    if (console::find_argument(argc, argv, "-project") >= 0) {
        console::parse_argument(argc, argv, "-project", project);
        all_project = false;
    }
    if (console::find_argument(argc, argv, "-imagelist") >= 0) {
        console::parse_argument(argc, argv, "-imagelist", imagelist);
        numImageListFiles++;
    }
    if (console::find_argument(argc, argv, "-lightimagelist") >= 0) {
        console::parse_argument(argc, argv, "-lightimagelist", lightimagelist);
        numImageListFiles++;
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
    if (console::find_argument(argc, argv, "-camfile") >= 0) {
        console::parse_argument(argc, argv, "-camfile", camfile);
    } else {
        cerr << "Error: No camera file specified" << endl;
        return false;
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
    if (!input && numImageListFiles != 2) {
        cerr << "Error: Must specify either a reprojectfile or two image lists!" << endl;
        return false;
    }
    return true;
}
