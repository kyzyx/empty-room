Empty Room Visualization Design Document
========================================

Components
----------
Basic view loads the mesh. Nontrivial processing components include
* Wallfinding, including orientationfinder
  * Outputs mesh labels and normals
  * Outputs floor plan and height
* Reprojection
  * Outputs mesh samples
* Scene Sampling
  * Outputs sampledata
* Inverse Rendering
  * Outputs final wall reflectance
  * Outputs floor texture
  * Outputs light intensities

Shared Data
-----------
Mesh data, including
* Mesh vertex locations and normals
* Mesh connectivity
* Per-vertex list of samples
* Per-vertex "labels" (i.e. floor, ceiling, etc.)
* Per-vertex "types" (originally light ids, now for utility)

Per-camera image data, including
* HDR color images
* Confidence maps
* Depth maps
* Axis-aligned edge filter results
* Label images
* Camera parameters

Visualization
-------------
Primary displays:
* A navigable 3D display based on the input mesh
* A 2D display for a particular selected camera or hemicube
* Also possible: a navigable 3D display based on the results

### 3D Display ###
* Basic display with no other data is a directionally-lit mesh
* Should display clickable camera positions, optionally displaying frusta
* Toggle display colors, including
  * Averaged samples per vertex
  * Samples from a particular reprojection
  * Confidence maps
  * Labels or types
  * Sample statistics, including density, variance, etc.
  * Directionality of samples
  * Some combination of the above
* HDR display (either tonemapped or configurable)
* Overlay detected floor plan somehow

Additional possibilities
* Show wallfinding debug info
* Show wall sample points

### 2D Display ###
Should allow toggling and saving of displayed image, as well as HDR control.

Workflow
--------
1. Start program and load a *mesh* (Required)
   * Check CCW
2. Load a *camfile*, allowing options for what data to load (confidence, depth)
   * *Parameters:*
     * Flipping around an axis
   * *Dynamic parameters:*
   * *Display:* Display cameras and enable 2D displays
   * Notes: Also load auxiliaries if they exist (edges, labels)
3. Load results of *wallfinding* or perform wallfinding
   * *Parameters:*
     * Angle equality threshold
     * Minimum wall length
     * Discrete resolution
     * Wall bucket count threshold
     * Scanline skips allowed
     * Scanline steepness
     * Parallel wall merge distance threshold
     * Perpendicular wall connection overlap threshold
   * *Dynamic parameters:*
   * *Display:* Enable display of label data and result mesh
   * Notes: Note that it will invalidate non-reprojection auxiliaries
4. Load results of *reprojection* or perform reprojection (depends on camfile)
   * *Parameters:*
     * Depth tolerance
   * *Dynamic parameters:*
     * Light radiance threshold (Retiring)
   * *Display:* Enable 3D colored displays
5. Load results of *sampling* or perform sampling (depends on all preceding)
   * *Parameters:*
     * Number of samples
     * Hemicube resolution
     * Max unsampled proportion threshold
     * RANSAC iterations and percent error threshold
   * *Dynamic parameters:* [None]
   * *Display:* Display sampled points and hemicube(?)
6. Perform *inverse* lighting (depends on all preceding)
7. Perform *texture* recovery
   * *Parameters:*
     * Number of samples
     * Angle threshold for viable camera
   * *Dynamic parameters:* [None]
   * *Display:* Result mesh with texture
8. Perform RWO-finding

Shared Memory Planning
----------------------
* Initial allocation and loading performed by command-line server, which is
  initialized with a mesh
* Semaphore for writes(?)
* Individual programs check for server status? Or memory status?

Refactor Tasks
--------------
* Port relevant data structures to Boost.Interprocess
* Write pure visualization interface
* Begin separating out components into separate applications, reading and writing
  appropriately and signalling for updates(?)

Design Questions
----------------
* Mesh - Write own mesh class?
* Hemicuberenderer - Save all results?
