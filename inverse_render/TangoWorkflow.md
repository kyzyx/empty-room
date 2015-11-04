Workflow for Project Tango Data Capture
=======================================
1. Get an Area Description File for the scene to scan (to prevent drift during
   data capture)
   a. Open Project Tango Explorer on the Tango
   b. Select "Area Learning"
   c. Press "Learn"
   d. Scan the space (slowly)
   e. Press stop when done
   f. Save the ADF
2. Grab data from the scene
   a. Open Tango Data Dump on the Tango
   b. Press "Open ADF"
   c. Select the ADF you just scanned
   d. Wait for both the green and blue circles to light up, indicating that
      tracking and global localization have stabilized
      (Optional: Press "Debug Overlay" to see where depth data is getting
      captured)
   e. Press "Start Capture" and choose a filename to save as
   f. Scan the scene (slowly)
   g. Press "Stop Capture"
3. Get the data to your computer
   a. Open a file explorer on the Tango (e.g. ES File Explorer)
   b. Navigate to the file location (default is in /sdcard)
   c. Rename the three generated data files: Assuming the filename you selected
      in 2e. was "data.bin", rename the following files
          - data.bin -> data.dat
          - data.bin.pts -> data.pts
          - data.bin.xforms -> data.xforms
      It doesn't matter what you rename them to, this is just because of a bug
      where the computer can't see the data files Tango has written until they
      are renamed.
   d. Connect the Tango to your computer and copy the files from the Tango
4. Preprocess the image data
   a. Run the following commands
        > split data.dat data.xforms image%04d.bin
        > ls image????.bin | create_camfile -i > data.cam
      This should create a bunch of image files as well as a camera file
      containing image filenames and pose data.
5. Preprocess the depth data using Poisson Surface Reconstruction or Floating Scale Surface Reconstruction
   Poisson:
       a. Run the following commands
            > processdepth data.pts data.ply
       b. Open data.ply in Meshlab and verify it looks correct
       c. In Meshlab, go to Filters > Remeshing, Simplification, and Reconstruction > Surface Reconstruction: Poisson
       d. Set Octree Depth to 8 or 9 and press "Apply"
       e. Save the resulting mesh as e.g. mesh.ply
   FSSR:
       a. Run the following commands
            > processdepth data.pts data.ply
       b. Navigate to the FSSR folder and run
            > mve/apps/fssrecon/fssrecon -s 2 data.ply raw.ply
            > mve/apps/meshclean/meshclean -c 100 raw.ply mesh.ply
6. Load the data into the visualizer to ensure no coordinates are flipped.
     > emptyroomui
   File > Open Mesh, select mesh.ply
   File > Open Camera File, select data.cam. You probably want to select "Flip Vertical"

Radiometric Calibration
-----------------------
1. Generate masked confidence files for reprojection
     $ generatemask camfile.cam
2. Reproject & solve
     $ dataserver -meshfile mesh.ply -ccw &
     $ reprojectapp -camfile camfile.cam -meshfile mesh.ply -noshm -flip_y
     $ exposuresolverapp -meshfile mesh.ply > exposures.txt
   Note that if your mesh is very large, it can take a long time to run the
   exposuresolverapp. To make this faster, we can solve on only a subset of
   the mesh vertices by using -subsample n, to sample only 1/n of the vertices.
3. Update camera file
     $ paste --delimiters='' camfile.cam exposures.txt > camfileexp.cam
4. Regenerate confidence files
     $ mkdir masks && mv *.conf masks/
     $ confidence camfile.cam


Troubleshooting
---------------
- Cannot see data files when connecting to computer
    - Navigate to /sdcard on the Project Tango using a file explorer.
      If the files can be seen there, rename them or move them using
      the file explorer. You should be able to see them on the computer
- Cannot see Tango when connected to computer
    - Try a different USB cable
    - Make sure USB MTP is turned on: On the Tango, go to 
         Settings > Storage > ... > USB computer connection
      and check Media device (MTP)
 - Poor alignment
    - Make sure the ADF you are using is accurate, i.e. has the same lighting
      and the same scene layout as when you scan.
    - Capture the ADF very slowly, carefully, and completely. The ADF size is
      miniscule compared to the data capture size, so take your time.

