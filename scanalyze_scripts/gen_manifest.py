import sys

if len(sys.argv) < 2:
    print "Usage: python gen_manifest.py num_files"
else:
    num_files = int(sys.argv[1])
    f = open("MANIFEST", "w")
    for i in range(num_files):
        f.write("frame%.4d.depth.ply\n"%i)
