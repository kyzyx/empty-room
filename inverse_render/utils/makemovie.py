import sys, subprocess, tempfile, shutil, multiprocessing

convert_binary = '/home/zyxlyr/Code/empty-room/inverse_render/build/convertimage'
ffmpeg_call = "ffmpeg -framerate 30 -pattern_type glob -i %s/image*.png -c:v libx264 -pix_fmt yuv420p %s"

def run_cmd(cmd):
    print ' '.join(cmd)
    subprocess.call(cmd)

if len(sys.argv) < 3:
    print "Usage: makemovie camfile.cam outputfile.mp4 [exposure]"
else:
    scale = 1
    if len(sys.argv) > 3:
        scale = float(sys.argv[3])

    n = 0
    pool = multiprocessing.Pool(None)
    cmds = []
    tmpdir = tempfile.mkdtemp()
    gamma = 1

    try:
        lines = [l.strip() for l in open(sys.argv[1]).readlines()]
        if lines[0].strip() == "CAMFILE_HEADER":
            hdr = lines[0:lines.index("end_header")]
            g = [float(s.split()[1]) for s in hdr if s.split()[0] == "Gamma"]
            if len(g) > 0:
                gamma = g[0]
            lines = lines[lines.index("end_header")+1:]
        else:
            lines = lines[1:]
        for line in lines:
            toks = line.split()
            outfile = "%s/image%04d.png"%(tmpdir, n)

            if len(toks) > 12:
                expr = float(toks[-3])*scale
                expg = float(toks[-2])*scale
                expb = float(toks[-1])*scale
                expstr = "%f,%f,%f"%(expr,expg,expb)
                cmd = [convert_binary, toks[0], outfile, '-colorscale', expstr, '-gamma', str(gamma), '-linear']
                cmds.append(cmd)
            else:
                cmd = [convert_binary, toks[0], outfile, '-intensityscale', str(scale), '-gamma', str(gamma), '-linear']
                cmds.append(cmd)
            n += 1

        pool.map(run_cmd, cmds)

        cmd = ffmpeg_call%(tmpdir, sys.argv[2])
        print cmd
        subprocess.call(cmd.split())
    finally:
        shutil.rmtree(tmpdir)


# Parse camfile to get filename and exposures
# convertimage filename exposure*arg exposure*arg exposure*arg
# ffmpeg -framerate 30 -pattern_type glob -i 'image*.png' -c:v libx264 -vf "vflip" -pix_fmt yuv420p OUTPUT.mp4
