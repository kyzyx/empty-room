import sys, subprocess, tempfile, shutil, multiprocessing

convert_binary = '/home/zyxlyr/Code/empty-room/inverse_render/build/convertimage'
ffmpeg_call = "ffmpeg -framerate 30 -pattern_type glob -i %s/image*.png -c:v libx264 -vf vflip -pix_fmt yuv420p %s"

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

    try:
        for line in open(sys.argv[1]).readlines()[1:]:
            toks = line.split()
            outfile = "%s/image%04d.png"%(tmpdir, n)

            if len(toks) > 12:
                expr = float(toks[-3])*scale
                expg = float(toks[-2])*scale
                expb = float(toks[-1])*scale
                cmd = [convert_binary, toks[0], outfile, str(expr), str(expg), str(expb)];
                cmds.append(cmd)
            else:
                cmd = [convert_binary, toks[0], outfile, str(scale)]
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
