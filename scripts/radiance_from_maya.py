#!/usr/bin/env python
import os,subprocess
from math import pi,sin,cos,atan
from FbxCommon import *

def rotate(p, v, a):
    theta = a*pi/180
    ca = cos(theta)
    sa = sin(theta)
    dot = p[0]*v[0] + p[1]*v[1] + p[2]*v[2]
    return [
      v[0]*dot*(1-ca) + p[0]*ca + (-v[2]*p[1] + v[1]*p[2])*sa,
      v[1]*dot*(1-ca) + p[1]*ca + (v[2]*p[0] - v[0]*p[2])*sa,
      v[2]*dot*(1-ca) + p[2]*ca + (-v[1]*p[0] + v[0]*p[1])*sa,
    ]

def translate(p,t):
    return [p[0]+t[0], p[1]+t[1], p[2]+t[2]]

def scale(p,t):
    return [p[0]*t[0], p[1]*t[1], p[2]*t[2]]

if len(sys.argv) < 3:
    print "Usage: radiance_from_maya.py scene.fbx scene [mayacamera.cam]"
    sys.exit(0)

# Write rad file
fbxfile = sys.argv[1]
prefix = sys.argv[2]
camfile = ""
if len(sys.argv) == 4:
    camfile = sys.argv[3]

mgr, scene = InitializeSdkObjects()
if not LoadScene(mgr, scene, fbxfile):
    print "Error loading FBX file!"
    sys.exit(0)

out = open("%s.rad"%prefix, 'w')

minp = [float("inf") for i in range(3)]
maxp = [float("-inf") for i in range(3)]

def processMeshNode(node):
    global minp, maxp
    mesh = node.GetNodeAttribute()
    geometry = mesh.GetNode()
    for j in range(geometry.GetMaterialCount()):
        material = geometry.GetMaterial(j)
        if not(material.GetClassId().Is(FbxSurfacePhong.ClassId) or material.GetClassId().Is(FbxSurfaceLambert.ClassId)):
            continue
        diffuse = material.Diffuse.Get()
        emissive = material.Emissive.Get()
        if emissive[0] == 0 and emissive[1] == 0 and emissive[2] == 0:
            if material.GetClassId().Is(FbxSurfacePhong.ClassId):
                specular = material.Specular.Get()
                refl = specular[0]/diffuse[0] + specular[1]/diffuse[1] + specular[2]/diffuse[2]
                refl /= 3
                out.write("void plastic %s\n"%geometry.GetMaterial(j).GetName())
                out.write("0\n0\n5 %f %f %f %f %f\n\n"%(diffuse[0], diffuse[1], diffuse[2], refl, 1/(2+material.Shininess.Get())))
            elif material.GetClassId().Is(FbxSurfaceLambert.ClassId):
                out.write("void plastic %s\n"%geometry.GetMaterial(j).GetName())
                out.write("0\n0\n5 %f %f %f 0 0\n\n"%(diffuse[0], diffuse[1], diffuse[2]))
        else:
            out.write("void light %s\n"%geometry.GetMaterial(j).GetName())
            out.write("0\n0\n3 %f %f %f\n\n"%(emissive[0], emissive[1], emissive[2]))

    for j in range(mesh.GetPolygonCount()):
        layermaterial = mesh.GetLayer(0).GetMaterials()
        mat = geometry.GetMaterial(layermaterial.GetIndexArray().GetAt(i))
        out.write("%s polygon %s_%d\n"%(mat.GetName(),geometry.GetName(),j))
        out.write("0 0 %d\n"%(3*mesh.GetPolygonSize(j)))
        frontface = ""
        backface = ""
        for k in range(mesh.GetPolygonSize(j)):
            v = mesh.GetControlPoints()[mesh.GetPolygonVertex(j,k)]
            v = scale(v, node.LclScaling.Get())
            v = rotate(v, [0,0,1], node.LclRotation.Get()[2])
            v = rotate(v, [0,1,0], node.LclRotation.Get()[1])
            v = rotate(v, [1,0,0], node.LclRotation.Get()[0])
            v = translate(v, node.LclTranslation.Get())

            minp = [min(v[a], minp[a]) for a in range(3)]
            maxp = [max(v[a], maxp[a]) for a in range(3)]

            out.write("%f %f %f\n"%(v[0], v[1], v[2]))
        out.write("\n")

def processLightNode(node):
    global minp, maxp
    light = node.GetNodeAttribute()
    lightname = node.GetName()
    if light.LightType.Get() == FbxLight.eArea:
        out.write("void light %s\n"%lightname)
        color = [c*light.Intensity.Get() for c in light.Color.Get()]
        out.write("0\n0\n3 %f %f %f\n\n"%(color[0], color[1], color[2]))
        out.write("%s polygon %s_\n"%(lightname, lightname))
        out.write("0 0 12\n")
        poly = [[-1,-1,0],
                [-1,1,0],
                [1,1,0],
                [1,-1,0]]
        for v in poly:
            v = scale(v, node.LclScaling.Get())
            v = rotate(v, [0,0,1], node.LclRotation.Get()[2])
            v = rotate(v, [0,1,0], node.LclRotation.Get()[1])
            v = rotate(v, [1,0,0], node.LclRotation.Get()[0])
            v = translate(v, node.LclTranslation.Get())
            minp = [min(v[a], minp[a]) for a in range(3)]
            maxp = [max(v[a], maxp[a]) for a in range(3)]
            out.write("%f %f %f\n"%(v[0], v[1], v[2]))
        out.write("\n")
    elif light.LightType.Get() == FbxLight.ePoint:
        out.write("void light %s\n"%lightname)
        color = [c*light.Intensity.Get() for c in light.Color.Get()]
        out.write("0\n0\n3 %f %f %f\n\n"%(color[0], color[1], color[2]))
        out.write("%s sphere %s_\n"%(lightname, lightname))
        v = node.LclTranslation.Get()
        minp = [min(v[a], minp[a]) for a in range(3)]
        maxp = [max(v[a], maxp[a]) for a in range(3)]
        out.write("0 0 4 %f %f %f 0.005\n\n"%(v[0],v[1],v[2]))

def processNode(node):
    for i in range(node.GetChildCount()):
        if not node.GetChild(i).GetNodeAttribute():
            continue
        elif node.GetChild(i).GetNodeAttribute().GetAttributeType() == FbxNodeAttribute.eMesh:
            processMeshNode(node.GetChild(i))
        elif node.GetChild(i).GetNodeAttribute().GetAttributeType() == FbxNodeAttribute.eLight:
            processLightNode(node.GetChild(i))
        elif node.GetChild(i).GetChildCount() > 0:
            processNode(node.GetChild(i))


processNode(scene.GetRootNode())

# Write rif file
out.close()
if camfile != "":
    out = open("%s.rif"%prefix, 'w')

    lines = [l.strip() for l in open(camfile, 'r').readlines() if len(l) > 2 and l[0:2] != "//"]
    cameras = [l for l in lines if '=' not in l]
    params = {l.split('=')[0]: l.split('=')[1] for l in lines if '=' in l}

    width = int(params['ResX'])
    height = int(params['ResY'])
    ap = float(params['CamApert'])
    foc = float(params['FocLen'])*0.03937008 * width/ap

    out.write("ZONE= I %f %f %f %f %f %f\n"%(minp[0], maxp[0], minp[1], maxp[1], minp[2], maxp[2]))
    out.write("scene= %s.rad\n"%prefix)
    out.write("VAR= Med\n")
    out.write("DET= L\n")
    out.write("QUAL= Hi\n")
    out.write("INDIRECT= 5\n")
    out.write("PENUMBRAS= T\n")
    out.write("RESOLUTION= %d %d\n"%(width, height))
    out.write("AMBFILE= %s.amb\n\n"%prefix)

    for i,c in enumerate(cameras):
        vals = [float(f) for f in c.split()]
        x = [1,0,0]
        y = [0,1,0]
        z = [0,0,1]

        up = y
        up = rotate(up, z, vals[5])
        up = rotate(up, x, vals[3])
        up = rotate(up, y, vals[4])
        towards = [0,0,-1]
        towards = rotate(towards, z, vals[5])
        towards = rotate(towards, x, vals[3])
        towards = rotate(towards, y, vals[4])

        hfov = 2*atan(width/(2*foc))*180/pi
        vfov = 2*atan(height/(2*foc))*180/pi

        out.write("view= pos%d -vtv "%i)
        out.write("-vp %f %f %f "%(vals[0], vals[1], vals[2]))
        out.write("-vu %f %f %f "%tuple(up))
        out.write("-vd %f %f %f "%tuple(towards))
        out.write("-vh %f -vv %f "%(hfov, vfov))
        out.write("-vo 0 -va 0 -vs 0 -vl 0\n")

    # Someday, when meshlab can read FBX...
    #subprocess.call(['meshlabserver', '-i', fbxfile, '-o', '%s.ply'%prefix,
                          #'-s', 'resample.mlx', '-om', 'vc', 'vt'])
