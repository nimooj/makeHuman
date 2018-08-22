#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

"""
Handles WaveFront .obj 3D mesh files.

**Project Name:**      MakeHuman

**Product Home Page:** http://www.makehuman.org/

**Code Home Page:**    https://bitbucket.org/MakeHuman/makehuman/

**Authors:**           Joel Palmius, Marc Flerackers, Jonas Hauquier

**Copyright(c):**      MakeHuman Team 2001-2017

**Licensing:**         AGPL3

    This file is part of MakeHuman (www.makehuman.org).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.


Abstract
--------

"""

import os
import module3d
import codecs
import math
import numpy as np
from codecs import open  # TODO should Wavefront OBJ files contain unicode characters, or would it be better to strip them?

def loadObjFile(path, obj = None):
    """
    Parse and load a Wavefront OBJ file as mesh.
    Parser does not support normals, and assumes all objects should be smooth
    shaded. Use duplicate vertices for achieving hard edges.
    """
    if obj == None:
        name = os.path.splitext( os.path.basename(path) )[0]
        obj = module3d.Object3D(name)

    objFile = open(path, 'rU', encoding="utf-8")

    fg = None
    mtl = None

    verts = []
    uvs = []
    fverts = []
    fuvs = []
    groups = []
    has_uv = False
    materials = {}
    faceGroups = {}

    for objData in objFile:

        lineData = objData.split()
        if len(lineData) > 0:

            command = lineData[0]

            # Vertex coordinate
            if command == 'v':
                verts.append((float(lineData[1]), float(lineData[2]), float(lineData[3])))

            # Vertex texture (UV) coordinate
            elif command == 'vt':
                uvs.append((float(lineData[1]), float(lineData[2])))

            # Face definition (reference to vertex attributes)
            elif command == 'f':
                if not fg:
                    if 0 not in faceGroups:
                        faceGroups[0] = obj.createFaceGroup('default-dummy-group')
                    fg = faceGroups[0]

                uvIndices = []
                vIndices = []
                for faceData in lineData[1:]:
                    vInfo = faceData.split('/')
                    vIdx = int(vInfo[0]) - 1  # -1 because obj is 1 based list
                    vIndices.append(vIdx)

                    # If there are other data (uv, normals, etc)
                    if len(vInfo) > 1 and vInfo[1] != '':
                        uvIndex = int(vInfo[1]) - 1  # -1 because obj is 1 based list
                        uvIndices.append(uvIndex)

                if len(vIndices) == 3:
                    vIndices.append(vIndices[0])
                fverts.append(tuple(vIndices))

                if len(uvIndices) > 0:
                    if len(uvIndices) == 3:
                        uvIndices.append(uvIndices[0])
                    has_uv = True
                if len(uvIndices) < 4:
                    uvIndices = [0, 0, 0, 0]
                fuvs.append(tuple(uvIndices))

                groups.append(fg.idx)

            elif command == 'g':
                fgName = lineData[1]
                if fgName not in faceGroups:
                    faceGroups[fgName] = obj.createFaceGroup(fgName)
                fg =  faceGroups[fgName]

            elif command == 'usemtl':
                pass # ignore materials

            elif command == 'o':

                obj.name = lineData[1]

    objFile.close()

    # Sanity check for loose vertices
    strayVerts = []
    referencedVerts = set([ v for fvert in fverts for v in fvert ])
    for vIdx in xrange(len(verts)):
        if vIdx not in referencedVerts:
            strayVerts.append(vIdx)
    if len(strayVerts) > 0:
        import log
        msg = "Error loading OBJ file %s: Contains loose vertices, not connected to a face (%s)"
        log.error(msg, path, strayVerts)
        raise RuntimeError(msg % (path, strayVerts))

    obj.setCoords(verts)
    obj.setUVs(uvs)
    obj.setFaces(fverts, fuvs if has_uv else None, groups)

    obj.calcNormals()
    obj.updateIndexBuffer()

    return obj


def splitSections(filepath, filename, meshes, joints, config=None, filterMaskedFaces=True):
    vertices = []

    if config and config.feetOnGround:
        offset = config.offset
    else:
        offset = [0, 0, 0]

    scale = config.scale if config is not None else 1.0

    if filterMaskedFaces:
        meshes = [m.clone(scale=scale, filterMaskedVerts=True) for m in meshes]
    else:
        # Unfiltered
        meshes = [m.clone(scale=scale, filterMaskedVerts=False) for m in meshes]


    flag = []
    for i in range(12):
        flag.append(False)

    h = []
    n = []
    t = []
    la = []
    lh = []
    ll = []
    lf = []
    ra = []
    rh = []
    rl = []
    rf = []
    s = []

    idx = 0
    h_c = 0
    n_c = 0
    t_c = 0
    la_c = 0
    lh_c = 0
    ll_c = 0
    lf_c = 0
    ra_c = 0
    rh_c = 0
    rl_c = 0
    rf_c = 0
    s_c = 0

    l_normal = np.subtract(np.array(joints["Left Wrist"]), np.array(joints["Left Shoulder"]))
    r_normal = np.subtract(np.array(joints["Right Wrist"]), np.array(joints["Right Shoulder"]))

    r_mid = [(joints["Right Wrist"][0] + joints["Right Pelvis"][0])/2, (joints["Right Wrist"][1] + joints["Right Pelvis"][1])/2, (joints["Right Wrist"][2] + joints["Right Pelvis"][2])/2]
    r_mid1 = [r_mid[0], r_mid[1], r_mid[2]-1]
    r_mid2 = [r_mid[0], r_mid[1], r_mid[2]+1]

    p1 = np.array(joints["Right Shoulder"])
    p2 = np.array(r_mid1)
    p3 = np.array(r_mid2)
    v2 = p2 - p1
    v3 = p3 - p1
    r_diag_normal =  np.cross(v2, v3)

    l_mid = [(joints["Left Wrist"][0] + joints["Left Pelvis"][0])/2, (joints["Left Wrist"][1] + joints["Left Pelvis"][1])/2, (joints["Left Wrist"][2] + joints["Left Pelvis"][2])/2]
    l_mid1 = [l_mid[0], l_mid[1], l_mid[2]+1]
    l_mid2 = [l_mid[0], l_mid[1], l_mid[2]-1]

    p1 = np.array(joints["Left Shoulder"])
    p2 = np.array(l_mid1)
    p3 = np.array(l_mid2)
    v2 = p2 - p1
    v3 = p3 - p1
    l_diag_normal =  np.cross(v2, v3)

    pel_1 = [joints["Pelvis Center"][0], joints["Pelvis Center"][1], joints["Pelvis Center"][2] - 1]
    pel_2 = [joints["Pelvis Center"][0], joints["Pelvis Center"][1], joints["Pelvis Center"][2] + 1]
    p1 = np.array(joints["Pelvis Center"])
    p2 = np.array(pel_1)
    p3 = np.array(pel_2)
    v2 = p2 - p1
    v3 = p3 - p1

    flatten = []
    crotch = -1.0
    for mesh in meshes:
        # Get max Y val
        for co in mesh.coord:
            l = str(tuple(co + offset))
            l = l.replace('(', '')
            l = l.replace(')', '')
            sx, sy, sz = l.split(', ')
            x = float(sx)
            y = float(sy)
            z = float(sz)

            #if x >= -0.0001 and x <= 0.0001:
            if abs(x) < 0.000001:
                flatten.append(y)

    flatten.sort()
    crotch = flatten[0]

    for mesh in meshes:
        for co in mesh.coord:
            tt = tuple(co + offset)
            x = float(tt[0])
            y = float(tt[1])
            z = float(tt[2])

            vert = np.array([x, y, z])


            # Head
            if y >= joints["Head Center"][1]-0.5 and x < joints["Left Shoulder"][0] and x > joints["Right Shoulder"][0]:
                if not flag[10]: # First access
                    h.append("Name=Head\n")
                    h.append("") # Node=??\n
                    flag[10] = True
                h.append(str(idx) + "\n")
                #h_c += 1

            # Neck
            elif y < joints["Head Center"][1]-0.5 and y >= joints["Neck Center"][1]:
                if not flag[11]:
                    n.append("Name=Neck\n")
                    n.append("")
                    flag[11] = True
                n.append(str(idx) + "\n")
                #n_c += 1

            # Right Arm
            elif x < joints["Right Shoulder"][0] and np.dot(r_normal, vert - np.array(joints["Right Shoulder"])) > 0 and np.dot(r_normal, vert - np.array(joints["Right Wrist"])) <= 0 and np.dot(r_diag_normal, vert - r_mid) > 0:
                if not flag[4]:
                    ra.append("Name=Right Arm\n")
                    ra.append("")
                    flag[4] = True
                ra.append(str(idx) + "\n")
                #ra_c += 1

            # Left Arm
            elif x > joints["Left Shoulder"][0] and np.dot(l_normal, vert - np.array(joints["Left Shoulder"])) > 0 and np.dot(l_normal, vert - np.array(joints["Left Wrist"])) <= 0 and np.dot(l_diag_normal, vert - l_mid) > 0:
                if not flag[5]:
                    la.append("Name=Left Arm\n")
                    la.append("")
                    flag[5] = True
                la.append(str(idx) + "\n")
                #la_c += 1

            # Right Hand
            elif np.dot(r_diag_normal, vert - r_mid) > 0 and np.dot(r_normal, vert - np.array(joints["Right Wrist"])) > 0:
                if not flag[6]:
                    rh.append("Name=Right Hand\n")
                    rh.append("")
                    flag[6] = True
                rh.append(str(idx) + "\n")
                #rh_c += 1

            # Left Hand
            elif np.dot(l_diag_normal, vert-l_mid) > 0 and np.dot(l_normal, vert - np.array(joints["Left Wrist"])) > 0:
                if not flag[7]:
                    lh.append("Name=Left Hand\n")
                    lh.append("")
                    flag[7] = True
                lh.append(str(idx) + "\n")
                #lh_c += 1


            # Right Leg
            elif y <= joints["Waist level"][1] and y > joints["Right Ankle"][1] and x <= joints["Pelvis Center"][0]:
                if not flag[2]:
                    rl.append("Name=Right Leg\n")
                    rl.append("")
                    flag[2] = True
                rl.append(str(idx) + "\n")
                #rl_c += 1

                if not flag[1]:
                    s.append("Name=Skirt\n")
                    s.append("")
                    flag[1] = True
                s.append(str(idx) + "\n")
                #s_c += 1

                if y > crotch: 
                    if not flag[0]:
                        t.append("Name=Torso\n")
                        t.append("")
                        flag[0] = True
                    t.append(str(idx) + "\n")

            # Left Leg
            elif y <= joints["Waist level"][1] and y > joints["Left Ankle"][1] and x >= joints["Pelvis Center"][0]:
                if not flag[3]:
                    ll.append("Name=Left Leg\n")
                    ll.append("")
                    flag[3] = True
                ll.append(str(idx) + "\n")
                #ll_c += 1

                if not flag[1]:
                    s.append("Name=Skirt\n")
                    s.append("")
                    flag[1] = True
                s.append(str(idx) + "\n")
                #s_c += 1

                if y > crotch: 
                    if not flag[0]:
                        t.append("Name=Torso\n")
                        t.append("")
                        flag[0] = True
                    t.append(str(idx) + "\n")

            # Right Foot
            elif x < joints["Pelvis Center"][0] and y <= joints["Right Ankle"][1]:
                if not flag[9]:
                    rf.append("Name=Right Foot\n")
                    rf.append("")
                    flag[9] = True
                rf.append(str(idx) + "\n")
                #rf_c += 1

            # Left Foot
            elif x > joints["Pelvis Center"][0] and y <= joints["Left Ankle"][1]:
                if not flag[8]:
                    lf.append("Name=Left Foot\n")
                    lf.append("")
                    flag[8] = True
                lf.append(str(idx) + "\n")
                #lf_c += 1


            # Torso ... 나머지 node 모두 배정 + (0, 0, 0) 과 crotch 사이의 점
            elif y < joints["Side Neck level"][1] and y > crotch:
                if not flag[0]:
                    t.append("Name=Torso\n")
                    t.append("")
                    flag[0] = True
                t.append(str(idx) + "\n")
                #t_c += 1

            idx += 1



    h_c = len(h) - 2
    n_c = len(n) - 2
    t_c = len(t) - 2
    la_c = len(la) - 2
    lh_c = len(lh) - 2
    ll_c = len(ll) - 2
    lf_c = len(lf) - 2
    ra_c = len(ra) - 2
    rh_c = len(rh) - 2
    rl_c = len(rl) - 2
    rf_c = len(rf) - 2
    s_c = len(s) - 2

    h[1] = "Node=" + str(h_c) + "\n"
    n[1] = "Node=" + str(n_c) + "\n"
    t[1] = "Node=" + str(t_c) + "\n"
    la[1] = "Node=" + str(la_c) + "\n"
    lh[1] = "Node=" + str(lh_c) + "\n"
    ll[1] = "Node=" + str(ll_c) + "\n"
    lf[1] = "Node=" + str(lf_c) + "\n"
    ra[1] = "Node=" + str(ra_c) + "\n"
    rh[1] = "Node=" + str(rh_c) + "\n"
    rl[1] = "Node=" + str(rl_c) + "\n"
    rf[1] = "Node=" + str(rf_c) + "\n"
    s[1] = "Node=" + str(s_c) + "\n"

    total = t + s + rl + ll + ra + la + rh + lh + rf + lf + h + n

    c = open(filename+"Indices", "w")
    c.write("Part=12\n")
    for l in total:
        c.write(l)

    c.close()
    return crotch


#def writeObjFile(path, meshes, writeMTL=True, config=None, filterMaskedFaces=True):
def writeObjFile(path, meshes, filepath, writeMTL=True, config=None, filterMaskedFaces=True):
    if not isinstance(meshes, list):
        meshes = [meshes]

    if isinstance(path, file):
        fp = path
    else:
        fp = open(path, 'w', encoding="utf-8")

    min_y = 0
    max_y = 0

    fp.write(
        "# MakeHuman exported OBJ\n" +
        "# www.makehuman.org\n\n")

    if writeMTL:
        mtlfile = path.replace(".obj",".mtl")
        fp.write("mtllib %s\n" % os.path.basename(mtlfile))

    scale = config.scale if config is not None else 1.0

    if config and config.feetOnGround:
        offset = config.offset
    else:
        offset = [0,0,0]

    if filterMaskedFaces:
        meshes = [m.clone(scale=scale, filterMaskedVerts=True) for m in meshes]
    else:
        # Unfiltered
        meshes = [m.clone(scale=scale, filterMaskedVerts=False) for m in meshes]

    # mj - exclude nipple point meshes
    # Vertices
    ni = 0
    for mesh in meshes:
        #fp.write("".join( ["v %.4f %.4f %.4f\n" % tuple(co + offset) for co in mesh.coord] ))
        for co in mesh.coord:
            if ni < 1778 or (ni > 1793 and ni < 8450) or ni > 8465:
                fp.write("".join(["v %.4f %.4f %.4f\n" % tuple(co + offset)]))
            else:
                co[2] -= 0.3
                fp.write("".join(["v %.4f %.4f %.4f\n" % tuple(co + offset)]))
            ni += 1

    flatten = []
    for mesh in meshes:
        # Get max Y val
        for co in mesh.coord:
            l = str(tuple(co + offset))
            l = l.replace('(', '')
            l = l.replace(')', '')
            sx, sy, sz = l.split(', ')
            x = float(sx)
            y = float(sy)
            z = float(sz)

            #if x >= -0.0001 and x <= 0.0001:
            if abs(x) < 0.000001:
                flatten.append(y)

            if y < min_y:
                min_y = y
            if y > max_y:
                max_y = y

    flatten.sort()
    crotch_y = flatten[0]
    centering = (min_y + max_y)/2

    # Vertex normals
    if config is None or config.useNormals:
        for mesh in meshes:
            fp.write("".join( ["vn %.4f %.4f %.4f\n" % tuple(no) for no in mesh.vnorm] ))

    # UV vertices
    for mesh in meshes:
        if mesh.has_uv:
            fp.write("".join( ["vt %.6f %.6f\n" % tuple(uv) for uv in mesh.texco] ))

    # Faces
    nVerts = 1
    nTexVerts = 1
    for mesh in meshes:
        fp.write("usemtl %s\n" % mesh.material.name)
        fp.write("g %s\n" % mesh.name)

        if config is None or config.useNormals:
            if mesh.has_uv:
                for fn,fv in enumerate(mesh.fvert):
                    if not mesh.face_mask[fn]:
                        continue
                    fuv = mesh.fuvs[fn]
                    line = [" %d/%d/%d" % (fv[n]+nVerts, fuv[n]+nTexVerts, fv[n]+nVerts) for n in range(4)]
                    fp.write("f" + "".join(line) + "\n")
            else:
                for fn,fv in enumerate(mesh.fvert):
                    if not mesh.face_mask[fn]:
                        continue
                    line = [" %d//%d" % (fv[n]+nVerts, fv[n]+nVerts) for n in range(4)]
                    fp.write("f" + "".join(line) + "\n")
        else:
            if mesh.has_uv:
                for fn,fv in enumerate(mesh.fvert):
                    if not mesh.face_mask[fn]:
                        continue
                    fuv = mesh.fuvs[fn]
                    line = [" %d/%d" % (fv[n]+nVerts, fuv[n]+nTexVerts) for n in range(4)]
                    fp.write("f" + "".join(line) + "\n")
            else:
                for fn,fv in enumerate(mesh.fvert):
                    if not mesh.face_mask[fn]:
                        continue
                    line = [" %d" % (fv[n]+nVerts) for n in range(4)]
                    fp.write("f" + "".join(line) + "\n")

        nVerts += len(mesh.coord)
        nTexVerts += len(mesh.texco)

#    f.close()
    fp.close()

    if writeMTL:
        fp = open(mtlfile, 'w', encoding="utf-8")
        fp.write(
            '# MakeHuman exported MTL\n' +
            '# www.makehuman.org\n\n')
        for mesh in meshes:
            writeMaterial(fp, mesh.material, config)
        fp.close()
    
#    return [centering, crotch_y]
    return centering
#
#   writeMaterial(fp, mat, config):
#

def writeMaterial(fp, mat, texPathConf = None):
    fp.write("\nnewmtl %s\n" % mat.name)
    diff = mat.diffuseColor
    spec =  mat.specularColor
    # alpha=0 is necessary for correct transparency in Blender.
    # But may lead to problems with other apps.
    if mat.diffuseTexture:
        alpha = 0
    else:
        alpha = mat.opacity
    fp.write(
        "Kd %.4g %.4g %.4g\n" % (diff.r, diff.g, diff.b) +
        "Ks %.4g %.4g %.4g\n" % (spec.r, spec.g, spec.b) +
        "d %.4g\n" % alpha
    )

    writeTexture(fp, "map_Kd", mat.diffuseTexture, texPathConf)
    writeTexture(fp, "map_D", mat.diffuseTexture, texPathConf)
    writeTexture(fp, "map_Ks", mat.specularMapTexture, texPathConf)
    #writeTexture(fp, "map_Tr", mat.translucencyMapTexture, texPathConf)
    # Disabled because Blender interprets map_Disp as map_D
    if mat.normalMapTexture:
        texPathConf.copyTextureToNewLocation(mat.normalMapTexture)
    #writeTexture(fp, "map_Disp", mat.specularMapTexture, texPathConf)
    #writeTexture(fp, "map_Disp", mat.displacementMapTexture, texPathConf)

    #writeTexture(fp, "map_Kd", os.path.join(getpath.getSysDataPath("textures"), "texture.png"), texPathConf)


def writeTexture(fp, key, filepath, pathConfig = None):
    if not filepath:
        return

    if pathConfig:
        newpath = pathConfig.copyTextureToNewLocation(filepath) # TODO use shared code for exporting texture files
        fp.write("%s %s\n" % (key, newpath))
    else:
        fp.write("%s %s\n" % (key, filepath))

