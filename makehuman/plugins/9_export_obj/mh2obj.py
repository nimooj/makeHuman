#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

"""
**Project Name:**      MakeHuman

**Product Home Page:** http://www.makehuman.org/

**Code Home Page:**    https://bitbucket.org/MakeHuman/makehuman/

**Authors:**           Thomas Larsson, Jonas Hauquier

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
Exports proxy mesh to obj

"""

import wavefront
import os
from progress import Progress
import numpy as np
import win32api, win32gui, win32con
import psutil
import json
import bvh
from core import G

def exportJoints(human, filepath, filename, centering):
    #path = os.path.join(filepath, filename+"joints.txt")
    #path2 = os.path.join(filepath, filename+"jointNames.txt")

    if not human.getSkeleton():
        G.app.prompt('Error', 'You did not select a skeleton from the library.', 'OK')
        return

    skel = human.getSkeleton()
    b = bvh.BVH()
    joints = {}

    if human.isPosed():
        joints = b.fromObjSkeleton(skel, filepath, filename, centering, human.getActiveAnimation())
    else:
        joints = b.fromObjSkeleton(skel, filepath, filename, centering)

    return joints


def exportObj(filepath, config=None):
    progress = Progress(0, None)
    human = config.human
    human.getSkeleton()
    config.setupTexFolder(filepath)
    filename = os.path.basename(filepath)
    name = config.goodName(os.path.splitext(filename)[0])

    # root dir
    root = filepath.replace(filename, "")
    height = 0

    progress(0, 0.3, "Collecting Objects")
    objects = human.getObjects(excludeZeroFaceObjs=not config.hiddenGeom)
    meshes = [o.mesh for o in objects]

    if config.hiddenGeom:
        # Disable the face masking on copies of the input meshes
        meshes = [m.clone(filterMaskedVerts=False) for m in meshes]
        for m in meshes:
            # Would be faster if we could tell clone() to do this, but it would 
            # make the interface more complex.
            # We could also let the wavefront module do this, but this would 
            # introduce unwanted "magic" behaviour into the export function.
            face_mask = np.ones(m.face_mask.shape, dtype=bool)
            m.changeFaceMask(face_mask)
            m.calcNormals()
            m.updateIndexBuffer()

    pure_name = filename.replace(".obj", "")
    progress(0.3, 0.99, "Writing Objects")



    #centering, crotch_y = wavefront.writeObjFile(filepath, meshes, True, config, filterMaskedFaces=not config.hiddenGeom)
    centering, crotch_y = wavefront.writeObjFile(filepath, meshes, os.path.join(filepath, "..\\" + pure_name + "vertices.txt"), True, config, filterMaskedFaces=not config.hiddenGeom)

    joints = exportJoints(human, filepath, pure_name, centering)
    wavefront.splitSections(filepath, pure_name, meshes, crotch_y, joints, config)

    crotch_y -= centering
    crotch_y *= 100

    #if 'Anaconda Prompt (2)' in win32gui.GetWindowText(0xffff):
    # print win32gui.GetWindowText(0xffff)

    # Get the pid of the process with given name i.e. 'SNU_3DP.exe'
    # p = [item for item in psutil.process_iter() if item.name() == 'SNU_3DP.exe']
    # print p
    # print p[0].pid

    #print  "before crotch_y: " + str(crotch_y)
    #print "centering: " + str(centering)
    #print  "after crotch_y: " + str(crotch_y)

   # crotch_y = (crotch_y*100-centering)
    print crotch_y
    r = win32api.SendMessage(win32con.HWND_BROADCAST, 56789, 0, 0)
    #win32api.RegisterWindowMessage('56789')

    cpid = win32api.GetCurrentProcessId();


    path = os.path.join(root, pure_name+ ".BodyInfo")

    bl = open(pure_name + "Landmarks", "r")
    bj = open(pure_name + "Joints", "r")
    bv = open(pure_name + "Indices", "r")
    #bv = open("common_idx", "r")

    f = open(path, "w")

    data = [None] * 20
    data[0] = "Landmark=6\n"
    # initial write
    while True:
        l = bl.readline()    
        if not l: break

        #f.write(l)
        if "Side Neck" in l:
            data[1] = l
        if "Shoulder" in l: 
            data[2] = l
        if "Waist" in l:
            data[3] = l
        #if "Crotch" in l:
   #         data[4] = "Crotch Level=" + str(crotch_y) + "\n"
        if "Wrist" in l:
            data[5] = l
        if "Ankle" in l:
            data[6] = l

    data[4] = "Crotch Level=" + str(crotch_y) + "\n"
    data[7] = "Joint=12\n"
    # init write
    while True:
        l = bj.readline()
        if not l: break
        #f.write(l)
        if "Head Center" in l:
            data[8] = l
        if "Neck Center" in l:
            data[9] = l
        if "Shoulder Center"  in l:
            data[10] = l
        if "Right Shoulder" in l:
            data[11] = l
        if "Left Shoulder" in l:
            data[12] = l
        if "Right Wrist" in l:
            data[13] = l
        if "Left Wrist" in l:
            data[14] = l
        if "Pelvis Center" in l:
            data[15] = l
        if "Right Pelvis" in l:
            data[16] = l
        if "Left Pelvis" in l:
            data[17] = l
        if "Right Ankle" in l:
            data[18] = l
        if "Left Ankle" in l:
            data[19] = l


    s = ''.join(str(e) for e in data)
    f.writelines(s)
    while True:
        l = bv.readline()
        if not l: break
        f.write(l)

    f.close()

    bl.close()
    bj.close()
    bv.close()

    os.remove(pure_name+"Landmarks")
    os.remove(pure_name+"Joints")
    os.remove(pure_name+"Indices")


    progress(1.0, None, "OBJ Export finished. Output file: %s" % filepath)

    # Kill MakeHuman
    #os.system("taskkill /PID " + str(cpid))
