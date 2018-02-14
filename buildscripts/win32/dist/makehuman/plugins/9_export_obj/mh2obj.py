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
import skeleton

#
#    exportObj(human, filepath, config):
#

jointGroupNames = []
jointPositions = []

def exportTargets(human, filepath, filename):
    path = os.path.join(filepath, filename+"l-leg_verts.txt")

    f = open(path, "w")
    t = open("data/targets/armslegs/l-leg-valgus-decr.target", "r")

    n = 0

    while True:
        l = t.readline()
        if not l: break

        l = l.split()

        if l[0] != "#":
            n += 1
            f.write(str(l[0]) + "\n")

    f.write("Total: " + str(n))

    f.close()

def exportJoints(human, filepath, filename):
    path = os.path.join(filepath, filename+"joints.txt")
    path2 = os.path.join(filepath, filename+"jointNames.txt")

    f = open(path, "w")
    o = open(path2, "w")

    jointGroupNames = [ group.name for group in human.meshData.faceGroups if group.name.startswith('joint-') ]

    for group in human.meshData.faceGroups:
        o.write(group.name + "\n")


    #for g in jointGroupNames:
        #o.write(g + "\n")


    if human.getSkeleton():
        jointGroupNames += human.getSkeleton().joint_pos_idxs.keys()
        for groupName in jointGroupNames:
            jointPositions.append(human.getSkeleton().getJointPosition(groupName, human))
    else:
        for groupName in jointGroupNames:
            jointPositions.append(skeleton._getHumanJointPosition(human, groupName))

    for c in jointPositions:
        f.write(str(c) + "\n")

    '''

    j = human.getJoints()

    for fg_name in j:
        p = human.getJointPosition(fg_name)
        o.write(fg_name + "\n")
        f.write(str(p) + "\n")
        #print p
        print human.getSkeleton().getJointPosition(fg_name, human)
    '''

    o.close()
    f.close()

def exportLandmarks(human, filepath, filename):
    path = os.path.join(filepath, filename+"landmarks.txt")

    j = human.getJoints()
    f = open(path, "w")

    for j_name in j:
        #if ("head" in j_name) | ("neck" in j_name) | ("elbow" in j_name) | ("shoulder" in j_name) | ("ground" in j_name) | ("calvice" in j_name):
        if ("neck" in j_name) | ("r-shoulder" in j_name) | ("joint-r-hand" == j_name) | ("pelvis" in j_name) | ("spine-3" in j_name) :
           # f.write(j_name + " ")
            p = human.getJointPosition(j_name)
            f.write(str(p) + "\n")

    f.close()


def exportCollisionVolume(human, filepath, filename):
    path = os.path.join(filepath, filename+"collisionvolume.txt")

    j = human.getJoints()
    f = open(path, "w")
    #l_path = os.path.join(filepath, "l_hand.txt")
   # l = open(l_path, "w")
   # r_path = os.path.join(filepath, "r_hand.txt")
   # r = open(r_path, "w")

    # Hand
    '''
    r_hand = {}
    l_hand = {}
    for j_name in j:
        if ("l-finger" in j_name) | ("l-hand" in j_name) :
            p = human.getJointPosition(j_name)
            l_hand[j_name] = p
            l.write(j_name + " ")
            l.write(str(p) + "\n")

        if ("r-finger" in j_name) | ("r-hand" in j_name) :
            p = human.getJointPosition(j_name)
            r_hand[j_name] = p
            r.write(j_name + " ")
            r.write(str(p) + "\n")

    l.close()
    r.close()
    '''
    f.close()



def exportBoundingBox(human, filepath, filename):
    path = os.path.join(filepath, filename+"boundingsurface.txt")

    j = human.getJoints()
    f = open(path, "w")
    rs = [0, 0, 0]
    rh = [0, 0, 0]

    for j_name in j:
        if "r-shoulder" in j_name :
            rs = human.getJointPosition(j_name)
        if j_name == "joint-r-hand":
            rh = human.getJointPosition(j_name)



    f.close()


def exportObj(filepath, config=None):
    progress = Progress(0, None)
    human = config.human
    human.getSkeleton()
    config.setupTexFolder(filepath)
    filename = os.path.basename(filepath)
    name = config.goodName(os.path.splitext(filename)[0])

    # root dir
    root = filepath.replace(filename, "")

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

    progress(0.3, 0.99, "Writing Objects")
    #wavefront.writeObjFile(filepath, meshes, True, config, filterMaskedFaces=not config.hiddenGeom)
    wavefront.writeObjFile(filepath, meshes, os.path.join(root, filename.replace(".obj", "") + "vertices.txt"), True, config, filterMaskedFaces=not config.hiddenGeom)

    #if 'Anaconda Prompt (2)' in win32gui.GetWindowText(0xffff):
    # print win32gui.GetWindowText(0xffff)

    # Get the pid of the process with given name i.e. 'SNU_3DP.exe'
    # p = [item for item in psutil.process_iter() if item.name() == 'SNU_3DP.exe']
    # print p
    # print p[0].pid

    r = win32api.SendMessage(win32con.HWND_BROADCAST, 56789, 0, 0)
    #win32api.RegisterWindowMessage('56789')

    cpid = win32api.GetCurrentProcessId();


    path = os.path.join(root, filename.replace(".obj", "")+ "meshes.txt")
    f = open(path, "w")
    f.close()

    exportTargets(human, root, filename.replace(".obj", ""))
    exportJoints(human, root, filename.replace(".obj", ""))
    exportLandmarks(human, root, filename.replace(".obj", ""))
    exportCollisionVolume(human, root, filename.replace(".obj", ""))
    exportBoundingBox(human, root, filename.replace(".obj", ""))


    progress(1.0, None, "OBJ Export finished. Output file: %s" % filepath)

    # Kill MakeHuman
    # os.system("taskkill /PID " + str(p[0].pid))
    os.system("taskkill /PID " + str(cpid))
