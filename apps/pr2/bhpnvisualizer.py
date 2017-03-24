import sys
import lcm
from director import viewerclient as vc
import bot_core as lcmbotcore
import webcolors
import json
import numpy as np
from geometry import shapes


def clear():
    vis = vc.Visualizer('HPN')
    vis.delete()


def getColor(colorName, default='black'):
    if colorName is None:
        colorName = default
    c = webcolors.name_to_rgb(colorName)
    return [c[0]/255.0, c[1]/255.0, c[2]/255.0, 1.0]


def drawFrames(frames):
    for frameName, frame in frames.iteritems():
        vis['frames'][frameName].setgeometry(vc.Triad(scale=0.2))
        vis['frames'][frameName].settransform(frame.matrix)


def collectGeometry(thing, geomList, color=None):

    if isinstance(thing, hu.Point):
        matrix = thing.matrix
        xyz = matrix[:3,0]
        # not implemented...

    elif isinstance(thing, np.ndarray):
        matrix = thing
        pts = matrix[:3,:].transpose()
        # not implemented...

    elif hasattr(thing, 'parts'):

        for part in thing.parts():

            if isinstance(part, shapes.Shape):
                partColor = color or part.properties.get('color')
                for p in part.parts():
                    drawThing(p, geomList, color=partColor)

            elif isinstance(part, (shapes.Thing, shapes.Prim)):
                verts = part.vertices()
                edges = part.edges()
                for e in xrange(edges.shape[0]):
                    p1 = verts[:3,edges[e,0]]
                    p2 = verts[:3,edges[e,1]]

                    geomList.append(
                        vc.GeometryData(
                            vc.PolyLine([list(p1), list(p2)], radius=False),
                            color=getColor(color)))


def drawShape(shape, color=None):
    name = shape.name()
    geomList = []
    collectGeometry(shape, geomList, color=color or shape.properties.get('color'))
    vis['geometries'][name].setgeometry(geomList)


def publishJointConf(conf):

    confMsg = lcmbotcore.viewer_command_t()
    confMsg.command_data = json.dumps(conf)
    lc = lcm.LCM()
    lc.publish('BHPN_PR2_JOINTCONF', confMsg.encode())


def visualizeCallback(planTest):

    clear()

    drawFrames(planTest.realWorld.frames)
    for shape in planTest.realWorld.getNonShadowShapes():
        drawShape(shape)

    drawShape(planTest.realWorld.robotPlace, color='gold')
    publishJointConf(planTest.realWorld.robotConf.conf)
