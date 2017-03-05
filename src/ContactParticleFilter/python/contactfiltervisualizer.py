__author__ = 'manuelli'

import numpy as np
import yaml
import os

import contactfilterutils as cfUtils
from contactfilter import ContactFilter
from director import lcmUtils
from director.debugVis import DebugData
from director import visualization as vis
from director import objectmodel as om
import robotlocomotion as robotlocomotion_lcmtypes


class ContactFilterVisualizer(object):

    def __init__(self, robotSystem, robotStateModel):
        self.robotStateModel = robotStateModel
        self.robotSystem = robotSystem
        self.addSubscribers()
        self.visualize = True
        self.options = cfUtils.loadConfig()

    def start(self):
        self.visualize = True

    def stop(self):
        self.visualize = False

    def addSubscribers(self):
        subscriber = lcmUtils.addSubscriber("CONTACT_PARTICLE_FILTER_DATA", robotlocomotion_lcmtypes.CPF_data_t, self.onContactFilterMsg)
        subscriber.setSpeedLimit(10)

    def getCurrentPose(self):
        return self.robotSystem.robotStateJointController.q

    def drawParticleSet(self, particleSet, name="particle set", color=None, drawMostLikely=True,
                        drawHistoricalMostLikely=True):


        drawMostLikely = self.options['vis']['drawMostLikely']

        # set the color if it was passed in
        defaultColor = [0.5,0,0.5]
        mostLikelyColor = [1,0,0] # red
        historicalMostLikelyColor = [0,1,0] # green


        if color is not None:
            defaultColor = color


        # for making some visualization for paper, remove later
        if False:
            defaultColor = [0,1,1]

        numParticlesAtCFP = {}
        numTotalParticles = len(particleSet.particleList)

        for particle in particleSet.particleList:
            cfp = particle.cfp
            if numParticlesAtCFP.has_key(cfp):
                numParticlesAtCFP[cfp] += 1
            else:
                numParticlesAtCFP[cfp] = 1

        # now we need to draw this
        plungerMaxLength = 0.4
        plungerMinLength = 0.02
        mostLikelyLength = 0.1


        d = DebugData()
        q = self.getCurrentPose()
        if self.options['vis']['drawParticles']:
            for cfp, numParticles in numParticlesAtCFP.iteritems():
                color = defaultColor

                # if particleSet.mostLikelyParticle is not None:
                #     if cfp == particleSet.mostLikelyParticle.cfp:
                #         color = mostLikelyColor

                rayLength = plungerMinLength + 1.0*numParticles/numTotalParticles*plungerMaxLength
                self.addPlungerToDebugData(d, cfp.linkName, cfp.contactLocation, cfp.contactNormal, rayLength, color)
                # forceDirectionWorldFrame, forceLocationWorldFrame =\
                #     cfUtils.getForceDirectionInWorld(q, self.robotStateModel,
                #                                                             cfp.linkName,
                #                                                             cfp.contactLocation,
                #                                                             cfp.contactNormal)
                #
                # rayEnd = forceLocationWorldFrame - forceDirectionWorldFrame*rayLength
                # d.addSphere(forceLocationWorldFrame, radius=0.01, color=color)
                # d.addLine(rayEnd, forceLocationWorldFrame, radius = 0.005, color=color)

        if self.options['vis']['drawHistoricalMostLikely'] and (particleSet.historicalMostLikely['particle'] is not None):
            particle = particleSet.historicalMostLikely['particle']
            cfp = particle.cfp
            color = historicalMostLikelyColor
            rayLength = mostLikelyLength
            forceDirection = cfp.contactNormal
            if particle.solnData is not None:
                forceDirection = particle.solnData['force']
                forceDirection = forceDirection/np.linalg.norm(forceDirection)
            self.addPlungerToDebugData(d, cfp.linkName, cfp.contactLocation, forceDirection, rayLength, color)

        if self.options['vis']['drawMostLikely'] and (particleSet.mostLikelyParticle is not None):
            particle = particleSet.mostLikelyParticle
            cfp = particle.cfp
            color = mostLikelyColor
            rayLength = mostLikelyLength

            forceDirection = cfp.contactNormal
            if particle.solnData is not None:
                forceDirection = particle.solnData['force']
                forceDirection = forceDirection/np.linalg.norm(forceDirection)

            self.addPlungerToDebugData(d, cfp.linkName, cfp.contactLocation, forceDirection, rayLength, color)

        vis.updatePolyData(d.getPolyData(), name, colorByName='RGB255')

    def addPlungerToDebugData(self, d, linkName, contactLocation, contactDirection, rayLength, color):
        q = self.getCurrentPose()
        forceDirectionWorldFrame, forceLocationWorldFrame =\
                cfUtils.getForceDirectionInWorld(q, self.robotStateModel,
                                                                        linkName,
                                                                        contactLocation,
                                                                        contactDirection)

        rayEnd = forceLocationWorldFrame - forceDirectionWorldFrame*rayLength
        d.addSphere(forceLocationWorldFrame, radius=0.01, color=color)
        d.addLine(rayEnd, forceLocationWorldFrame, radius = 0.005, color=color)


    def drawParticleSetList(self, particleSetList, drawMostLikely=True, drawHistoricalMostLikely=True):


        numParticleSets = len(particleSetList)
        maxNumParticleSets = 4
        for i in xrange(0, maxNumParticleSets):
            name = "particle set " + str(i+1)
            om.removeFromObjectModel(om.findObjectByName(name))

        for i, particleSet in enumerate(particleSetList):
            name = "particle set " + str(i+1)

            if i < numParticleSets:
                self.drawParticleSet(particleSet, name=name, color=particleSet.color,
                                     drawMostLikely=drawMostLikely, drawHistoricalMostLikely=drawHistoricalMostLikely)

    def onContactFilterMsg(self, msg):
        particleSetList = ContactFilter.decodeCPFData(msg)

        self.drawParticleSetList(particleSetList)

