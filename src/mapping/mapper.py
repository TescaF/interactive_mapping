#!/usr/bin/env python

import pdb
import warnings
#import roslib
#import rospy
#import smach
#import smach_ros
import sys
import os
import time
import scipy.stats
import itertools
import math
import copy
import numpy as np
#from std_msgs.msg import String
from entity import Entity 
#from hlpr_speech_synthesis import speech_synthesizer
from object_defs import *
from metrics import *
from utils import *

class Mapper:
  @staticmethod
  def LEARNING_RATE():
    return 0.5

  def __init__(self, problem, verbose = False):
    self.verbose = verbose

    self.src = problem.srcObjects
    self.tgt = problem.tgtObjects
    self.unmappedSrc = copy.copy(self.src)
    self.unmappedTgt = copy.copy(self.tgt)
    self.srcIds = self.initIdDict(self.src)
    self.tgtIds = self.initIdDict(self.tgt)
    self.srcSpatial = self.initSpatialRelations(self.src)
    for o in self.src:
      o.setSpatialRelations(Utils.getSpatialRelns(o.label, self.srcSpatial))
    self.tgtSpatial = self.initSpatialRelations(self.tgt)
    for o in self.tgt:
      o.setSpatialRelations(Utils.getSpatialRelns(o.label, self.tgtSpatial))
    self.hueShifts = []
    self.sizeShifts = []
    #self.objectOrder = []
    #for t in targeters:
    #  self.objectOrder.append(t.split(")")[0][1:])
    self.objectOrder = problem.srcOrder
    self.latestHint = None
    self.currMap = None
    self.evalVals = None
    self.margin = None

  def initIdDict(self, objects):
    ids = dict()
    i = 0
    for o in objects:
      ids[o.label] = i
      i += 1
    return ids

  def initSpatialRelations(self, objects):
    relns = []
    for o1 in objects:
      for o2 in objects:
        if not(o1 is o2):
	  x1 = float(o1.location[0])
	  y1 = float(o1.location[1])
	  w1 = float(o1.dimensions[0])/100.0
	  h1 = float(o1.dimensions[1])/100.0
	  left1 = y1 + (w1 / 2.0)
	  right1 = y1 - (w1 / 2.0)
	  top1 = x1 + (h1 / 2.0)
	  bottom1 = x1 - (h1 / 2.0)

	  x2 = float(o2.location[0])
	  y2 = float(o2.location[1])
	  w2 = float(o2.dimensions[0])/100.0
	  h2 = float(o2.dimensions[1])/100.0
	  left2 = y2 + (w2 / 2.0)
	  right2 = y2 - (w2 / 2.0)
	  top2 = x2 + (h2 / 2.0)
	  bottom2 = x2 - (h2 / 2.0)

	  if left1 < right2:
	    relns.append(str(o1.label) + ":RIGHT_OF:" + str(o2.label))
	  if right1 > left2:
	    relns.append(str(o1.label) + ":LEFT_OF:" + str(o2.label))
	  if top1 < bottom2:
	    relns.append(str(o1.label) + ":BELOW:" + str(o2.label))
	  if bottom1 > top2:
	    relns.append(str(o1.label) + ":ABOVE:" + str(o2.label))
    return relns

  def init(self):
    self.evals = None
    tgtLabels = []
    if self.verbose:
      print "Source:"
      for t in self.src:
        t.printObject()
      print "Target:"
    for t in self.tgt:
      if self.verbose:
        t.printObject()
      tgtLabels.append(t.label)
    mappingSpace = []
    mapPerms = itertools.permutations(tgtLabels, len(self.src))
    for m in mapPerms:
      mappingSpace.append(m)

    ftSetSpace = []
    s = list(Entity.featureList)
    #ftSpaceComb = itertools.chain.from_iterable(itertools.combinations(s, r) for r in range(2))
    ftSpaceComb = itertools.chain.from_iterable(itertools.combinations(s, r) for r in range(len(s)+1))
    for ftSet in ftSpaceComb:
      ftSetSpace.append(ftSet)
    ftSetSpace.remove(())

    avgWeight = 1.0 / len(Entity.featureList)
    weights = []
    for i in range(len(Entity.featureList)):
      weights.append(avgWeight)
    self.hs = mappingSpace
    self.fss = ftSetSpace
    return mappingSpace, ftSetSpace #, weights

  def update(self, srcLabel, tgtLabel, mappingSpace=None, ftSetSpace=None, nextObject=None):
    if mappingSpace is None:
      mappingSpace = self.hs
    if ftSetSpace is None:
      ftSetSpace = self.fss
    self.margin = None
    #msg = "Received hint: " + srcLabel + " -> " + tgtLabel + "\n"
    msg = ""

    # Get objects referenced in hint
    srcObj = self.src[self.srcIds[srcLabel]]
    tgtObj = self.tgt[self.tgtIds[tgtLabel]]

    # Verify that it is not already assigned
    if srcObj not in self.unmappedSrc:
        return msg, None, None, mappingSpace, ftSetSpace
    self.unmappedSrc.remove(srcObj)
    if tgtObj in self.unmappedTgt:
        self.unmappedTgt.remove(tgtObj)
    if len(self.unmappedSrc) is 0:
        msg += "No more objects to map! Ignoring hint.\n"
        return msg, None, None, mappingSpace, ftSetSpace

    # Add hue/size shifts
    self.hueShifts.append(tgtObj.hue - srcObj.hue)
    self.sizeShifts.append(float(tgtObj.size)/float(srcObj.size))

    # Prune hypothesis and feature spaces
    hypSpace = self.pruneMappingHypotheses(mappingSpace, srcObj, tgtObj)
    if len(hypSpace) is 0:
        msg += "Ignoring conflicting hint: " + str(srcObj.label) + " -> " + str(tgtObj.label) + "\n"
        return msg, None, None, mappingSpace, ftSetSpace
    ftSpace = self.pruneFeatureSpace(ftSetSpace, self.unmappedSrc, self.unmappedTgt)

    self.hs = hypSpace
    self.fss = ftSpace

    # Evaluate remaining hypotheses
    if self.evals is None:
      self.evals = self.evalHypotheses(ftSpace, self.src, self.tgt)
    else:
      self.evals = self.updateEvals(self.src, self.tgt, self.evals)
    predictedMapping = self.maximizeMatrix(hypSpace, ftSpace, self.evals)
    return msg, predictedMapping, None, hypSpace, ftSpace

  def getMargin(self, matrix):
    rowMargin = 1.0
    colMargin = 1.0
    '''if len(matrix) > 1:
        maxRows = sorted(self.maxRows(matrix), reverse=True)
        rowMargin = maxRows[0] - maxRows[1]
        print "Row margin: " + str(rowMargin)
        #print maxRows'''
    if len(matrix[0]) > 1:
        maxCols = sorted(self.maxCols(matrix), reverse=True)
        colMargin = maxCols[0] - maxCols[1]
    return colMargin

  def maxCols(self, matrix):
    numcols = len(matrix[0])
    vals = []
    for i in range(numcols):
        vals.append(matrix[:,i].max())
    return vals

  def maxRows(self, matrix):
    numrows = len(matrix)
    vals = []
    for i in range(numrows):
        vals.append(matrix[i].max())
    return vals

  def updateWithoutHints(self, mappingSpace=None, ftSetSpace=None):
    if mappingSpace is None:
      mappingSpace = self.hs
    if ftSetSpace is None:
      ftSetSpace = self.fss
    self.margin = None

    # Prune feature space
    ftSpace = self.pruneFeatureSpace(ftSetSpace, self.unmappedSrc, self.unmappedTgt)

    # Evaluate remaining hypotheses
    self.evals = self.evalHypotheses(ftSpace, self.src, self.tgt)
    #return self.maximizeValue(mappingSpace, ftSpace, evals), mappingSpace, ftSpace
    self.hs = mappingSpace
    self.fss = ftSpace
    return None, self.maximizeMatrix(mappingSpace, ftSpace, self.evals), None, mappingSpace, ftSpace

  def pruneMappingHypotheses(self, hypSpace, srcObj, tgtObj):
    pruned = []
    if self.verbose:
      print "Initial hypothesis space: " + str(hypSpace)
    for m in hypSpace:
      if m[self.srcIds[srcObj.label]] is tgtObj.label:
	pruned.append(m)
    return pruned

  def pruneFeatureSpace(self, ftSetSpace, unmappedSrc, unmappedTgt):
    pruned = []
    for ftSet in ftSetSpace:
      addFtSet = True
      for ft in ftSet:
        srcVar = Utils.featureVariance(unmappedSrc, ft)
        tgtVar = Utils.featureVariance(unmappedTgt, ft)
        if srcVar is 0 or tgtVar is 0:
            addFtSet = False
      if addFtSet:
        pruned.append(ftSet)
    return pruned

  def evalHypotheses(self, ftSetSpace, src, tgt):
    sim = SimilarityMetric()
    e = np.zeros((len(src), len(tgt), len(Entity.featureList)))
    sd = Utils.getMaxSizeDiff(src)
    for i in range(len(src)):
      for j in range(len(tgt)):
	h = sim.hueSim(src[i].hue, tgt[j].hue)
        hueDiff = 0.0 if len(self.hueShifts) is 0 else float(sum(self.hueShifts)) / float(len(self.hueShifts))
	ch = sim.hueDiffSim(src[i].hue, tgt[j].hue, hueDiff)
	d = sim.sizeSim(src[i].size, tgt[j].size, sd)
        sizeDiff = 0.0 if len(self.sizeShifts) is 0 else float(sum(self.sizeShifts)) / float(len(self.sizeShifts))
	dh = sim.sizeDiffSim(src[i].size, tgt[j].size, sizeDiff)
	s = sim.spatialSim(Utils.getSpatialRelnTypes(src[i].label, self.srcSpatial), Utils.getSpatialRelnTypes(tgt[j].label, self.tgtSpatial))
	a = sim.affSim(src[i].affordances, tgt[j].affordances)
	p = sim.propSim(src[i].properties, tgt[j].properties)
	e[i][j] = [h, ch, d, dh, s, a, p]
    return e

  def updateEvals(self, src, tgt, evals):
    sim = SimilarityMetric()
    for i in range(len(src)):
      for j in range(len(tgt)):
        hueDiff = 0.0 if len(self.hueShifts) is 0 else float(sum(self.hueShifts)) / float(len(self.hueShifts))
	ch = sim.hueDiffSim(src[i].hue, tgt[j].hue, hueDiff)
        sizeDiff = 0.0 if len(self.sizeShifts) is 0 else float(sum(self.sizeShifts)) / float(len(self.sizeShifts))
	dh = sim.sizeDiffSim(src[i].size, tgt[j].size, sizeDiff)
	evals[i][j][1] = ch
	evals[i][j][3] = dh
    return evals

  def getBinaryMatrix(self, mapping):
    m = np.zeros((len(self.src), len(self.tgt)))
    for i in range(len(mapping)):
      for j in range(len(self.tgt)):
	if mapping[i] is self.tgt[j].label:
	  m[i][j] = 1
    return m

  def convertToDict(self, hypothesis):
    newDict = {}
    for i in range(len(self.src)):
      newDict[self.src[i].label] = hypothesis[i]
    return newDict

  def maximizeMatrix(self, hypSpace, ftSetSpace, evals, weights = None):
    valueMatrix = np.zeros((len(hypSpace), len(ftSetSpace)))
    print "Matrix is size " + str(len(hypSpace)) + " x " + str(len(ftSetSpace))
    #print "hyp space: " + str(hypSpace)
    #print "ft set space: " + str(ftSetSpace)

    evalSlices = []
    for ft in Entity.featureList:
      evalSlices.append(np.array(evals)[:,:,Entity.indexOfFeature(ft)])
    #print evalSlices
    # Evaluate each hypothesis
    for i in range(len(hypSpace)):
      mapEvals = []
      m = hypSpace[i]
      binaryMap = self.getBinaryMatrix(m)

      # Record evaluations according to each feature
      for fi in range(len(Entity.featureList)):
        eval = sum(sum(np.multiply(binaryMap, evalSlices[fi])))
        mapEvals.append(eval)

      # Record total evaluation for current mapping on each feature set
      for j in range(len(ftSetSpace)):
        ftSet = ftSetSpace[j]
        eval = 0
        for ft in ftSet:
            eval += mapEvals[Entity.indexOfFeature(ft)]
        valueMatrix[i][j] = eval / (float(len(ftSet)) * float(len(m)))
        #valueMatrix[i][j] = eval / float(len(ftSet))
    maxIdx = np.unravel_index(valueMatrix.argmax(), valueMatrix.shape)
    maxMap = hypSpace[maxIdx[0]]
    maxFts = ftSetSpace[maxIdx[1]]
    self.currMap = maxMap
    self.evalScores = valueMatrix
    mapDict = self.convertToDict(maxMap)
    self.margin = self.getMargin(valueMatrix)
    return maxMap, mapDict, maxFts, valueMatrix, self.margin

  #def getMaxMargin(self, data):
    
