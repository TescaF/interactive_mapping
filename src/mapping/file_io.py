#!/usr/bin/env python

'''
This is a file-based offline test for the object mapping code. Consists mainly of just IO code and calls to the current mapping code. 
'''

import pdb
import warnings
import sys
import os
import time
import scipy.stats
import itertools
import math
import copy
import numpy as np
from entity import Entity 

class IO:
  def __init__(self, directory, knowDir):
    self.directory = os.path.expanduser(directory)
    self.affDict, self.propDict = self.loadDictionaries(knowDir, ["lunch_knowledge.txt", "bowls_knowledge.txt", "cups_knowledge.txt"])
    self.src = self.loadObjects(self.directory + "objects.txt")
    self.tgt = self.loadObjects(self.directory + "target_objects.txt")
    self.targeters = self.loadTargeters()
    self.hints = self.loadHints()
    #print "Hints: " + str(self.hints)

  @staticmethod
  def readablePrediction(src, pred):
    st = ""
    i = 0
    if pred is None:
      return ""
    for p in pred[0]:
      st += src[i].label + " -> " + p + "  "
      i += 1
    return st + "\nMapping type: " + str(pred[2][0])

  @staticmethod
  def loadDictionaries(directory, filenames):
    affDict = dict()
    propDict = dict()
    for filename in filenames:
      with open(os.path.expanduser(directory + filename)) as f:
        lines = f.readlines()
      for a in lines[0][:-1].split(";")[1:]:
        l = a.split(":")[0]
        aff = a.split(":")[1].split(",")
        affDict[l] = aff

      for p in lines[1][:-1].split(";")[1:]:
        l = p.split(":")[0]
        prop = p.split(":")[1].split(",")
        propDict[l] = prop
    return affDict, propDict

  @staticmethod
  def loadObjects(filename, knowDir=None):
    if knowDir is None:
      affDict = None
      propDict = None
    else:
      affDict, propDict = IO.loadDictionaries(knowDir, ["lunch_knowledge.txt", "bowls_knowledge.txt", "cups_knowledge.txt"])
    with open(filename) as f:
        lines = f.readlines()
    objects = []
    for l in lines:
        s = l[:-1].split(",")
        label = s[0]
        loc = [float(s[1]),float(s[2]),float(s[3])]
        dim = [float(s[4]),float(s[5]),float(s[6])]
        hue = float(s[7])
        if affDict is None:
          aff = s[8][1:-1].split(";")
        else:
          aff = affDict[label] #s[8][1:-1].split(";")
        if propDict is None:
          prop = s[9][1:-1].split(";")
        else:
          prop = propDict[label] #s[9][1:-1].split(";")
        o = Entity(label,loc,dim,hue,aff,prop)
        objects.append(o)
    return objects

  @staticmethod
  def loadTargeters(filename):
    with open(filename) as f:
        lines = [l[:-1] for l in f.readlines()][:-1]
    return lines

  @staticmethod
  def loadObjectOrder(filename):
    targeters = IO.loadTargeters(filename)
    object_order = []
    for t in targeters:
      o = t[1:].split(")")[0]
      object_order.append(o)
    return object_order

  @staticmethod
  def loadHints(filename):
    with open(filename) as f:
        lines = [l[:-1] for l in f.readlines()]
    #return lines
    return IO.filterHints(lines)

  @staticmethod
  def loadOrderedSolutions(filename):
    solutions = []
    with open(filename) as f:
      lines = [l[:-1] for l in f.readlines()]
      sol = []
      for l in lines:
        if l[0] is '-':
          solutions.append(sol)
	  sol = []
        else:
	  sol.append(l)
    solutions.append(sol)
    return solutions

  @staticmethod
  def loadSolutions(filename):
    solutions = dict()
    with open(filename) as f:
      lines = [l[:-1] for l in f.readlines()]
      for l in lines:
	sol = []
	src = l.split(":")[0]
	tgt = l.split(":")[1]
	for s in tgt.split(","):
	  sol.append(s)
	solutions[src] = sol
    return solutions

  @staticmethod
  def filterHints(hints):
    retHints = []
    prevHint = None
    for h in hints:
      #Check for repeated hints
      if not h == prevHint:
	retHints.append(h)
	#Check for repeated hint sequences
        if IO.containsRepetition(retHints):
	  retHints = retHints[:-2]
	prevHint = h 
    return retHints

  @staticmethod
  def containsRepetition(hints):
    if len(hints) <= 2:
      return False
    if hints[0] == hints[-2] and hints[1] == hints[-1]:
      return True
    return IO.containsRepetition(hints[1:])

if __name__ == "__main__":
  warnings.simplefilter("error")
  #rospy.init_node("mapping", anonymous=False)
  io = IO("~/data/hri17/user4_task0/c1/", "~/projects/sandbox/transfer/data/")
  #io = IO("~/data/9-16_Austin_Data/tesca/user4_task0/c1/", "~/projects/tesca-sandbox/transfer/data/")
  mapper = Mapping(io.src, io.targeters, io.tgt)
  hypSpace, ftSetSpace = mapper.initSpaces()
  i = 0
  prevHint = None
  for hint in io.hints:
    if i < len(io.targeters) and (not hint == prevHint):
      print "Hint " + str(i + 1)
      srcHint = io.targeters[i].split(")")[0][1:]
      prediction, hypSpace, ftSetSpace = mapper.provideHint(hypSpace, ftSetSpace, srcHint, hint)
      if prediction is not None:
        print "prediction: " + str(io.readablePrediction(mapper.src, prediction)) 
        print "eval values: " + str(sorted(prediction[2])[::-1])
      prevHint = hint
    i += 1


