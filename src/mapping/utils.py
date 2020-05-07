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
#from hlpr_speech_synthesis import speech_synthesizer

class Utils:
  @staticmethod
  def getSpatialRelns(label, relns):
    relevant = []
    for rel in relns:
      if rel.startswith(label):
        relevant.append(rel)
    return relevant

  @staticmethod
  def getSpatialRelnTypes(label, relns):
    relevant = []
    for rel in relns:
      if rel.startswith(label):
        relevant.append(rel.split(":")[1])
    return relevant

  @staticmethod
  def featureVariance(objs, ft):
    vals = []
    for o in objs:
      vals.append(o.getFeatureValue(ft))
    if any(c.isalpha() for c in str(vals[0])):
      if ":" in vals[0][0]:
        newVals = []
        for vs in vals:
          newVals.append([v.split(":")[1] for v in vs])
        vals = newVals
      if len(vals) is 1:
        return 0
      s1 = set()
      for v in vals[0]:
        s1.add(v)
      s2 = set()
      for v in vals[1]:
        s2.add(v)
      s = s1.symmetric_difference(s2)
      return len(list(s))
    else:
      return np.var(vals)
  
  @staticmethod
  def getMaxSizeDiff(src):
    maxS = None
    minS = None
    for o in src:
      if maxS is None or o.size > maxS:
        maxS = o.size
      elif minS is None or o.size < minS:
        minS = o.size
    return maxS/minS
