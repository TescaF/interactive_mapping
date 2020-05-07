#!/usr/bin/env python

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
from object_defs import *

class SimilarityMetric:

  def diff(self, v1, v2, r):
    dist = scipy.stats.norm(v1, 0.34 * r)
    n1 = dist.pdf(v2)
    n2 = dist.pdf(v1)
    return n1/n2

  def hueSim(self, h1, h2):
    v1 = 0.0
    v2 = math.degrees(math.atan2(math.sin(math.radians(h1-h2)), math.cos(math.radians(h1-h2))))
    return self.diff(v1, v2, 360)

  def hueDiffSim(self, c1, c2, mean):
    v1 = c2
    v2 = c1 + mean
    return self.diff(v1, v2, 360)

  def sizeSim(self, d1, d2, maxDiff):
    v1 = 1
    v2 = (float(d2)/float(d1))
    return self.diff(v1, v2, maxDiff)

  def sizeDiffSim(self, d1, d2, mean):
    v1 = mean
    v2 = (float(d2)/float(d1))
    return self.diff(v1, v2, 1)

  def intersection(self, s1, s2):
    inter = []
    b = copy.deepcopy(s2)
    for val in s1:
      if val in b:
        inter.append(val)
        b.remove(val)
    return inter

  def spatialSim(self, s1, s2):
    v1 = len(s1)
    #v2 = len([filter(lambda x: x in s1, sublist) for sublist in s2])
    v2 = len(self.intersection(s1,s2))
    return self.diff(v1, v2, len(s1))

  def affSim(self, a1, a2):
    v1 = len(a1)
    v2 = len(self.intersection(a1,a2))
    return self.diff(v1, v2, v1)

  def propSim(self, p1, p2):
    v1 = len(p1)
    v2 = len(self.intersection(p1,p2))
    return self.diff(v1, v2, v1)


