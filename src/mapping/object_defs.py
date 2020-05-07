#!/usr/bin/env python
from file_io import *

class MappingProblem:
  srcObjects = None
  srcOrder = None
  tgtObjects = None

  def __init__(self, src, tgt, srcObjectsOrder, knowDir=None):
    if isinstance(src, basestring):
      self.srcObjects = IO.loadObjects(src,knowDir)
    else:
      self.srcObjects = src
  
    if isinstance(tgt, basestring):
      self.tgtObjects = IO.loadObjects(tgt,knowDir)
    else:
      self.tgtObjects = tgt
    self.srcOrder = srcObjectsOrder

class Mapping:
  mapping = None
  binaryMatrix = None

  def getBinaryMatrix(self, mapping, srcObjects, tgtObjects):
    m = np.zeros(len(srcObjects), len(tgtObjects))
    for i in range(len(mapping)):
      for j in range(len(tgtObjects)):
        if mapping[i] is tgtObjects[j].label:
          m[i][j] = 1
    return m

  def __init__(self, mapping, srcObjects, tgtObjects):
    self.mapping = mapping
    self.binaryMatrix = self.getBinaryMatrix(mapping, srcObjects, tgtObjects)
