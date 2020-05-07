#!/usr/bin/env python

class Entity:
  featureList = ["h", "ch", "d", "dh", "s", "a", "p"]

  def __init__(self, label, locations, dimensions, hue, affordances, properties):
    self.label = label
    self.location = locations
    self.dimensions = dimensions
    if len(dimensions) is 3:
      self.size = 100 * dimensions[0] * dimensions[1] * dimensions[2]
    if len(dimensions) is 2:
      self.size = dimensions[0] * dimensions[1]
    self.hue = hue
    self.hueDiff = 0
    self.sizeDiff = 0
    self.spatialSet = None
    self.affordances = affordances
    self.properties = properties

  def printObject(self):
    pStr = ""
    if len(self.location) is 3:
      pStr = self.label + ": [" + str(self.location[0]) + " " + str(self.location[1]) + " " + str(self.location[2])
    if len(self.location) is 2:
      pStr = self.label + ": [" + str(self.location[0]) + " " + str(self.location[1])
    if len(self.dimensions) is 3:
      pStr += "], [" + str(self.dimensions[0]) + " " + str(self.dimensions[1]) + " " + str(self.dimensions[2])
    if len(self.dimensions) is 2:
      pStr += "], [" + str(self.dimensions[0]) + " " + str(self.dimensions[1]) 
    pStr += "], " + str(self.hue) + ", {"
    for a in self.affordances:
      pStr += a + " "
    pStr = pStr[:-1] + "}, {"
    for p in self.properties:
      pStr += p + " "
    pStr = pStr[:-1] + "}"
    print pStr

  def setSpatialRelations(self, spatial):
    self.spatialSet = spatial

  def getFeatureValue(self, feature):
    if feature is "h":
      return self.hue
    if feature is "ch":
      return self.hueDiff
    if feature is "d":
      return self.size
    if feature is "dh":
      return self.sizeDiff
    if feature is "s":
      return self.spatialSet
    if feature is "a":
      return self.affordances
    if feature is "p":
      return self.properties

  @staticmethod
  def indexOfFeature(feature):
    for i in range(len(Entity.featureList)):
      if feature is Entity.featureList[i]:
	return i
    return -1 
  
