#!/usr/bin/env python

'''
Contains all file-reading code and mapping code to generate results used for HRI17 and RSS17 papers.
'''

import sys
import os
import time
sys.path.append('../src')
from entity import Entity
from mapper import *
from file_io import *
from object_defs import *

if __name__ == "__main__":

  # Define problem elements
  dir = os.path.dirname(__file__)
  filename = os.path.join(dir, '../data/example/')
  source_objects_filename = filename + "source_objects.txt"
  target_objects_filename = filename + "target_objects.txt"
  object_order = ["small_orange", "large_orange", "small_green", "large_green", "small_blue", "large_blue"]
  problem = MappingProblem(source_objects_filename, target_objects_filename, object_order)
  mapper = Mapper(problem)
 
  # List of hints to be provided
  hints = ["red_2", "red_1", "green_1", "green_2", "blue_1", "blue_2"]

  # Initialize hypothesis space and feature set space
  hs, fss = mapper.init()

  # Loop through each mapping hint
  i = 0
  for hint in hints:
    if i < object_order: 
      # Get objects referenced in hint
      srcObj = object_order[i]
      tgtObj = hints[i]

      # Update prediction
      prediction, hs, fss = mapper.update(srcObj, tgtObj, hs, fss)
      if prediction is not None:
        print "Prediction: " + str(IO.readablePrediction(mapper.src, prediction)) + "\n"
    i += 1
