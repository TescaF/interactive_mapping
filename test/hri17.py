#!/usr/bin/env python

'''
Contains all file-reading code and mapping code to generate results used for THRI18 paper.
'''

import sys
import os
import errno
import time
sys.path.append('../src/mapping')
from entity import Entity
from mapper import *
from file_io import *
from object_defs import *

THRESHOLD = 0.02

def correct_mapping(hypothesis, source_order, solutions):
  i = 0
  correct = True
  #print source_order
  #print target_order
  for o in source_order:
    if not (hypothesis[o] in solutions[o]):
      correct = False
      return False
  return True

def test_file(directory, filename, solutions, output_file):
  # Define problem elements
  source_objects_filename = filename + "source_objects.txt"
  target_objects_filename = filename + "target_objects.txt"
  object_order = IO.loadObjectOrder(filename + "targeters.txt")
  hints = IO.loadHints(filename + "hints.txt")
  knowDir = directory + "knowledge/"
  problem = MappingProblem(source_objects_filename, target_objects_filename, object_order, knowDir)
  mapper = Mapper(problem)
  outText = ""
 
  # Initialize hypothesis space and feature set space
  hs, fss = mapper.init()

  # Get initial prediction without mapping hints
  correct = False
  firstCorrect = False
  hintCount = 0
  margin = 0.0
  updateMsg,prediction,nextObject, hs, fss = mapper.updateWithoutHints(hs, fss)
  if prediction is not None:
    correct = correct_mapping(prediction[1], object_order, solutions)
    if correct:
      outText += "*"
      firstCorrect = True
    outText += "Hint 0 - Prediction: " + str(IO.readablePrediction(mapper.src, prediction)) + "\n"
    matrix = prediction[3]
    if len(matrix) > 1:
        maxR = sorted(maxRows(matrix), reverse=True)
        rowMargin = maxR[0] - maxR[1]
        outText += "Row margin: " + str(rowMargin) + "\n"
        maxC = sorted(maxCols(matrix), reverse=True)
        idx = np.unravel_index(matrix.argmax(), matrix.shape)
        nextC = sorted(nextMax(matrix, idx[0], idx[1]), reverse=True)
        oldMargin = maxC[0] - maxC[1]
        colMargin = maxC[0] - nextC[0]
        print "old margin: " + str(oldMargin) + ", new margin: " + str(colMargin)
	    #margin = max(rowMargin,colMargin)
        margin = colMargin
        outText += "Col margin: " + str(colMargin) + "\n"

  # Loop through each mapping hint
  i = 0
  for hint in hints:
    if margin < THRESHOLD and len(hs) > 1:
      if i < len(object_order): 
        # Get objects referenced in hint
        srcObj = object_order[i]
        tgtObj = hints[i]
 
        # Update prediction
        updateMsg, prediction, nextObject, hs, fss = mapper.update(srcObj, tgtObj, hs, fss)
        outText += updateMsg

        if prediction is not None:
          correct = correct_mapping(prediction[1], object_order, solutions)
          if not firstCorrect:
              hintCount = i + 1
          if correct:
              outText += "*"
              firstCorrect = True
          outText += "Hint " + str(i + 1) + " - " + srcObj + " -> " + tgtObj + "\n"
          outText += "Prediction: " + str(IO.readablePrediction(mapper.src, prediction)) + "\n"

          matrix = prediction[3]
          if len(matrix) > 1:
              maxR = sorted(maxRows(matrix), reverse=True)
              rowMargin = maxR[0] - maxR[1]
              outText += "Row margin: " + str(rowMargin) + "\n"
              maxC = sorted(maxCols(matrix), reverse=True)
              idx = np.unravel_index(matrix.argmax(), matrix.shape)
              nextC = sorted(nextMax(matrix, idx[0], idx[1]), reverse=True)
              oldMargin = maxC[0] - maxC[1]
              colMargin = maxC[0] - nextC[0]
              print "old margin: " + str(oldMargin) + ", new margin: " + str(colMargin)
              #pruned = pruneMatrix(matrix, np.unravel_index(matrix.argmax(), matrix.shape))
              #maxCPruned = sorted(maxCols(pruned), reverse=True)
              #print "matrix: " + str(matrix)
              #print "pruned matrix: " + str(pruned)
              #print "max: " + str(maxC[0]) + ", next max: " + str(maxCPruned[0])
              outText += "Col margin: " + str(colMargin) + "\n"
              margin = colMargin
	      #margin = max(rowMargin,colMargin)
        i += 1
  print outText
  f = open(output_file, 'w')
  f.write(outText)
  f.close()
  return hintCount, i, correct

def nextMax(matrix, rowIgnore, colIgnore):
    numcols = len(matrix[0])
    vals = []
    for i in range(numcols):
        if not i == colIgnore:
            maxRow = matrix[:,i].argmax()
            #print "maxrow: " + str(maxRow) + ", matrix: " + str(matrix[:,i])
            if not maxRow == rowIgnore:
                vals.append(matrix[:,i].max())
            else:
                vals.append(0.0)
    return vals

    

def pruneMatrix(matrix, idx):
    newM = np.delete(matrix, idx[0], axis=0)
    newM = np.delete(newM, idx[1], axis=1)
    return newM

def maxCols(matrix):
    numcols = len(matrix[0])
    vals = []
    for i in range(numcols):
        vals.append(matrix[:,i].max())
    return vals

def maxRows(matrix):
    numrows = len(matrix)
    vals = []
    for i in range(numrows):
        vals.append(matrix[i].max())
    return vals


if __name__ == "__main__":
  directory = os.path.dirname(os.path.realpath(__file__)) + "/../data/hri17/"
  output_dir = directory + "results/"
  try:
      os.makedirs(output_dir)
  except OSError as exception:
      if exception.errno != errno.EEXIST:
          raise
  dirs = os.listdir(directory)
  solns = dict()
  results = dict()

  for dir in dirs:
    if dir[0] is 'u':
      print dir
      task = dir.split("_")[1]
      if not (task in solns):
	solns[task] = IO.loadSolutions(directory + "solutions/" + task + ".txt")
      minHints, numHints, correct = test_file(directory, directory + dir + "/", solns[task], output_dir + dir + ".txt")
      results[dir] = [minHints, numHints, correct]

  hintCount = []
  hintCount.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
  hintCount.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
  hintCount.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
  numCorrect = [0,0,0]
  print "Hints needed per task:"
  print "=========="
  for key,value in sorted(results.items()):
    print str(key) + ": " + str(value)
    t = int(key.split("t")[1])
    if value[2] is True:
      numCorrect[t] = numCorrect[t] + 1
    hintCount[t][value[1]] = hintCount[t][value[1]] + 1

  print "Confidence threshold: " + str(THRESHOLD)
  print "Task 0:"
  print "Occurrances of each hint count: " + str(hintCount[0])
  print "Number of correct mappings: " + str(numCorrect[0])
  print "Task 1:"
  print "Occurrances of each hint count: " + str(hintCount[1])
  print "Number of correct mappings: " + str(numCorrect[1])
  print "Task 2:"
  print "Occurrances of each hint count: " + str(hintCount[2])
  print "Number of correct mappings: " + str(numCorrect[2])


