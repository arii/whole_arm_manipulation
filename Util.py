#!/usr/bin/env python
import numpy as np
from math import pi

def determine_nearest_state(array, angles, delta=1):
  valid_angles = return_valid_angles(angles)
  idx = find_nearest_vector(array, valid_angles)
  nearest_state = array[idx]
  if sum(np.abs(np.array(nearest_state)-np.array(valid_angles)))>delta:
    return -1,-1
  else:
    return idx, nearest_state

def find_nearest(array,value):
  idx = (np.abs(array-value)).argmin()
  return idx

def vector_distance(vector, pt):
  return np.sum(np.abs(np.array(vector) - np.array(pt))**2, axis=-1)

def find_nearest_vector(array,value):
  try:
    tmp1 = np.array(array) - np.array(value)
    idx = np.sum(np.abs(tmp1)**2,axis=-1).argmin()
  except: # 1d case
    arr =  list(np.array(array).reshape(-1)) 
    val =  value[0]
    idx = find_nearest(arr, val)
  return idx

def return_valid_angle(angle):
  if angle < 0:
    return angle + 2*pi
  if angle > 2*pi:
    return angle - 2*pi
  return angle

def return_valid_angles(angle):
  return [return_valid_angle(a) for a in angle]

def create_space_idx(n, array_len):
  A = [ [0 for x in range(n)] for x in range(array_len**n)]
  for i in range(array_len**n):
    for j in range(n):
      idx =( i/(array_len**j) )%array_len

      A[i][j] = idx
  return A


def create_space(n, array, mode=int):
  a_n = len(array)
  A = np.empty((a_n**n,n), dtype=mode)
  for i in range(a_n**n):
    for j in range(n):
      idx =( i/(a_n**j) )%a_n

      A[i][j] = array[idx]
  A = [list(x) for x in A]
  return A

def load_policy(filename, length):
  policy = [None]*length
  with open(filename, 'r') as f:
    for line in f:
      data = line.split(',')
      policy[int(data[0])]=[float(data[1]) , float(data[2])]
  return policy

def store_policy(filename, policy):
  def stc(val):
    return str(val) + ','

  with open(filename,'w') as f:
    for i in range(len( policy)):
      p = policy[i]
      data_str = stc(i)
      for j in range(len(p)):
        if j == (len(p) -1):
          data_str = data_str + str(p[j]) + "\n"
        else:
          data_str = data_str + stc(p[j])
      #data_str = stc(i)+ stc(p[0]) + str(p[1])+ "\n"
      f.write(data_str)



