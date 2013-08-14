#!/usr/bin/env python
""" 
  Forward_kinematics.py
  author: Ariel Anders
  Modified approach to find the homogenous transformation matrix
  as described by John Craig.
"""
import sys, math,numpy
from numpy import matrix, zeros,shape, reshape, array,around, cos, sin
#from CollisionDetector import Pt
import numpy as np

#modified approach (check Link frame DH params)

XY = True

class Link:
  #(a, alpha, d, theta)  Default parameters for reference frame
  def __init__(self, length=0, twist=0, offset=0):
    self.length = length
    self.twist = twist
    self.offset = offset
    
    # static coefficients
    self.cos_twist = cos(twist)
    self.sin_twist = sin(twist)
    self.m24 = -self.sin_twist*offset
    self.m34 =  self.cos_twist*offset
    

  def transform(self, theta, offset=None):
    if offset is not None:
      self.offset = offset
    trans = zeros(shape=(4,4))

    c = cos(theta)
    s = sin(theta)
    
    trans = matrix([[c,-s,0,self.length],\
    [s*self.cos_twist, c*self.cos_twist, -self.sin_twist, self.m24],\
    [s*self.sin_twist, c*self.sin_twist,  self.cos_twist, self.m34],\
    [0, 0, 0, 1]])

    return trans 


class Fk:
  #Links is an matrix of Link
  def __init__(self, Links):
    self.Links = Links
    self.T = None
    self.T_fk = None

  def getFk(self,angles, offsets=None):
    
    if len(angles) != len(self.Links):
      print("number of angles should be the same as Links")
      sys.exit(2)
    
    if offsets is not None:
      self.m24 = -self.sin_twist*offset
      self.m34 =  self.cos_twist*offset
    

    T = [None]*len(angles)  # list of transforms
    T_fk = [None]*len(angles)  #propogated transforms
    T_pos = [None]*len(angles) #propogated transforms positions 
    for  i in range(len(angles)):
      if offsets is not None:
        T[i] = self.Links[i].transform(angles[i], offsets[i])
      else:
        T[i] = self.Links[i].transform(angles[i])
      
      if i == 0:
        T_fk[0] = T[0]
        tmp = T[0]
      else:
        T_fk[i] = T_fk[i-1] * T[i]
      
      # modified to use Pt
      pos = array(around(T_fk[i][:-2,3],decimals=6)).reshape(-1,)
      T_pos[i] = array([pos[0],pos[1]])
      self.T_fk = T_fk
      self.T = T

    return T_pos
     
def unit_vector(vect):
  n = np.linalg.norm(vect,2)
  if n > 0:
    return (1/n)*vect, n
  else:
    return vect, n

class RevoluteRobot:
  def __init__(self, Link_lens):
    base = Link()
    Links = [None]*(len(Link_lens)+1)
    Links[0] = base
    for i in range(1,len(Links)):
      Links[i] = Link(Link_lens[i-1],0,0)
    self.fk = Fk(Links)
    self.Links = Links
    self.end_points = None
  
  def getFk(self,angles):
    return self.fk.getFk(angles + [0])

  def detect_collisions(self, obstacle, delta):
    d, collisions, v_c= self.perp_distance(obstacle.center, obstacle.radius, delta)
    dynamic_v = np.array([0.0, 0.0])
    for v in v_c:
      dynamic_v += v
    obstacle.center = self.compute_center(obstacle.center, dynamic_v, obstacle.radius)
    new_center = self.compute_center(obstacle.center, dynamic_v, obstacle.radius)
    d, collisions, v_c = self.perp_distance(new_center, obstacle.radius, delta)
    return collisions, d, new_center
  
  def compute_center(self, center, dynamic_v, radius):
    #print "dynamic v is %s" % dynamic_v
    p_center = matrix([[center[0]], [center[1]], [0], [1]])
    if dynamic_v[0] == 0 and dynamic_v[1] == 0:
      return center
    r_2 = radius**2
    dyn_v = matrix([[dynamic_v[0]], [dynamic_v[1]], [0],[0]])
    for i in range(0, len(self.fk.T_fk)-1):
      dyn_v = matrix([[dynamic_v[0]], [dynamic_v[1]], [0],[0]])
      inverse_transform = np.linalg.inv(self.fk.T_fk[i])
      transformed_pt = inverse_transform*p_center
      transformed_dyn_v = inverse_transform*dyn_v
      new_y = transformed_pt.item(1) + transformed_dyn_v.item(1)
      sign = 1 if new_y >= 0 else -1
      if new_y**2 < r_2:
        new_y = sign*np.sqrt(r_2- new_y**2)
        transformed_dyn_v[1] = new_y
        dyn_v = self.fk.T_fk[i]*transformed_dyn_v
    x = dyn_v.item(0)
    y = dyn_v.item(1)
    return center + np.array([x,y])

  def perp_distance(self, pt, radius,delta):
    p = matrix([[pt[0]], [pt[1]], [0], [1]])
    d = [None]*(len(self.fk.T_fk)-1)
    collisions = [False]*(len(self.fk.T_fk)-1)
    v_c = [np.array([0,0])]*(len(self.fk.T_fk)-1) # collision vector
    r_2 = radius**2

    for i in range (0,len(self.fk.T_fk)-1):
      #print "transformation matrix is"
      #print self.fk.T_fk[i]
      transformed_pt  = np.linalg.inv(self.fk.T_fk[i])*p
      tp_x = transformed_pt.item(0)
      tp_y = transformed_pt.item(1)
      sign = 1 if tp_y >=0 else -1
      link_len = self.fk.Links[i+1].length
      if tp_x < -radius:
        dist = tp_x**2 + tp_y**2
      elif tp_x > (radius + link_len):
        dist = (tp_x-link_len)**2 + tp_y**2
      else:
        dist = tp_y**2
        if dist < r_2:
          v = np.matrix([[0], [sign], [0], [0]])
          v_t = np.array(self.fk.T_fk[i]*v).reshape(-1)
          #print "vector (0,%f) -> {0} (%f, %f)" %(tp_y, v_t[0], v_t[1])
          v_c[i] = np.abs(tp_x*delta)*(np.array([v_t[0],v_t[1]]))
      if dist < r_2:
        collisions[i] = True
      d[i] = dist* sign
      #print "perp distances for segment is %f" % d[i]
      #print "transformed point is %f,%f" %(tp_x, tp_y)
    #print "d is %s" % d
    return d, collisions, v_c
 
if __name__=="__main__":
  robot = RevoluteRobot([1]*3)
  angles= robot.getFk([math.pi/2]*3)
  for angle in angles:
    print angle

  
  """
  base = Link()

  l1 = Link(1,0,0)
  print "original transform"
  print l1.transform(0)
  print "transform pi degrees"
  print l1.transform(math.pi)
  fk = Fk([base,l1])
  print fk.getFk([math.pi/2,0])

  """
