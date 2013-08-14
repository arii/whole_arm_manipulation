#!/usr/bin/env python
# Simulator
from ForwardKinematics import Link, Fk, RevoluteRobot
import numpy as np
from SimGraphics import SimGraphics
from Util import *


class Actions:
  INC = 1
  DEC = -1
  NOTHING = 0


class Obstacle:
  def __init__(self, center, radius):
    self.center = center
    self.radius = radius
  def __str__(self):
    return "Obstacle with center at %s and radius %f"%(self.center,self.radius)

class Simulator:
  # Simulator Class Variables
  object_values = None
  angle_values = None

  object_space = None 
  angle_space = None
  state_space = None

  obstacle = None
  STEP_SIZE = None
  robot = None
  end_points = None
  angles = None
  d = None
  collisions = None
  reward = None
  
  def __init__(self, obstacle, robot, init_pos, sim=True, STEP_SIZE=.5, discrete=True):
    self.obstacle = obstacle
    self.robot = robot
    self.sim = sim
    self.STEP_SIZE = STEP_SIZE
    self.discrete=discrete


    if (sim):
      self.sg = SimGraphics(obstacle.radius, obstacle.center, [[0,0]]*(len(init_pos) +1), False)


    self.end_points = None
    self.d = None
    self.collisions = None
    self.reward = None
    self.point_distances = None
    if self.discrete:
      self.initialize_state_space()
    self.angles = init_pos
    self.do_action(init_pos)
    if self.discrete:
      self.get_current_state()
  
  def update(self):
    #print "distances are %s" % self.d
    #print "end points at: " + str( self.end_points)
    if self.discrete:
      state = self.get_current_state()
      self.force_state(state)


    self.get_reward()
    if (self.sim):
      self.sg.plot_links(self.obstacle, self.end_points)
      #print self.obstacle.center
    #print "Reward: %f" % self.reward
  
  def force_state(self, state):
    state_values = self.get_state_values(state)
    self.obstacle.center =state_values[0]
    angles = state_values[1]

    fk = self.robot.getFk(angles)
    self.collisions, d, self.obstacle.center = self.robot.detect_collisions(self.obstacle, self.STEP_SIZE)
    self.end_points = fk
    self.d = d
    self.angles = angles
    self.obstacle.center =state_values[0]
    self.get_reward()
    return 

  def force_continuous_state(self, state):
    self.obstacle.center =state.object_pos
    angles = state.angles
    fk = self.robot.getFk(angles)
    self.collisions, d, self.obstacle.center =self.robot.detect_collisions(self.obstacle, self.STEP_SIZE)
    self.end_points = fk
    self.d = d
    self.angles = angles
    self.obstacle.center = state.object_pos
    self.get_reward()
    return self.reward

  def get_reward(self):
    enveloped = 1

    if self.d[0] >= 0:
      for dists in self.d:
        if dists <0:
          enveloped = 0
          #enveloped = -1
    else:
      for dists in self.d:
        if dists >= 0:
          enveloped = 0
          #enveloped = -1
    num_collisions_arr = [1 if collide else 0 for collide in self.collisions]
    self.num_collisions = sum(num_collisions_arr)

    d = sum([np.abs(dist) for dist in self.d])
    distances = 1/(d+1) 
    self.reward_distances = distances
    self.reward_enveloped = enveloped
    self.reward = distances + enveloped +  10*self.num_collisions
    #print "enveloped score %f, distances %f, total reward=%f"\
    #%(enveloped, distances, self.reward)


  def do_action(self, angles, object_pose=None):
    if object_pose is not None:
      self.obstacle.center = object_pose

    fk = self.robot.getFk(angles)
    self.collisions, d, self.obstacle.center = self.robot.detect_collisions(self.obstacle, self.STEP_SIZE)
    if not(True in self.collisions):
      self.end_points = fk
      self.d = d
      self.angles = angles
    self.update()
    return self.obstacle.center

  def discrete_action(self, actions, angles=None, object_pose=None):
    if angles is None:
      angles = list(self.angles)
    for i in range(len(actions)):
      angles[i] += actions[i]*self.STEP_SIZE
    return self.do_action(angles, object_pose)

  def discrete_action_from_state(self, state, actions):
    state_values = self.get_state_values(state)
    object_pose =state_values[0]
    angles = state_values[1]
    for i in range(len(actions)):
        angles[i] += actions[i]*self.STEP_SIZE
    self.do_action(angles, object_pose)
    return self.get_current_state()

  """ Discretized State space functions for MDP.py """
  def get_state_values(self, state):
    # get state indices
    pos = self.object_space[state[0]]
    angle_state_indices = self.angle_space[state[1]]

    # get values for each state index
    object_x_value = self.object_x_values[pos[0]]
    object_y_value = self.object_y_values[pos[1]]
    object_state = [object_x_value, object_y_value]
    
    angle_state = [self.angle_values[idx] for idx in angle_state_indices] 
    return [object_state, angle_state]
  
  def get_current_state(self):
    object_x_indices = find_nearest(self.object_x_values, self.obstacle.center[0])
    object_y_indices = find_nearest(self.object_y_values, self.obstacle.center[1])
    object_state_indices = [object_x_indices, object_y_indices]
    #object_state_indices = [find_nearest(self.object_values, x) for x in self.obstacle.center]
    angle_state_indices = [find_nearest(self.angle_values, x) for x in
        return_valid_angles(self.angles)]
    object_idx = self.object_space.index(object_state_indices)
    angle_idx = self.angle_space.index(angle_state_indices)
    state = [object_idx, angle_idx]
    #state = self.State(object_idx, angle_idx)
    """print state
    print self.get_state_values(state)
    print "actual state is"
    print self.obstacle.center
    print self.angles
    raw_input("does this state look correct?")"""
    return state
    
  def initialize_state_space(self):
    self.num_links = len(self.robot.Links)-1
    #self.angle_values = np.arange(0,2*np.pi, self.STEP_SIZE)
    #self.object_values = np.arange(-1,1,self.STEP_SIZE)
  
    self.angle_values = np.arange(1.03, 2.25, self.STEP_SIZE)
    
    self.object_x_values = np.arange(-0.510, 0.0, self.STEP_SIZE)
    self.object_y_values = np.arange(0.206, 0.565, self.STEP_SIZE)
    #print self.object_x_values
    #print self.object_y_values
    self.angle_space = create_space_idx(self.num_links, len(self.angle_values))
    
    x_len = len(self.object_x_values)
    y_len = len(self.object_y_values)
    self.object_space = [None]*(x_len*y_len)
    #self.object_space = [ [0 for x in range(2)] for x in range(x_len*y_len)]
    ptr = 0
    for i in range(x_len):
        for j in range(y_len):
          self.object_space[ptr] = [i,j]
          ptr+=1


    #self.object_space = create_space_idx(2, len(self.object_values))
    
    s = [None]*(len(self.angle_space)*len(self.object_space))
    ptr = 0
    for i in range(len(self.object_space)):
      for j in range(len(self.angle_space)):
        #s[ptr] = self.State(i,j)
        s[ptr] = [i,j]
        #print self.get_state_values(s[ptr])

        ptr += 1
    self.state_space = s

    

def main():
  num_links = 2
  center = np.array([0,.5])
  radius = 0.25

  link_lengths = [1]*num_links
  robot = RevoluteRobot(link_lengths)
  obstacle = Obstacle(center, radius)
  angles = [0]*num_links 
  #angles = [5*np.pi/3, 3*np.pi/2]
  angles = [1.03,1.03]
  sim = Simulator(obstacle, robot, angles, True, 0.1, False) #[0]*num_links)
  quit = 0
  mv = 1
  max_x = 0
  max_y = .5
  min_x = 0
  min_y = 0.5
  
  while quit is not "q":
    #if True in sim.collisions:
    #  mv *= -1
    center = sim.obstacle.center
    if center[0] > max_x:
      max_x = center[0]
    elif center[0] < min_x:
      min_x = center[0]

    if center[1] > max_y:
      max_y = center[1]
    elif center[1] < min_y:
      min_y = center[1]

    print "object: %s, link angles: %s" % (sim.obstacle.center, sim.angles)
    print "minimum:(%f,%f), maximum:(%f,%f)" %(min_x, min_y, max_x, max_y)
    quit = raw_input("[q]uit or any key to continue")
    sim.discrete_action([mv]*num_links)
if __name__=="__main__":
  main()
