#!/usr/bin/env python
import numpy as np
from simulator import *
from ForwardKinematics import RevoluteRobot


class RL_MDP:
  class State:
    def __init__(self, object_pos, angles):
      self.object_pos = object_pos
      self.angles = angles
    def __str__(self):
      return "Object: %s; angles %s" %(self.object_pos, self.angles)
  
  def __init__(self,obstacle, robot,STEP_SIZE=1):
    
    self.st = Simulator(obstacle, robot, [0]*num_links, False, STEP_SIZE, False)
    self.sim = Simulator(obstacle, robot, [0]*num_links, True, STEP_SIZE, False)

    actions = [Actions.DEC, Actions.INC, Actions.NOTHING]
    self.gamma = 0.999
    self.actions = create_space(num_links, actions)
    self.weights = np.ones(3)
    self.features = np.zeros(3)
    self.fourier_order = 4
    self.alpha = 0.1

  def do_time_step(self, n = 300):
    next_center = self.sim.obstacle.center
    for i in range (n):
      state = self.State(next_center, self.sim.angles)
      next_action, action_val = self.determine_best_action(state)
      next_center = self.sim.discrete_action(next_action)
    
      self.update_value_function(state)
      #print "next action is %s, expected val %f " %(next_action, action_val)
      #print "weights are %s" % self.weights

  def value_function(self, state=None):
    
    if state is not None:
      self.st.force_continuous_state(state)
    
    self.features[0] = self.st.reward_distances
    self.features[1] = self.st.reward_enveloped
    self.features[2] = self.st.num_collisions
    value = np.dot(self.features, self.weights)
    
    """print "\nfeatures : %s" % self.features
    print "features: %s, weights: %s" %(self.features, self.weights)
    print "value is %f " % value"""
    return value
  
  def update_value_function(self, state):
    reward = self.st.force_continuous_state(state)
    next_state_action, next_state_val = self.determine_best_action(state)
    expected_reward = reward + self.gamma*next_state_val
    value_approx = self.value_function(state)
    self.weights = self.weights + self.alpha*(expected_reward - value_approx)*self.features
    
    
  def determine_best_action(self, state):
    max_val = 0
    best_action = self.actions[0]
    object_pos = state.object_pos
    for a in self.actions:
      self.st.discrete_action(a, angles=state.angles, object_pose=object_pos)
      value_new_state = self.value_function()
      #print "actions %s has value %f" %(a, value_new_state)
      if value_new_state > max_val:
        max_val = value_new_state
        best_action = a
        #print "\tnew best action is %s" % a
    #print "best action is %s" % a
    return best_action, max_val

 

num_links = 3
step = 0.01
"""
#pr2 values
robot = RevoluteRobot([.4, .321, .160])
radius = 0.1
center = np.array([-.1, .2])
robot = RevoluteRobot([.4, .321, .160])
"""
# original MDP values
center = np.array([-.25,.75])
radius = 0.25
robot = RevoluteRobot([0.75]*num_links)
obstacle = Obstacle(center, radius)

rl_mdp = RL_MDP(obstacle,robot, STEP_SIZE=step)
quit = '0'
while quit is not 'q':
  quit = raw_input("[q]uit?")
  rl_mdp.do_time_step()
