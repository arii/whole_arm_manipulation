#!/usr/bin/env python
import numpy as np
from simulator import *
from Util import *
from ForwardKinematics import RevoluteRobot


class MDP:
  def __init__(self, obstacle, robot, STEP_SIZE=1):
    num_links = len(robot.Links)-1
    self.st = Simulator(obstacle, robot, [0]*num_links, False, STEP_SIZE)
    actions = [Actions.DEC, Actions.INC, Actions.NOTHING]
    self.gamma = 0.999
    self.threshold = .5
    self.max_k = 50
    self.actions = create_space(num_links, actions)
    
    self.V = [0]*len(self.st.state_space)
    self.Policies = [None]*len(self.V)

  def solve(self):
    self.value_iteration()
    #print self.V
    self.get_policy()
    #print self.Policies
  
  def value_iteration(self):
    k = 0
    V_prev = [0]*len(self.st.state_space)
    V_curr = [0]*len(self.st.state_space)
    cont_loop =True
    while cont_loop:
      #print V_curr
      k +=1
      V_prev = list(V_curr)
      precision_reached = True
      print k
      for state in self.st.state_space:
        idx = self.st.state_space.index(state)
        r = self.evaluate_state(state)
        max_value = 0
        for a in self.actions:
          next_state = self.evaluate_state_action(state, a)
          n_idx = self.st.state_space.index(next_state)
          max_value = V_prev[n_idx] if V_prev[n_idx] > max_value else max_value
        #print "V[%d] = %f" %(idx, max_value)
        V_curr[idx] = r + self.gamma*max_value
        prec = np.fabs(V_curr[idx] - V_prev[idx])
        if prec > self.threshold:
          #print prec
          precision_reached = False
      if precision_reached or k >= self.max_k:
        print "precision reached!!"
        cont_loop = False
    self.V = V_curr
  def evaluate_state_action(self, state, action):
    curr_state = self.st.discrete_action_from_state(state, action)
    return curr_state

  def evaluate_state (self, state):
    self.st.force_state(state)
    return self.st.reward

  def get_policy(self):
    for state in self.st.state_space:
        idx = self.st.state_space.index(state)
        max_value = 0
        max_a = Actions.NOTHING
        max_value = 0
        for a in self.actions:
          next_state = self.evaluate_state_action(state, a)
          n_idx = self.st.state_space.index(next_state)
          if self.V[n_idx] > max_value :
            max_value = self.V[n_idx]
            max_a = a
        self.Policies[idx]=max_a

class State(MDP):
  def __init__(self,object_idx, angles_idx):
    global angle_space, object_space
    self.object_idx = object_idx
    self.angles_idx = angles_idx
  def get_state(self):
    return [object_space[self.object_idx], angle_space[self.angles_idx] ]

  def __str__(self):
    return "object pose at %s, with link angles: %s"\
        % (self.object_idx, self.angles_idx)


def main(store):
  load = not store # and False
  center = np.array([0,.5])
  radius = 0.25
  num_links = 2
  step = 0.075
  link_lengths = [1]*num_links
  robot = RevoluteRobot(link_lengths)
  obstacle = Obstacle(center, radius)
  mdp = MDP(obstacle, robot, step)

  #mdp.solve()
  #policy = mdp.Policies

  filename = 'policy2.txt'
  np.random.seed(0)
  if store:
    mdp.solve()
    policy = mdp.Policies
    store_policy(filename,policy)
  elif load:
    policy = load_policy(filename, len(mdp.st.state_space))

  init_pos = [0]*num_links 
  st = Simulator(obstacle, robot, init_pos, True, step)
  prev_idx = 0

  c = 0
  state = st.get_current_state()
  while c is not 'q':
    
    if c is 'n':
      idx = np.random.randint(0, len(st.state_space)-1)
      state = st.state_space[idx]
      a = policy[idx]
      state = st.discrete_action_from_state(state,a)
      while True in st.collisions:
        idx = np.random.randint(0, len(st.state_space)-1)
        state = st.state_space[idx]
        a = policy[idx]
        state = st.discrete_action_from_state(state,a)

    else:
      idx = st.state_space.index(state)
    a = policy[idx]
    print "idx %d, with action %s, and reward %f" %(idx, a, st.reward)
    state = st.discrete_action_from_state(state, a)
    c = raw_input("\n\n[q]uit or [n]ew initial angle ",)

if __name__=="__main__":
  store =False
  main(store)
  
