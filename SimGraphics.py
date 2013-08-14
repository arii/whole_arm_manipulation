#!/usr/bin/python
from graphics import *


class SimGraphics:
  def __init__(self, r, center, end_points,use_point=True):
    self.win = GraphWin("simulator", 500,500)
    s = 2
    self.win.setBackground('white')
    self.win.setCoords(-s,-s,s,s)

    # draw coordinate frame
    x_line = Line(Point(-s, 0), Point(s,0))
    y_line = Line(Point(0, -s), Point(0,s))
    x_line.setArrow('both')
    y_line.setArrow('both')
    x_line.draw(self.win)
    y_line.draw(self.win)
    self.use_point = use_point
    if use_point:
      self.c = Circle(Point(center.x, center.y),r)
    else:
      self.c = Circle(Point(center[0], center[1]),r)
    self.c.setWidth(5)
    self.c.draw(self.win)
    
    self.l = [None]*(len(end_points)-1)
    if self.use_point:
      points = [Point(pt.x, pt.y) for pt in end_points]
    else:
      points = [Point(pt[0],pt[1]) for pt in end_points]
    
    for i in range (len(points)-1):
      self.l[i] = Line(points[i], points[i+1])
      self.l[i].setWidth(5)
      self.l[i].draw(self.win)
    
    self.l_pts = [None]*len(points)
    for i in range(len(points)):
      self.l_pts[i] = Circle(points[i],.025)
      self.l_pts[i].setWidth(5)
      self.l_pts[i].setFill('black')
      self.l_pts[i].draw(self.win)


  def plot_links(self, obstacle, end_points):
    self.c.undraw()
    self.c = Circle(Point(obstacle.center[0], obstacle.center[1]), obstacle.radius)
    self.c.setWidth(5)
    self.c.setOutline('gray')
    self.c.setFill('gray')
    self.c.draw(self.win)

    for line in self.l:
      line.undraw()
    if self.use_point:
      points = [Point(pt.x, pt.y) for pt in end_points]
    else:
      points = [Point(pt[0],pt[1]) for pt in end_points]
    for i in range (len(points)-1):
      self.l[i] = Line(points[i], points[i+1])
      self.l[i].setWidth(5)
      self.l[i].draw(self.win)
    for i in range(len(points)):
      self.l_pts[i].undraw()
      self.l_pts[i] = Circle(points[i],.025)
      self.l_pts[i].setWidth(5)
      self.l_pts[i].setFill('black')
      self.l_pts[i].draw(self.win)


def main():
  win = GraphWin("My Circle", 100,100)
  c = Circle(Point(50,50),10)
  c.draw(win)
  l = Line(Point(0,0), Point(100,100))
  l.draw(win)
  raw_input("click enter to move line")
  l.undraw()
  l = Line(Point(0,0), Point(50,50))
  l.draw(win)
  win.getMouse()
  win.close()
 
if __name__=="__main__":
  sg = SimGraphics(1, Point(0,0), [Point(0,0), Point(0,1)])
  raw_input()
