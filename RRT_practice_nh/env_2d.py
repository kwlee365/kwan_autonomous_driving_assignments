#!/usr/bin/env python
""" @package environment_interface
Loads an environment file from a database and returns a 2D
occupancy grid.

Inputs : file_name, x y resolution (meters to pixel conversion)
Outputs:  - 2d occupancy grid of the environment
          - ability to check states in collision
"""
import random
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sklearn.neighbors import KDTree

def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.299, 0.587, 0.114])

def rgb2binary(rgb): # it converts binary image in rgb-form to binary image as itself.
    rgb = np.mean(rgb, axis=2)
    rgb[rgb!=0.0] = 1.0 # if it is obviously not an obstacle pixel(0), then it is regarded as free pixel(1)
    return rgb

class Env2D():
  def __init__(self):
    self.plot_initialized = False
    self.image = None
    

  def initialize(self, envfile, params):
    """Initialize environment from file with given params

      @param envfile - full path of the environment file
      @param params  - dict containing relevant parameters
                           {x_lims: [lb, ub] in x coordinate (meters),
                            y_lims: [lb, ub] in y coordinate (meters)}
      The world origin will always be assumed to be at (0,0) with z-axis pointing outwards
      towards right
    """
    try:
      self.image = plt.imread(envfile)
      self.image2 = plt.imread(envfile)# for visualizing alpha image
      self.image2[self.image2 != 0] = 1

      print("img shape: ", self.image2.shape)
      if len(self.image.shape) > 2:
        self.image = rgb2binary(self.image) # change the image to rgb
    except IOError:
      print("File doesn't exist. Please use correct naming convention for database eg. 0.png, 1.png .. and so on. You gave, %s"%(envfile))
    self.x_lims = params['x_lims']
    self.y_lims = params['y_lims']

    # resolutions
    self.x_res  = (self.x_lims[1] - self.x_lims[0])/((self.image.shape[1]-1)*1.)
    self.y_res  = (self.y_lims[1] - self.y_lims[0])/((self.image.shape[0]-1)*1.)

    orig_pix_x = math.floor(0 - self.x_lims[0]/self.x_res) #x coordinate of origin in pixel space
    orig_pix_y = math.floor(0 - self.y_lims[0]/self.y_res) #y coordinate of origin in pixel space
    self.orig_pix = (orig_pix_x, orig_pix_y)
    
    self.turning_radius = params['turning_radius']
    self.initialize_kd_tree(self.image)
    self.collision_radius = params['collision_radius']
    
  def initialize_kd_tree(self,img):
    obstacles=np.argwhere(img==0)
    self.kd_tree = KDTree(list(obstacles), leaf_size=2)

  def free_space_volume(self):
    free_space_volume = 0
    for i in range(len(self.image)):
      for j in range(len(self.image[i])):
        free_space_volume += round(self.image[i][j])

    return free_space_volume

  def get_env_image(self):
    return self.image
  
  def set_env_image(self, image):
    self.image = image
    
  def set_params(self, params):
    self.x_lims = params['x_lims']
    self.y_lims = params['y_lims']

    # resolutions
    self.x_res  = (self.x_lims[1] - self.x_lims[0])/((self.image.shape[1]-1)*1.)
    self.y_res  = (self.y_lims[1] - self.y_lims[0])/((self.image.shape[0]-1)*1.)

    orig_pix_x = math.floor(0 - self.x_lims[0]/self.x_res) #x coordinate of origin in pixel space
    orig_pix_y = math.floor(0 - self.y_lims[0]/self.y_res) #y coordinate of origin in pixel space
    self.orig_pix = (orig_pix_x, orig_pix_y)

  def get_random_start_and_goal(self):
    random_start = tuple()
    random_goal = tuple()
    while True:
      random_start = (random.uniform(self.x_lims[0], self.x_lims[1] - 1), random.uniform(self.y_lims[0], self.y_lims[1] - 1))
      if self.collision_free_state(random_start):
        break
    while True:
      random_goal = (random.uniform(self.x_lims[0], self.x_lims[1] - 1), random.uniform(self.y_lims[0], self.y_lims[1] - 1))
      if self.collision_free_state(random_goal):
        break
    return (random_start, random_goal)

  def get_collision_circles_image_coordinates(self, state):
    """ Returns the collision circles centers (in image coordinates) of the vehicle given its state

      @param state - tuple of (x,y,yaw) values in world frame
      @return list of tuples of position in image coordinates
    """
    pix_x, pix_y, yaw = self.to_image_coordinates_continuous(state)
    return [(pix_y, pix_x),
                    (pix_y+math.sin(yaw)*1.5*self.collision_radius, pix_x+math.cos(yaw)*1.5*self.collision_radius),
                    (pix_y+math.sin(yaw)*3*self.collision_radius, pix_x+math.cos(yaw)*3*self.collision_radius)]


  def get_collision_circles(self, state):
    """ Returns the collision circles centers (in world frame) of the vehicle given its state

      @param state - tuple of (x,y,yaw) values in world frame
      @return list of tuples of position in world frame
    """
    (pix_x, pix_y, yaw)=state
    return [(pix_x, pix_y),
                    (pix_x+math.cos(yaw)*1.5*self.collision_radius, pix_y+math.sin(yaw)*1.5*self.collision_radius),
                    (pix_x+math.cos(yaw)*3*self.collision_radius, pix_y+math.sin(yaw)*3*self.collision_radius)]


  def is_inside_boundaries(self, circles_center):
    """ Check if the collision circles are inside the environment

      @param circles_center - tuple of (x,y) values in world frame
      @return True - inside the boundaries
              False - at least one circle outside
    """
    ### TODO ###
    return True
  
  def collision_free_state(self, state):
    """ Check if a state (continuous values) is in collision or not.

      @param state - tuple of (x,y,yaw) values in world frame
      @return 1 - free
              0 - collision
    """
    ### TODO ###
    # circles_center_image_coordinates=self.get_collision_circles_image_coordinates(state)
    # circles_center=self.get_collision_circles(state)
    
    # return is_free
    return True
  
  def collision_free_states(self, states):
    """ Check if every states are collision free.

      @param states - list of tuples of (x,y,yaw) values in world frame
      @return 1 - free
              0 - at least one collision
    """
    ### TODO ###
    # return are_free
    return True

  def to_image_coordinates(self, state):
    """Helper function that returns pixel coordinates for a state in
    continuous coordinates

    @param  - state in continuous world coordinates
    @return - state in pixel coordinates """
    pix_x = int(self.orig_pix[0] + math.floor(state[0]/self.x_res))
    pix_y = int(self.image.shape[1]-1 - (self.orig_pix[1] + math.floor(state[1]/self.y_res)))
    return (pix_x,pix_y)

  def to_image_coordinates_continuous(self, state):
    """Helper function that returns pixel coordinates for a state in
    continuous coordinates

    @param  - state in continuous world coordinates
    @return - state in pixel coordinates """
    pix_x = self.orig_pix[0] + state[0]/self.x_res
    pix_y = self.image.shape[1]-1 - (self.orig_pix[1] + state[1]/self.y_res)
    return (pix_x,pix_y,-state[2])
  
  def to_world_coordinates(self, pix):
    """Helper function that returns world coordinates for a pixel

    @param  - state in continuous world coordinates
    @return - state in pixel coordinates """
    world_x = (pix[0] - self.orig_pix[0])*self.x_res 
    world_y = (pix[1] - self.orig_pix[0])*self.y_res   
    return (world_x, world_y)

  def get_env_lims(self):
    return self.x_lims, self.y_lims

  def initialize_plot(self, start=None, goal=None, grid_res=None, plot_grid=False):
    # if not self.plot_initialized:
    self.figure, self.axes = plt.subplots(figsize=(7,7))
    self.axes.set_xlim(self.x_lims)
    self.axes.set_ylim(self.y_lims)
    if plot_grid and grid_res:
      self.axes.set_xticks(np.arange(self.x_lims[0], self.x_lims[1], grid_res[0]))
      self.axes.set_yticks(np.arange(self.y_lims[0], self.y_lims[1], grid_res[1]))
      self.axes.grid(which='both')
    # self.figure.show() # if it is ON (un-commented), the plot figure is showed up and then disapp ear... 
    self.visualize_environment()
    self.line, = self.axes.plot([],[])
    self.background = self.figure.canvas.copy_from_bbox(self.axes.bbox) 
    if start is not None:
        self.plot_state(start, color='red', edge_color='white', msize=12)
    if goal is not None:
        self.plot_state(goal, color=[0.06, 0.78, 0.78], edge_color='white', msize=12)
    self.figure.canvas.draw()
    self.background = self.figure.canvas.copy_from_bbox(self.axes.bbox) 
    self.plot_initialized = True

  def reset_plot(self, start, goal, grid_res=None):
    if self.plot_initialized:
      plt.close(self.figure) 
      self.initialize_plot(start, goal, grid_res)

  def visualize_environment(self):
    # if not self.plot_initialized:
        #// convert white pixels to transparent pixels
    alpha = ~np.all(self.image2 == 1.0, axis=2) * 255
    rgba = np.dstack((self.image2, alpha)).astype(np.uint8)
    self.axes.imshow(rgba, extent = (self.x_lims[0], self.x_lims[1], self.y_lims[0], self.x_lims[1]), cmap='gray', zorder=1)
    # self.axes.imshow(self.image, extent = (self.x_lims[0], self.x_lims[1], self.y_lims[0], self.x_lims[1]), cmap='gray')


  def plot_edge(self, edge, linestyle='solid', color='blue', linewidth=2):
    x_list = []
    y_list = []
    for s in edge:
      x_list.append(s.q[0])
      y_list.append(s.q[1])
    self.figure.canvas.restore_region(self.background)
    self.line.set_xdata(x_list)
    self.line.set_ydata(y_list)
    self.line.set_linestyle(linestyle)
    self.line.set_linewidth(linewidth)
    self.line.set_color(color)
    self.axes.draw_artist(self.line)
    self.figure.canvas.blit(self.axes.bbox)
    self.background = self.figure.canvas.copy_from_bbox(self.axes.bbox) 

  def plot_pos(self, state, color='red', edge_color='black', alpha=1.0, msize=9):
    """Plot a single state on the environment"""
    # self.figure.canvas.restore_region(self.background)
    self.axes.plot(state[0], state[1], marker='o',  markeredgecolor=edge_color, markersize=msize, color = color, alpha=alpha)
    
    self.figure.canvas.blit(self.axes.bbox)
    self.background = self.figure.canvas.copy_from_bbox(self.axes.bbox)

  def plot_state(self, state, color='red', edge_color='black', alpha=1.0, msize=9, arrow=False):
    """Plot a single state on the environment"""
    # self.figure.canvas.restore_region(self.background)
    # self.figure.canvas.blit(self.axes.bbox)
    # self.background = self.figure.canvas.copy_from_bbox(self.axes.bbox)
    if arrow:
        return self.axes.arrow(state[0], state[1], math.cos(state[2])*msize, math.sin(state[2])*msize, color = color, alpha=alpha, width=msize/6)
    else:
        return self.axes.plot(state[0], state[1], marker='o',  markeredgecolor=edge_color, markersize=msize, color = color, alpha=alpha)


    
  def plot_states(self, states, color='red', edge_color='black', alpha=1.0, msize=9):
    for state in states:
      self.plot_state(state, color=color, edge_color=edge_color, alpha=alpha, msize=msize)   
    
  def plot_path(self, path, color='blue', msize=3, msize_waypoints=0.3):
    self.plot_states(list(zip(*path))[0], color, edge_color=color, msize=msize)
    self.plot_states(sum(list(zip(*path))[1], []), color, edge_color=color, msize=msize_waypoints)
    
  def plot_car(self, state, collision=False):
      self.last_car_plot=[]
      msize=self.collision_radius*2
      if collision:
          arrow=self.axes.arrow(state[0], state[1], math.cos(state[2])*msize, math.sin(state[2])*msize, color='y', width=msize/6)
      else:
          arrow=self.axes.arrow(state[0], state[1], math.cos(state[2])*msize, math.sin(state[2])*msize, color='r', width=msize/6)
      self.last_car_plot.append(arrow)
      circles_center=self.get_collision_circles(state)
      for circle_center in circles_center:
          if collision:
              circle = plt.Circle(circle_center, self.collision_radius, color='y', fill=False, linestyle='dashed', lw=0.7)
          else:
              circle = plt.Circle(circle_center, self.collision_radius, color='r', fill=False, linestyle='dashed', lw=0.7)
          self.last_car_plot.append(self.axes.add_patch(circle))
    
  def animate(self, i, path):
      while self.last_car_plot!=[]:
          self.last_car_plot.pop().remove()
      self.plot_car(path[i])
      # self.last_car_plot.append(self.plot_state(path[i], color="red", edge_color='red', alpha=1.0, msize=9)[0])
    
  def plot_path_video(self, path, file_path, interval):
      self.last_car_plot=[]
      # frames = [] # for storing the generated images
      path2=[]
      for (v,w) in path:
          path2+=w
          path2.append(v)
      ani = animation.FuncAnimation(self.figure, self.animate, interval=100, save_count=len(path2)-1, fargs=(path2,))

      ani.save(file_path)

  def plot_pcolor(self, X, Y, Z, alpha=1.0, cmap='viridis'):
    self.axes.pcolor(X, Y, Z, alpha=alpha, cmap=cmap, shading='auto')

  def plot_title(self, name, fontsize=20):
    self.axes.set_title(name, fontdict = {'fontsize' : fontsize})

  def plot_current(self, tree, q_new, alpha=1.0):
    self.plot_tree(tree, 'dashed', 'blue', 1)
    self.plot_state(q_new, color='pink', alpha=alpha)

  def plot_tree(self, tree, color='blue', msize=3, msize_waypoints=0.3):
    for v in tree: # tree is a dictionary
        self.plot_state(v.q,color, edge_color=color, msize=msize)
        if v.parent is not None:
            self.plot_states(v.path,color, edge_color=color, msize=msize_waypoints)

  def plot_save(self, name):
    plt.savefig(name +'.png')
    plt.close(self.figure) 

  def close_plot(self):
    if self.plot_initialized:
      plt.close(self.figure)
      self.plot_initialized = False

  def clear(self):
    if self.plot_initialized:
      plt.close(self.figure)
      self.plot_initialized = False
    self.image = None
   
    
