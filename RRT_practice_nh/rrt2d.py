#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import reeds_shepp as rs
import decimal

class Vertex():
    def __init__(self,q,parent=None,RS_result=None):
        self.q=q
        self.parent=parent
        if RS_result!=None:
            self.cost=self.parent.cost+RS_result["path_cost"]
            self.path_cost=RS_result["path_cost"]
            self.path=RS_result["path"]
        else:
            self.cost=0
            self.path_cost=0
            self.path=None
        self.children=[]
        
    def update_parent(self, parent, RS_result, reversed_path=False):
        self.parent=parent
        self.cost=self.parent.cost+RS_result["path_cost"]
        self.path_cost=RS_result["path_cost"]
        if reversed_path:
            self.path=list(reversed(RS_result["path"]))
        else:
            self.path=RS_result["path"]
        
    def get_parent(self):
        return self.parent
    
    def get_cost(self):
        return self.cost

class RRT(object): # assume 2d car-like non-holonomic robot
    def __init__(self, q_init, env, extend_len=1, goal_bias_ratio=0.05, waypoint_step_size=1):
        assert decimal.Decimal(str(extend_len))%decimal.Decimal(str(waypoint_step_size))==0, f"extend_len must be a multiple of waypoint_step_size, but {extend_len} is not a multiple of {waypoint_step_size}"
        root=Vertex(q_init)
        self.vertices = [root]
        self.qs = [q_init]
        self.tree=root
        self._env = env
        self._extend_len = extend_len
        self._num_vertices = 1
        self.goal_bias_ratio = goal_bias_ratio
        self.waypoint_step_size = waypoint_step_size
        self._q_goal_set = []
        self._q_best = None
        self._best_cost = math.inf
    
    def get_vertices(self):
        return self.vertices
    

    def is_contain(self, q):
        """ 
          @param q - tuple of (x,y,yaw), a state
          @return True - the state is in the tree 
                  False - not in the tree
        """  
        return q in self.qs
    
    
    def search_nearest_vertex(self, p):
        """ Given a state, returns the nearest vertex in the tree using the reeds shepp distance

          @param p - tuple of (x,y,yaw), a state
          @return Vertex, nearest neighbor
        """  
        ### TODO ###
        # return min_v
        pass
    

    def get_RS_result(self, start, end, turning_radius, step_size):
        """ Returns the result of the reeds shepp steering function between 2 states

          @param start - tuple of (x,y,yaw)
          @param end - tuple of (x,y,yaw)
          @param turning_radius - float
          @param step_size - float, step size between 2 waypoints
          @return Dict
        """  
        ### TODO ###
        # RS_result["path_cost"]= # It should contains the cost (path length) as double
        # RS_result["path"]= # It should contains the list of tuples of (x,y,yaw) along the RS path 
        # return RS_result
        pass
    

    def _calc_new_point(self, near_vertex, q_rand, waypoint_step_size, _extend_len=1.0):
        """ Compute the state using the randomly sampled state and its nearest neighbor
        
          @param near_vertex - Vertex, nearest neighbor
          @param q_rand - tuple of (x,y,yaw), sampled state
          @param waypoint_step_size - float, distance between 2 waypoints
          @param _extend_len - maximum extention length
          @return Dict, reeds shepp steering function result between the nearest neighbor and the new state
          @return tuple of (x,y,yaw), the new state
        """
        ### TODO ###
        # HINT: You should use the get_RS_result function that you code.
        # max_waypoint_number=int(decimal.Decimal(str(_extend_len))//decimal.Decimal(str(waypoint_step_size)))-1
        # q_new=
        # RS_result["path_cost"]=
        # RS_result["path"]=
        # return RS_result, q_new #Dict, tuple
        pass
    

    def add(self, q_new, vertex_near, RS_result):
        """ Creates a new vertex and add it to the tree
        
          @param q_new - tuple of (x,y,yaw), state of the new vertex
          @param vertex_near - Vertex, the parent
          @param RS_result - dict
          @return Vertex, new vertex
        """
        v_new=Vertex(q_new,vertex_near,RS_result)
        vertex_near.children.append(v_new)
        self.vertices.append(v_new)
        self.qs.append(q_new)
        return v_new


    def extend(self, q_rand):
        """ Adds a vertex to the tree given a randomly sampled state
        
          @param q_rand - tuple of (x,y,yaw)
          @return Vertex, new vertex
        """
        ### TODO ###
        # (HINT: You should use the nearest_neighbor, and calc_new_point functions)
        # v_new=self.add(q_new, near_vertex, RS_result)
        # self._num_vertices += 1
        # return v_new
        return None 

    def is_collision(self, p):
        return not self._env.collision_free_state(p)


    def euclidian_distance(self, p1, p2):
        """ Returns the euclidian distance between 2 states
        
          @param p1 - tuple of (x,y,yaw), a state
          @param p2 - tuple of (x,y,yaw), a state
          @return float, the euclidian distance
        """
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    

    def angular_distance(self, p1, p2):
        """ Returns the angular distance between 2 states
        
          @param p1 - tuple of (x,y,yaw), a state
          @param p2 - tuple of (x,y,yaw), a state
          @return float, the angular distance
        """
        return abs(p1[2] - p2[2])


    def is_goal_reached(self, q, goal, goal_region_threshold):
        """ Returns whether a state q is in the goal region
        
          @param q - tuple of (x,y,yaw)
          @param goal - tuple of (x,y,yaw)
          @param goal_region_threshold - float
          @return True - v in goal region
                  False - v not in goal region
        """
        if (0.35*self.euclidian_distance(q, goal))+(0.65*self.angular_distance(q, goal)) <= goal_region_threshold:
            return True
        else:
            return False
    

    def reconstruct_path(self, end):
        """ Returns the path to reach a vertex from the start
        
          @param end - Vertex
          @return list of tuples corresponding to each state of the path, and the list of waypoints to reach it from its parent
        """
        path = []
        v = end
        while v.parent != None:
            path.append((v.q, v.path))
            v = v.get_parent()
        path.append((v.q, []))
        path.reverse()
        return path
    

    def update_best(self):
        """ Finds the best state in the goal region (self._q_goal_set) and updates self._q_best and self._best_cost
        """
        for v in self._q_goal_set:
            new_cost = v.get_cost()
            if new_cost < self._best_cost:
                self._q_best = v
                self._best_cost = new_cost
