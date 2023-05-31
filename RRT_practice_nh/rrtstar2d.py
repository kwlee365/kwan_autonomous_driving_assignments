# -*- coding: utf-8 -*-
import math
from rrt2d import RRT as RRTBase

class RRTstar(RRTBase):
    def __init__(self, q_init, env, extend_len=1.0, waypoint_step_size=1, dimension=2):
        super(RRTstar, self).__init__(q_init, env, extend_len=extend_len, waypoint_step_size=waypoint_step_size)
        self._dimension = dimension
        self._gamma = 2.0 * math.pow(1.0 + 1.0 / self._dimension, 1.0 / self._dimension) \
                         * math.pow(self._env.free_space_volume() / math.pi, 1.0 / self._dimension)
        self._near_distance = 0


    def valid_near(self, v_new, vertex_set):
        """ Given a Vertex v_new, return all vertices reachable within a distance <= _near_distance, along with the reeds shepp steering function result.

          @param v_new - Vertex, the newly added vertex
          @param vertex_set - List of vertices in the tree
          @return list of tuples (Vertex v, Dict RS_result)
        """
        ### TODO ###
        # return v_near
        pass

    def update_cost_R(self, v):
        for v_child in v.children:
            v_child.cost=v_child.parent.cost+v_child.path_cost
            self.update_cost_R(v_child)

    def change_parent(self, v_parent, v_child, RS_result, reversed_path=False):
        """ Replace the previous parent of v_child by v_parent

          @param v_parent - Vertex, the new parent
          @param v_child - Vertex, the child
          @param RS_result - Dict, results of the steering function between v_parent and v_child
          @param reversed_path - bool, if True reverse RS_result["path"] when saving it into v_child 
        """
        prev_parent = v_child.parent
        v_child.update_parent(v_parent, RS_result, reversed_path)
        v_parent.children.append(v_child)
        prev_parent.children.remove(v_child)
        self.update_cost_R(v_child)


    def rewire(self, v_new):
        """ Updates _near_distance and rewire the whole tree given a vertex

          @param v_new - Vertex, the newly added vertex
        """
        ### TODO ###
        pass
                
    



