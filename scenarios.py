%load_ext autoreload
%autoreload 2
%matplotlib inline
from motion_planning_utils import *
from matplotlib import pyplot as plt
import numpy as np
from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment, plot_environment, plot_line, plot_poly
#from pyscipopt import *
import json
import scenarios

import random

one_obstacle = {'start': (0,0),
                'goal': [(6,6),(6,7),(7,7),(7,6)],
                'obs_list': [[(3,2), (2,6), (4,5), (6,3)]]
                }


two_obstacle = {'start': (3,0),
                'goal': [(5,6),(5,7),(6,7),(6,6)],
                'obs_list': [[(1,3), (4,1), (4,3)],
                            [(4,6), (3.5,4), (7,3)]],
                }

quad_boxes = {'start': (3.1,0),
              'goal': [(3.5,6.5),(2.5,6.5),(2.5,5.5),(3.5,5.5)],
              'obs_list': [[(1,1), (1,3), (3,3), (3,1)],
                          [(1,3), (1,5), (3,5), (3,3)],
                          [(3,1), (3,3), (5,3), (5,1)],
                          [(3,3), (3,5), (5,5), (5,3)]],
              }

five_obstacle = {'start': (0,0),
                'goal': [(12,12),(12,14),(14,14),(14,12)],
                'obs_list': [[(2,1),(3,1),(3,4),(1,4)],
                            [(5,1),(6,3),(5,5),(4,3)],
                            [(1,6),(6,6),(6,7),(2,9)],
                            [(11,4),(14,5),(8,11),(6,9)],
                            [(5,10),(7,11),(8,14),(4,13)]],
                }

def random_obstacle_sample(num_obstacles):
    #take a random sample of obstacles from the given JSON.
    with open('notamoon.json') as f:
        ex_json_dict = json.load(f)
        
    obstacle_list = []
    choices = []
    num_obs = len(ex_json_dict['obstacles'])
    while(len(choices) != num_obstacles):
        choice = random.randint(0,num_obs-1)
        if(choice not in choices):
            choices.append(choice)
            
    index = 0
    for obs in ex_json_dict['obstacles']:
        if(index in choices):
            obstacle_list.append(np.array(obs['geometry']['coordinates']))
        index = index + 1
    
    d = {'start':(0,0)}
    coord = np.array(ex_json_dict['goal']['geometry']['coordinates'])
    d['goal'] = (coord[:,0],coord[:,1])
    d['obs_list'] = obstacle_list
    
    return d
        



# obs_list = [[(3,2), (2,6), (4,5), (6,3)]]
# obs_list = [[(1,3), (4,1), (4,3)], [(4,6), (3.5,4), (7,3)]]
# obs_list = buff_obs
# obs_list = [[(1,3), (5,2), (4,3)], [(3,6), (3,4), (7,3)]]
# obs_list = [[(1,1), (1,3), (3,3), (3,1)],
#             [(1,3), (1,5), (3,5), (3,3)],
#             [(3,1), (3,3), (5,3), (5,1)],
#             [(3,3), (3,5), (5,5), (5,3)]]
