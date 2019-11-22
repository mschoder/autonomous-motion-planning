from matplotlib import pyplot as plt
import numpy as np
from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment, plot_environment, plot_line, plot_poly
# import pyclipper
from pyscipopt import *
import random



def grow_obstacles(obstacles):
    obs = (obstacles)
    pco = pyclipper.PyclipperOffset()
    pco.MiterLimit = 10
    pco.AddPath(pyclipper.scale_to_clipper(obs), pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
    buff_obs = pyclipper.scale_from_clipper(pco.Execute(pyclipper.scale_to_clipper(.2)))
    return buff_obs

def plot_env(obstacles, start, goal_region, bounds, env=Environment(None)):
    env.add_obstacles([Polygon(obs) for obs in obstacles])
    ax = plot_environment(env, bounds=bounds)
    start = Point(start).buffer(0.1, resolution=3)
    goal_region = Polygon(goal_region)
    plot_poly(ax, start,'magenta')
    plot_poly(ax, goal_region,'green')
    return ax


def find_vertex_avg(obstacle):
    ''' Averages all vertices in a given obstacle. Average of x's and y's is
    guaranteed to lie inside polygon
    '''
    x_avg = sum([x[0] for x in obstacle])/len(obstacle)
    y_avg = sum([x[1] for x in obstacle])/len(obstacle)
    return (x_avg, y_avg)

def process_json_obstacles(json_dict, rand_obs=None):
    obs_list = []
    obs_list_dd = []
    for item in json_dict['obstacles']:
        obs = item['geometry']['coordinates']
        # Remove duplicate points
        obs_dd = []
        [obs_dd.append(x) for x in obs if x not in obs_dd]
        obs_list_dd.append(obs_dd)
        # Original Obstacles
        obs_list.append(obs)
    if rand_obs != None:
        sel = random.sample(range(0, len(json_dict['obstacles'])), rand_obs)
        obs_list = [obs_list[x] for x in sel]
        obs_list_dd = [obs_list_dd[x] for x in sel]
    return obs_list, obs_list_dd


def get_bounds(start,goal,obs_list):
    '''Creates problem environment bounds based on size of inputs'''
    x_max = max(start[0],
                max([g[0] for g in goal]),
                max([v[0] for obs in obs_list for v in obs]))
    x_min = min(start[0],
                min([g[0] for g in goal]),
                min([v[0] for obs in obs_list for v in obs]))
    y_max = max(start[1],
                max([g[1] for g in goal]),
                max([v[1] for obs in obs_list for v in obs]))
    y_min = min(start[1],
                min([g[1] for g in goal]),
                min([v[1] for obs in obs_list for v in obs]))
    xrange = x_max - x_min
    yrange = y_max - y_min
    xb_max = x_max + abs(xrange)*0.1
    xb_min = x_min - abs(xrange)*0.1
    yb_max = y_max + abs(yrange)*0.1
    yb_min = y_min - abs(yrange)*0.1
    return (xb_min,yb_min,xb_max,yb_max)


def create_model(start, goal, obs_list, N, timestep, V_max, buffer):
    '''
    Instantiates gurobi model and builds constraints
    '''
    T = int(N * timestep)
    time = list(range(1,N+1))
    M = 1e6
    vts = (timestep*V_max)**2
    bounds = get_bounds(start, goal, obs_list)

    model = Model()
    # Set x,y decision vars
    x,y,d,z = {},{},{},{}
    for t in range(0,N+1):
        x[t] = model.addVar(lb=bounds[0], ub=bounds[2], name="x(%s)"%(t))
        y[t] = model.addVar(lb=bounds[1], ub=bounds[3], name="y(%s)"%(t))
        d[t] = model.addVar(lb=0, ub=vts, name="d(%s)"%(t))

    # Set initial conditions
    model.addCons(x[0] == start[0])
    model.addCons(y[0] == start[1])
    model.addCons(d[0] == 0)

    # Set velocity constraint for each time period
    for t in time:
        # model.addCons((x[t] - x[t-1])*(x[t] - x[t-1]) + (y[t] - y[t-1])*(y[t] - y[t-1]) <= vts)
        model.addCons(d[t] == ((x[t] - x[t-1])**2 + (y[t] - y[t-1])**2))
        # model.addCons(d[t] == (x[t]**2 -2*x[t-1]*x[t] + x[t-1]**2 + y[t]**2 - 2*y[t-1]*y[t] + y[t-1]**2)**0.5)

    # Set Obstacle Constraints
    for idx_obs, obs in enumerate(obs_list):
        vertex_avg = find_vertex_avg(obs) # known that avg of vertices lies inside convex polygon
        for idx_v, vertex, in enumerate(obs):
            for t in time:
                # Create slack "z" variables
                z[(idx_obs, idx_v, t)] = model.addVar(name="z(%s,%s,%s)"%(idx_obs,idx_v,t), vtype="BINARY")
            vertex1 = obs[idx_v]
            vertex2 = obs[(idx_v + 1)%len(obs)] # grab next vertex and loop back to the first for last index
            delta_x = vertex2[0] - vertex1[0]
            delta_y = vertex2[1] - vertex1[1]
            if delta_x != 0:  # check if line is vertical
                m = delta_y / delta_x
                b = vertex1[1] - m * vertex1[0]
                if vertex_avg[1] < m * vertex_avg[0] + b:  # then flip constraint
                    for t in time:
                        model.addCons(-y[t] <= -m * x[t] - b + M*z[(idx_obs, idx_v, t)] - buffer)
                else:
                    for t in time:
                        model.addCons(y[t] <= m * x[t] + b + M*z[(idx_obs, idx_v, t)] - buffer)
            else:
                if vertex_avg[0] <= vertex1[0]:  # then flip constraint
                    for t in time:
                        model.addCons(-x[t] <= -vertex1[0] + M*z[(idx_obs, idx_v, t)] - buffer)
                else:
                    for t in time:
                        model.addCons(x[t] <= vertex1[0] + M*z[(idx_obs, idx_v, t)] - buffer)
        for t in time:
            model.addCons(quicksum(z[(idx_obs,v,t)] for v in range(len(obs))) <= (len(obs) - 1))

        # Set goal region constraints - final point must be inside goal region
        goal_avg = find_vertex_avg(goal) # known that avg of vertices lies inside convex polygon
        for idx_v, vertex, in enumerate(goal):
            vertex1 = goal[idx_v]
            vertex2 = goal[(idx_v + 1)%len(goal)] # grab next vertex and loop back to the first for last index
            delta_x = vertex2[0] - vertex1[0]
            delta_y = vertex2[1] - vertex1[1]
            if delta_x != 0:  # check if line is vertical
                m = delta_y / delta_x
                b = vertex1[1] - m * vertex1[0]
                if goal_avg[1] > m * goal_avg[0] + b:  # then flip constraint
                    model.addCons(-y[N] <= -m * x[N] - b - buffer)
                else:
                    model.addCons(y[N] <= m * x[N] + b - buffer)
            else:
                if goal_avg[0] > vertex1[0]:  # then flip constraint
                    model.addCons(-x[N] <= -vertex1[0] - buffer)
                else:
                    model.addCons(x[N] <= vertex1[0] - buffer)

    # Set objective function
    model.setObjective(quicksum(d[t] for t in time), "minimize")
    # model.setObjective(quicksum((x[t] - x[t-1])*(x[t] - x[t-1]) + (y[t] - y[t-1])*(y[t] - y[t-1]) for t in time))

    model.hideOutput(quiet=False)
    return model, x, y, d, bounds


def parse_model_path(m,x,y,d):
    '''Takes the constraint dictionary and returns (x, y) coordinate array
    as well as distance array d'''
    path_list = [(m.getVal(x[0]), m.getVal(y[0]))]
    dist = [m.getVal(d[0])]
    for t in range(1,len(x)):
        path_list.append((m.getVal(x[t]), m.getVal(y[t])))
        dist.append(m.getVal(d[t]))
    return path_list, dist
