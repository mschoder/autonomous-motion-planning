from matplotlib import pyplot as plt
import numpy as np
from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment, plot_environment, plot_line, plot_poly
import pyclipper


def grow_obstacles(obstacles):
    obs = (obstacles)
    pco = pyclipper.PyclipperOffset()
    pco.MiterLimit = 10
    pco.AddPath(pyclipper.scale_to_clipper(obs), pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
    buff_obs = pyclipper.scale_from_clipper(pco.Execute(pyclipper.scale_to_clipper(.2)))
    return buff_obs

def plot_env(obstacles, start, goal_region, bounds, env=Environment(None)):
    env.add_obstacles(obstacles)
    ax = plot_environment(env, bounds=bounds)
    start_point = Point(start).buffer(0.1, resolution=3)
    goal_region = Polygon(goal_region)
    plot_poly(ax, start_point,'magenta')
    plot_poly(ax, goal_region,'green')