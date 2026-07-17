import pybullet as p
import math
import random

def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
    def gaussian(x, y, sigma=arena_size/4):
        return mountain_height * math.exp(-((x**2 + y**2) / (2 * sigma**2)))

    for _ in range(num_rocks):
        x = random.uniform(-arena_size / 2, arena_size / 2)
        y = random.uniform(-arena_size / 2, arena_size / 2)
        z = gaussian(x, y)
        size_factor = 1 - (z / mountain_height)
        size = random.uniform(0.1, max_size) * size_factor
        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14) for _ in range(3)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size] * 3)
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size] * 3, rgbaColor=[0.5, 0.5, 0.5, 1])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)

def make_rocks(num_rocks=100, max_size=0.25, arena_size=10):
    for _ in range(num_rocks):
        x = random.uniform(-arena_size / 2, arena_size / 2)
        y = random.uniform(-arena_size / 2, arena_size / 2)
        z = 0.5
        size = random.uniform(0.1, max_size)
        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14) for _ in range(3)])
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size] * 3)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size] * 3, rgbaColor=[0.5, 0.5, 0.5, 1])
        p.createMultiBody(0, shape, visual, [x, y, z], orientation)

def make_arena(arena_size=10, wall_height=1):
    wall_thickness = 0.1
    floor_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[arena_size/2]*2 + [wall_thickness])
    floor_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[arena_size/2]*2 + [wall_thickness], rgbaColor=[1, 1, 0, 1])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision, baseVisualShapeIndex=floor_visual, basePosition=[0, 0, -wall_thickness])

    wall_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2])
    wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7]*3 + [1])

    for y in [arena_size/2, -arena_size/2]:
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision, baseVisualShapeIndex=wall_visual, basePosition=[0, y, wall_height/2])

    wall_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2])
    wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], rgbaColor=[0.7]*3 + [1])

    for x in [arena_size/2, -arena_size/2]:
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision, baseVisualShapeIndex=wall_visual, basePosition=[x, 0, wall_height/2])

def load_terrain(arena_size=40):
    make_arena(arena_size=arena_size)
    mountain_position = (0, 0, -1)
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
    p.setAdditionalSearchPath("shapes/")
    p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)
    # make_mountain()

def load_terrain_hill_and_valleys(arena_size=40):
    make_arena(arena_size=arena_size)
    mountain_position = (0, 0, 0)
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
    p.setAdditionalSearchPath("shapes/")
    p.loadURDF("hills_and_valleys.urdf", mountain_position, mountain_orientation, useFixedBase=1)
    # make_rocks(num_rocks=500, arena_size=10, mountain_height=5)
