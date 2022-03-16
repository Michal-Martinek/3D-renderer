import pygame
from pygame.math import Vector3, Vector2
from vnoise import vnoise
import math
import time
import numpy as np
from copy import deepcopy

from render import drawTerrainCollored, Triangle2, render, square, point3

# TODO: remove unused functions
def rotatePoint(p: Vector3, rot: Vector3):
    return p.rotate_z_rad(rot.z).rotate_y_rad(rot.y).rotate_x_rad(rot.x)

def getCameraPlanes(cameraPos: point3, cameraRotation: Vector3, farPlaneDistance: float, closePlaneDistance: float):
    screenRect3d = [Vector3(closePlaneDistance, -1, 1), Vector3(closePlaneDistance, 1, 1), Vector3(closePlaneDistance, 1, -1), Vector3(closePlaneDistance, -1, -1)]
    closePlanePoints = [rotatePoint(p, cameraRotation) for p in screenRect3d]
    farPlanePoints  = [p * (farPlaneDistance / closePlaneDistance) + cameraPos for p in closePlanePoints]
    closePlanePoints = [p + cameraPos for p in closePlanePoints]
    return closePlanePoints, farPlanePoints

# TODO: move these functions to render.py
def getPointsArr(minX, minY, maxX, maxY, tilesize=8.40, scale=8):
    # [y, x, (x, y, z)] - 3D
    points = np.ndarray(( maxY-minY, maxX-minX, 3))
    points[:, :, 0] = np.reshape( np.tile( np.arange(minX, maxX), maxY-minY), points.shape[:2])
    points[:, :, 1] = np.reshape( np.repeat( np.arange(minY, maxY), maxX-minX), points.shape[:2])
    heights = vnoise.Noise().noise2(np.arange(minY, maxY)/tilesize, np.arange(minX, maxX)/tilesize)
    heights += 0.75
    heights *= scale / 1.5
    points[:, :, 2] = heights
    return points

def rotate_z(points, a):
    newPoints = np.zeros_like(points)
    cosVal, sinVal = math.cos(a), math.sin(a)
    newPoints[:, :, 0] = points[:, :, 0] * cosVal - points[:, :, 1] * sinVal
    newPoints[:, :, 1] = points[:, :, 0] * sinVal + points[:, :, 1] * cosVal
    newPoints[:, :, 2] = points[:, :, 2]
    return newPoints
def rotate_y(points, a):
    newPoints = np.zeros_like(points)
    cosVal, sinVal = math.cos(a), math.sin(a)
    newPoints[:, :, 0] = points[:, :, 0] * cosVal + points[:, :, 2] * sinVal
    newPoints[:, :, 1] = points[:, :, 1]
    newPoints[:, :, 2] = -points[:, :, 0] * sinVal + points[:, :, 2] * cosVal
    return newPoints
def rotate_x(points, a):
    newPoints = np.zeros_like(points)
    cosVal, sinVal = math.cos(a), math.sin(a)
    newPoints[:, :, 0] =  points[:, :, 0]
    newPoints[:, :, 1] = points[:, :, 1] * cosVal - points[:, :, 2] * sinVal
    newPoints[:, :, 2] = points[:, :, 1] * sinVal + points[:, :, 2] * cosVal
    return newPoints
def renderPointsArr(points, cameraPos, cameraRotation, screenSize, closePlaneDistance):
    points -= np.array( ((cameraPos.xyz)) )
    # rotate
    # TODO: is the np.array mutable or not?
    # TODO: merge the rotations into single function
    points = rotate_z(points, -cameraRotation.z)
    points = rotate_y(points, -cameraRotation.y)
    points = rotate_x(points, -cameraRotation.x)
    distances = points [:-1, :-1, 0]
    points2D = points[:, :, 1:]
    behindCamArr = points[:, :, :1] <= 0
    points2D *= closePlaneDistance / (points[:, :, :1] - behindCamArr)
    screenSize = screenSize / 2
    points2D *= np.array((((screenSize.x, -screenSize.y))))
    points2D += np.array(((screenSize.xy)))

    # remove points which are not in front of the camera
    points2D = points2D * (1. - behindCamArr) -1000 * behindCamArr
    return points2D, distances

def chopPointsIntoTris(points):
    tris = np.ndarray((points.shape[0]-1, points.shape[1]-1, 2, 3, 2))
    x = np.repeat(points[:-1, :-1, :], 2, axis=1)
    tris[:, :, :, 0, :] = x.reshape((x.shape[0], x.shape[1]//2, 2, 2))
    tris[:, :, 0, 1, :] = points[:-1, 1:, :]
    tris[:, :, 0, 2, :] = points[1:,  1:, :]
    tris[:, :, 1, 1, :] = points[1:,  1:, :]
    tris[:, :, 1, 2, :] = points[1:, :-1, :]
    return tris.reshape(tris.shape[0] * tris.shape[1] * 2, 3, 2)
def getSurfaceHeight(cameraPos):
    adjacentSquares = getPointsArr(int(cameraPos.x), int(cameraPos.y), int(cameraPos.x)+2, int(cameraPos.y)+2)
    return np.max( adjacentSquares[:, :, 2] ) + 1.5
def main():
    # pygame
    screenSize = 700
    display = pygame.display.set_mode((screenSize, screenSize), pygame.DOUBLEBUF)
    pygame.event.set_allowed([pygame.QUIT, pygame.KEYDOWN])
    frameClock = pygame.time.Clock()

    # TODO: use the fogSurface
    fogSurface = pygame.Surface((screenSize, screenSize))
    fogSurface.fill((2,204,254))
    fogSurface.set_alpha(15)

    # camera
    cameraPos = Vector3(0, 0, 0)
    cameraPos.z = getSurfaceHeight(cameraPos)
    cameraRotation = Vector3(0, 0, 0)
    cameraSpeed = Vector3(0., 0., 0.)
    space = False
    airTime = True

    # rendering
    farPlaneDistance = 20
    closePlaneDistance = 1.

    # controls per second
    cameraMovementSpeed = 5.
    cameraRotationSpeed = 3. # radians
    cameraJumpStartVelocity = 2.5
    gravity = Vector3(0, 0, -2.)
    
    # inner vars
    startFrameTime = time.time()
    running = True
    while running:
        x = startFrameTime
        startFrameTime = time.time()
        numSecsPassed = startFrameTime - x

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    space = True
        
        # controls
        speedScalingFactor = cameraMovementSpeed * numSecsPassed
        moveVectorForward = Vector2(math.cos(cameraRotation.z), math.sin(cameraRotation.z)) * speedScalingFactor
        moveVectorSideways = Vector2(-math.sin(cameraRotation.z), math.cos(cameraRotation.z)) * speedScalingFactor

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            cameraPos.xy += moveVectorForward
        if keys[pygame.K_s]:
            cameraPos.xy -= moveVectorForward
        if keys[pygame.K_a]:
            cameraPos.xy -= moveVectorSideways
        if keys[pygame.K_d]:
            cameraPos.xy += moveVectorSideways
        if keys[pygame.K_e]:
            cameraRotation.z += cameraRotationSpeed * numSecsPassed
        if keys[pygame.K_q]:
            cameraRotation.z -= cameraRotationSpeed * numSecsPassed
        if space:
            space = False
            if not airTime:
                cameraSpeed.z = cameraJumpStartVelocity
                airTime = True

        # camera movement
        surfaceHeight = getSurfaceHeight(cameraPos)
        cameraPos += numSecsPassed * (cameraSpeed + gravity * numSecsPassed/2)
        cameraSpeed += gravity * numSecsPassed

        if not airTime and cameraPos.z <= surfaceHeight + .5:
            cameraPos.z = surfaceHeight
            cameraSpeed += gravity * 6
        elif cameraPos.z < surfaceHeight:
            cameraPos.z = surfaceHeight
            cameraSpeed.z = 0.
            airTime = False

        # rendering
        display.fill((2,204,254))

        closePlane, farPlane = getCameraPlanes(cameraPos, cameraRotation, farPlaneDistance, closePlaneDistance)
        minX, maxX = int(min(closePlane+farPlane, key=lambda p: p.x).x) - 1, int(max(closePlane+farPlane, key=lambda p: p.x).x) + 1
        minY, maxY = int(min(closePlane+farPlane, key=lambda p: p.y).y) - 1, int(max(closePlane+farPlane, key=lambda p: p.y).y) + 1

        points = getPointsArr(minX, minY, maxX, maxY)
        heights = points[:-1, :-1, 2].copy()
        flattenedHeights = np.repeat( heights.reshape(heights.shape[0] * heights.shape[1]), 2)
        # TODO: make the triangles a structured array with height for color / whole color and distance from camera for sorting
        points, distances = renderPointsArr(points, cameraPos, cameraRotation, Vector2((screenSize, screenSize)), closePlaneDistance)
        triangles = chopPointsIntoTris(points)
        
        structTris = np.ndarray(triangles.shape[0], dtype=[('color', 'u4', 3), ('points', 'f8', (3, 2)), ('camDistance', 'f8')])
        structTris['points'] = triangles
        structTris['camDistance'] = np.repeat(distances.flat, 2)
        structTris['color'] = (0, 90, 30)
        structTris['color'][:, 1] += (flattenedHeights * 20).astype('u4')
        structTris['color'][:, :] = np.clip(structTris['color'][:, :], 0, 255)

        onScreen = structTris['points'].copy() # (a > 1) & (a < 5)
        onScreen = (-10 < onScreen) & (onScreen < screenSize + 10)
        onScreen = np.any( np.all(onScreen, axis=2), axis=1)
        structTris = structTris[onScreen]

        structTris[::-1].sort(order='camDistance')
        
        # TODO: somehow prevent rendering triangles which are too big
        # TODO: don't render triangles which are fully behind another ones
        drawTerrainCollored(structTris, display)

        pygame.display.update()
        frameClock.tick(30)
        # print(f'{frameClock.get_fps():.1f}')
    pygame.quit()


if __name__ == '__main__':
    pygame.init()
    main()