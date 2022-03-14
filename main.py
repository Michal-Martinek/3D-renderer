import pygame
from pygame.math import Vector3, Vector2
import noise
import math
import functools
import time
import numpy as np
from copy import deepcopy

from render import drawTerrainCollored, Triangle2, render, square, point3

# TODO: remove unused functions
@ functools.lru_cache(1000)
def getPointAtCoord(x: int, y: int, tileSize=8):
    val = noise.pnoise2(x / tileSize, y / tileSize, octaves=2)
    # normalize then scale
    val = (val + 0.75) / 1.5
    val *= 8.
    return pygame.math.Vector3(x, y, val)
def rotatePoint(p: Vector3, rot: Vector3):
    return p.rotate_z_rad(rot.z).rotate_y_rad(rot.y).rotate_x_rad(rot.x)

def getSquareTriangles(x, y):
    p1 = getPointAtCoord(x, y)
    p2 = getPointAtCoord(x+1, y)
    p3 = getPointAtCoord(x+1, y+1)
    p4 = getPointAtCoord(x, y+1)
    return (p1, p2, p3), (p1, p3, p4)

def getCameraPlanes(cameraPos: point3, cameraRotation: Vector3, farPlaneDistance: float, closePlaneDistance: float):
    screenRect3d = [Vector3(closePlaneDistance, -1, 1), Vector3(closePlaneDistance, 1, 1), Vector3(closePlaneDistance, 1, -1), Vector3(closePlaneDistance, -1, -1)]
    closePlanePoints = [rotatePoint(p, cameraRotation) for p in screenRect3d]
    farPlanePoints  = [p * (farPlaneDistance / closePlaneDistance) + cameraPos for p in closePlanePoints]
    closePlanePoints = [p + cameraPos for p in closePlanePoints]
    return closePlanePoints, farPlanePoints

def getSquaresVisibleByCamera(closePlane: square, farPlane: square):
    minX, maxX = min(closePlane+farPlane, key=lambda p: p.x), max(closePlane+farPlane, key=lambda p: p.x)
    minY, maxY = min(closePlane+farPlane , key=lambda p: p.y), max(closePlane+farPlane, key=lambda p: p.y)

    squares = []
    for x in range(int(minX.x - 1), int(maxX.x + 1)):
            squares.extend( [(x, y) for y in range(int(minY.y - 1), int(maxY.y + 1))] )
    return squares
def renderSquares(squares: list[square], cameraPos: point3, cameraRotation: Vector3, screenSize: float):
    squares.sort(key=lambda s: (cameraPos.x - s[0])**2 + (cameraPos.y - s[1])**2, reverse=True)
    triangles = []
    for s in squares:
        triangles.extend(getSquareTriangles(*s))

    triangles2 = []
    for t in triangles:
        triangles2.append(render(t, cameraPos, cameraRotation, Vector2(screenSize//2)))
    triangles = list(filter(lambda t: t.shouldDraw(screenSize), triangles2))
    return triangles

# TODO: move these functions to render.py

# TODO: this func should use native numpy ops instead of a for-loop
def getPointsArr(minX, minY, maxX, maxY):
    # [x, y, (x, y, z)] - 3D
    points = np.ndarray((maxX-minX, maxY-minY, 3))
    for x in range(maxX-minX):
        for y in range(maxY-minY):
            points[x, y, :] = getPointAtCoord(x+minX, y+minY).xyz
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
    points = rotate_z(points, -cameraRotation.z)
    points = rotate_y(points, -cameraRotation.y)
    points = rotate_x(points, -cameraRotation.x)

    points2D = points[:, :, 1:]
    inFrontArr = points[:, :, :1] > 0
    points2D *= closePlaneDistance / points[:, :, :1]
    screenSize /= 2
    points2D *= np.array((((screenSize.x, -screenSize.y))))
    points2D += np.array(((screenSize.xy)))

    # remove points which are not in front of the camera
    points2D = points2D * inFrontArr -1000 * (1 - inFrontArr)
    return points2D

def chopPointsIntoTris(points, screenSize):
    tris = []
    for x in range(0, points.shape[0] - 1):
        for y in range(0, points.shape[1] - 1):
            t1 = points[x, y, :], points[x+1, y, :], points[x+1, y+1, :]
            t2 = points[x, y, :], points[x+1, y+1, :], points[x, y+1, :]
            tris.append(Triangle2.fromArr(t1))
            tris.append(Triangle2.fromArr(t2))
    tris = list(filter(lambda t: t.shouldDraw(screenSize), tris))
    return tris

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
    cameraPos = Vector3(0, 0, getPointAtCoord(0, 0).z)
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
                # debug
                elif event.key == pygame.K_p:
                    print(f'cameraPos {cameraPos}\ncameraRotation {cameraRotation}\ncameraPlanes\n{getCameraPlanes(cameraPos, cameraRotation, farPlaneDistance)}')
        
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
        t1, t2 = getSquareTriangles(int(cameraPos.x), int(cameraPos.y))
        surfaceHeight = max((*t1, *t2), key=lambda p: p.z).z + 1.5
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
        
        # new
        minX, maxX = int(min(closePlane+farPlane, key=lambda p: p.x).x), int(max(closePlane+farPlane, key=lambda p: p.x).x) + 1
        minY, maxY = int(min(closePlane+farPlane , key=lambda p: p.y).y), int(max(closePlane+farPlane, key=lambda p: p.y).y)

        points = getPointsArr(minX, minY, maxX, maxY)
        points = renderPointsArr(points, cameraPos, cameraRotation, Vector2((screenSize, screenSize)), closePlaneDistance)
        triangles = chopPointsIntoTris(points, screenSize)
        # TODO: readd colors
            # we would need an array of additional info - original distance from cam, original height
        # TODO: sort the triangles by distance from camera
        # TODO: somehow prevent rendering triangles which are too big
        # TODO: don't render triangles which are fully behind another ones
        # TODO: use numpy for generating triangle from points
        
        triangles = [[(int(p.x), int(p.y)) for p in t.points] for t in triangles]
        drawTerrainCollored(triangles, display)

        pygame.display.update()
        frameClock.tick(30)
        # print(f'{frameClock.get_fps():.1f}')
    pygame.quit()


if __name__ == '__main__':
    pygame.init()
    main()