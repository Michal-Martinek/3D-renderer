import pygame
from pygame.math import Vector3, Vector2
from vnoise import vnoise
import math
import time
import numpy as np

from render import drawTerrainCollored, renderPipeline, point3


def rotatePoint(p: Vector3, rot: Vector3):
    return p.rotate_z_rad(rot.z).rotate_y_rad(rot.y).rotate_x_rad(rot.x)
# TODO: migrate from pygame.Vector to numpy everywhere
def getCameraPlanes(cameraPos: point3, cameraRotation: Vector3, farPlaneDistance: float, closePlaneDistance: float):
    screenRect3d = [Vector3(closePlaneDistance, -1, 1), Vector3(closePlaneDistance, 1, 1), Vector3(closePlaneDistance, 1, -1), Vector3(closePlaneDistance, -1, -1)]
    closePlanePoints = [rotatePoint(p, cameraRotation) for p in screenRect3d]
    farPlanePoints  = [p * (farPlaneDistance / closePlaneDistance) + cameraPos for p in closePlanePoints]
    closePlanePoints = [p + cameraPos for p in closePlanePoints]
    return closePlanePoints, farPlanePoints

vnoiseObj = vnoise.Noise()
def terrainGenerator(minX, minY, maxX, maxY, tilesize=8.40, scale=8):
    heights = vnoiseObj.noise2(np.arange(minY, maxY)/tilesize, np.arange(minX, maxX)/tilesize)
    heights += 0.75
    heights *= scale / 1.5
    return heights
def getPointsArr(minX, maxX, minY, maxY):
    points = np.ndarray(( maxY-minY, maxX-minX, 3))
    points[:, :, 0] = np.reshape( np.tile( np.arange(minX, maxX), maxY-minY), points.shape[:2])
    points[:, :, 1] = np.reshape( np.repeat( np.arange(minY, maxY), maxX-minX), points.shape[:2])
    points[:, :, 2] = terrainGenerator(minX, minY, maxX, maxY)
    return points

def chopPointsIntoTris(points):
    tris = np.ndarray((points.shape[0]-1, points.shape[1]-1, 2, 3, 3))
    x = np.repeat(points[:-1, :-1], 2, axis=1)
    tris[:, :, :, 0, :] = x.reshape((x.shape[0], x.shape[1]//2, 2, 3))
    tris[:, :, 0, 1, :] = points[:-1, 1:, :]
    tris[:, :, 0, 2, :] = points[1:,  1:, :]
    tris[:, :, 1, 1, :] = points[1:,  1:, :]
    tris[:, :, 1, 2, :] = points[1:, :-1, :]
    return tris.reshape(tris.shape[0] * tris.shape[1] * 2, 3, 3)
def getVisibleMapSquare(cameraPos, cameraRotation, farPlaneDistance, closePlaneDistance):
    closePlane, farPlane = getCameraPlanes(cameraPos, cameraRotation, farPlaneDistance, closePlaneDistance)
    minX, maxX = int(min(closePlane+farPlane, key=lambda p: p.x).x) - 1, int(max(closePlane+farPlane, key=lambda p: p.x).x) + 1
    minY, maxY = int(min(closePlane+farPlane, key=lambda p: p.y).y) - 1, int(max(closePlane+farPlane, key=lambda p: p.y).y) + 1
    return (minX, maxX, minY, maxY)
def generateColor(tris, baseColor=(0, 90, 30), scale=20):
    heights = tris['points'][:, 0, 2]
    tris['color'] = baseColor
    tris['color'][:, 1] += (heights * scale).astype('u1')
    return tris
def constructTriangles(points):
    triangles = chopPointsIntoTris(points)
    tris = np.ndarray(triangles.shape[0], dtype=[('color', 'u1', 3), ('points', 'f4', (3, 3))])
    tris['points'] = triangles
    generateColor(tris)
    return tris

def getTriangles(cameraPos, cameraRotation, farPlaneDistance, closePlaneDistance):
    boundaries = getVisibleMapSquare(cameraPos, cameraRotation, farPlaneDistance, closePlaneDistance)    
    points = getPointsArr(*boundaries)
    return constructTriangles(points)
# TODO: merge this into the rendering so we don't have to call vnoise twice
def getSurfaceHeight(cameraPos, playerHeight=1.5):
    adjacentSquares = getPointsArr(int(cameraPos.x), int(cameraPos.x)+2, int(cameraPos.y), int(cameraPos.y)+2)
    cameraFractX = cameraPos.x % 1
    cameraFractY = cameraPos.y % 1
    a = adjacentSquares[0, :, 2]
    b = adjacentSquares[1, :, 2]
    newA, newB = cameraFractY * (b - a) + a
    height = cameraFractX * (newB - newA) + newA
    return height + playerHeight


def main():
    # pygame
    screenSize = 700
    display = pygame.display.set_mode((screenSize, screenSize), pygame.DOUBLEBUF)
    pygame.event.set_blocked(None)
    pygame.event.set_allowed([pygame.QUIT, pygame.KEYDOWN, pygame.KEYUP])
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
    airTime = True

    # rendering
    farPlaneDistance = 20
    closePlaneDistance = 1.

    # controls per second
    cameraMovementSpeed = 5.
    cameraRotationSpeed = 3. # radians
    cameraJumpStartVelocity = 2.5
    gravity = Vector3(0, 0, -2.)

    keysDown = {x: False for x in [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, pygame.K_q, pygame.K_e, pygame.K_SPACE]}
    
    # inner vars
    startFrameTime = time.time()
    running = True
    while running:
        temp = startFrameTime
        startFrameTime = time.time()
        numSecsPassed = startFrameTime - temp

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, pygame.K_q, pygame.K_e, pygame.K_SPACE]:
                    keysDown[event.key] = True
                if event.key == pygame.K_p: # debug
                    print('camera pos:', cameraPos, '\ncamera rot:', cameraRotation)
            elif event.type == pygame.KEYUP:
                if event.key in [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, pygame.K_q, pygame.K_e, pygame.K_SPACE]:
                    keysDown[event.key] = False
        
        # controls
        speedScalingFactor = cameraMovementSpeed * numSecsPassed
        moveVectorForward = Vector2(math.cos(cameraRotation.z), math.sin(cameraRotation.z)) * speedScalingFactor
        moveVectorSideways = Vector2(-math.sin(cameraRotation.z), math.cos(cameraRotation.z)) * speedScalingFactor

        if keysDown[pygame.K_w]:
            cameraPos.xy += moveVectorForward
        if keysDown[pygame.K_s]:
            cameraPos.xy -= moveVectorForward
        if keysDown[pygame.K_a]:
            cameraPos.xy -= moveVectorSideways
        if keysDown[pygame.K_d]:
            cameraPos.xy += moveVectorSideways
        if keysDown[pygame.K_q]:
            cameraRotation.z -= cameraRotationSpeed * numSecsPassed
        if keysDown[pygame.K_e]:
            cameraRotation.z += cameraRotationSpeed * numSecsPassed
        if keysDown[pygame.K_SPACE]:
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
        tris = getTriangles(cameraPos, cameraRotation, farPlaneDistance, closePlaneDistance)
        tris2D = renderPipeline(tris, cameraPos, cameraRotation, screenSize)

        # drawing
        display.fill((2,204,254))
        drawTerrainCollored(tris2D, display)

        pygame.display.update()
        frameClock.tick(30)
        # print(f'{frameClock.get_fps():.1f}')


if __name__ == '__main__':
    pygame.init()
    main()
    pygame.quit()