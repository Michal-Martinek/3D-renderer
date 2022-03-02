import pygame
from pygame.math import Vector3, Vector2
import noise
import math
import functools

from copy import deepcopy

from render import drawTerrainCollored, Triangle2, render

pygame.init()

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
    cameraPos = Vector3(0.1, 0, 10)
    cameraRotation = Vector3(0, 0, 0.01)
    cameraSpeed = Vector3(0., 0., 0.)
    space = False
    airTime = True

    # rendering
    farPlane = 20
    squaresPerRow = 20
    


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    space = True
        
        # controls
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            cameraPos.xy += Vector2(math.cos(cameraRotation.z), math.sin(cameraRotation.z))
        if keys[pygame.K_s]:
            cameraPos.xy -= Vector2(math.cos(cameraRotation.z), math.sin(cameraRotation.z))
        if keys[pygame.K_a]:
            cameraPos.xy -= Vector2(-math.sin(cameraRotation.z), math.cos(cameraRotation.z))
        if keys[pygame.K_d]:
            cameraPos.xy += Vector2(-math.sin(cameraRotation.z), math.cos(cameraRotation.z))
        if keys[pygame.K_e]:
            cameraRotation.z += 0.3 
        if keys[pygame.K_q]:
            cameraRotation.z -= 0.3
        
        if space:
            space = False
            if not airTime:
                cameraSpeed.z = .5
                airTime = True
        
        # # rendering
        fov = 1.
        screenRect3d = [Vector3(fov, -1, 1), Vector3(fov, 1, 1), Vector3(fov, 1, -1), Vector3(fov, -1, -1)]
        closePlanePoints = [rotatePoint(p, cameraPos) for p in screenRect3d]
        farPlanePoints  = deepcopy(closePlanePoints)
        [p.scale_to_length(farPlane) for p in farPlanePoints]
        farPlanePoints = [p + cameraPos for p in farPlanePoints]
        closePlanePoints = [p + cameraPos for p in closePlanePoints]

        minX, maxX = min(closePlanePoints+farPlanePoints, key=lambda p: p.x), max(closePlanePoints+farPlanePoints, key=lambda p: p.x)
        minY, maxY = min(closePlanePoints+farPlanePoints, key=lambda p: p.y), max(closePlanePoints+farPlanePoints, key=lambda p: p.y)

        # for x in range(int(minX.x), int(maxX.x)):
        #     for y in range(int(minY.y), int(minY.y)):
        #         pass


        t1, t2 = getSquareTriangles(int(cameraPos.x), int(cameraPos.y))
        surfaceHeight = max((*t1, *t2), key=lambda p: p.z).z + 1.5
        cameraPos += cameraSpeed
        cameraSpeed.z -= .05

        if not airTime and cameraPos.z <= surfaceHeight + .5:
            cameraPos.z = surfaceHeight
            cameraSpeed.z -= 0.3
        elif cameraPos.z < surfaceHeight:
            cameraPos.z = surfaceHeight
            cameraSpeed.z = 0.
            airTime = False



        squares = []
        for x in range(int(cameraPos.x+farPlane), int(cameraPos.x-farPlane), -1):
            squares.extend( [(x, y) for y in range(int(cameraPos.y-squaresPerRow), int(cameraPos.y+squaresPerRow))] )

        squares.sort(key=lambda s: (cameraPos.x - s[0])**2 + (cameraPos.y - s[1])**2, reverse=True)
        triangles = []
        for s in squares:
            triangles.extend(getSquareTriangles(*s))

        triangles2 = []
        for t in triangles:
            triangles2.append(render(t, cameraPos, cameraRotation, Vector2(screenSize//2)))
        triangles = list(filter(lambda t: t.shouldDraw(screenSize), triangles2))

        display.fill((2,204,254))
        drawTerrainCollored(triangles, display)

        pygame.display.update()
        frameClock.tick(30)
        # print(f'{frameClock.get_fps():.1f}')
    pygame.quit()


if __name__ == '__main__':
    main()