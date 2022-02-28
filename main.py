import pygame
from pygame.math import Vector3, Vector2
import noise
import math
import functools

from render import drawTerrainCollored, Triangle3, Triangle2



@ functools.lru_cache(1000)
def getHeightAt(x: int, y: int, tileSize=8):
    val = noise.pnoise2(x / tileSize, y / tileSize)
    # val = val * -10 - 8.
    val = (val + 0.75) / 1.5
    val *= 8.
    # val = -10
    return val

def getPointAtCoord(x, y):
    return pygame.math.Vector3(x, y, getHeightAt(x, y))

def getSquareTriangles(x, y):
    p1, p2, p3, p4 = getPointAtCoord(x, y), getPointAtCoord(x+1, y), getPointAtCoord(x+1, y+1), getPointAtCoord(x, y+1)
    return Triangle3(p1, p2, p3), Triangle3(p1, p3, p4)

def main():
    # pygame
    screenSize = 700
    display = pygame.display.set_mode((screenSize, screenSize), pygame.DOUBLEBUF)
    pygame.event.set_allowed([pygame.QUIT, pygame.KEYDOWN])
    
    fogSurface = pygame.Surface((screenSize, screenSize))
    fogSurface.fill((2,204,254))
    fogSurface.set_alpha(15)

    # camera
    cameraPos = Vector3(0.1, 0, 10)
    cameraRotation = Vector3(0, 0., 0)
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
        

        surfaceHeight = getHeightAt(int(cameraPos.x), int(cameraPos.y)) + 1.5
        cameraPos += cameraSpeed
        cameraSpeed.z -= .05

        # if airTime and cameraPos.z < surfaceHeight:
        #     cameraPos.z = surfaceHeight
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
            triangles2.append(t.render(cameraPos, cameraRotation, screenSize))
        triangles = list(filter(lambda t: t.shouldDraw(screenSize), triangles2))

        display.fill((2,204,254))
        drawTerrainCollored(triangles, display)

        pygame.display.update()
        pygame.time.Clock().tick(30)
    pygame.quit()


if __name__ == '__main__':
    main()