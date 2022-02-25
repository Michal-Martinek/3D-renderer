import pygame
from pygame.math import Vector3, Vector2
import noise
import math
import functools

from render import drawTerrainCollored, Triangle3



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
    points = [getPointAtCoord(x1, y1) for (x1, y1) in ((x, y), (x+1, y), (x+1, y+1), (x, y+1))]
    return Triangle3(*points[:3]), Triangle3(points[0], points[2], points[3])

def main():
    screenSize = 700
    display = pygame.display.set_mode((screenSize, screenSize))
    fogSurface = pygame.Surface((screenSize, screenSize))
    fogSurface.fill((2,204,254))
    fogSurface.set_alpha(15)

    # camera
    cameraPos = Vector3(0.1, 0, 10)
    cameraRotation = Vector3(0, 0., 0)
    cameraSpeed = Vector3(0., 0., 0.)
    farPlane = 20
    squaresPerRow = 20

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    pass
                elif event.key == pygame.K_s:
                    pass
        
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
            cameraRotation.z += 0.1 
        if keys[pygame.K_q]:
            cameraRotation.z -= 0.1
        if keys[pygame.K_SPACE]:
            cameraSpeed.z = .5

        surfaceHeight = getHeightAt(int(cameraPos.x), int(cameraPos.y)) + 1.5
        cameraPos += cameraSpeed
        cameraSpeed.z -= .05
        if (cameraPos.z) < surfaceHeight:
            cameraPos.z = surfaceHeight
            cameraSpeed.z = 0.
        # cameraBoundarySide = Vector2(0, squaresPerRow//2).rotate_rad(cameraRotation.z)
        # cameraBoundaryFront = Vector2(farPlane, 0).rotate_rad(cameraRotation.z)
        # cameraBoundary = (cameraPos.xy - cameraBoundarySide + cameraBoundaryFront, 
        #                     cameraPos.xy + cameraBoundarySide + cameraBoundaryFront, 
        #                     cameraPos.xy + cameraBoundarySide, 
        #                 cameraPos.xy - cameraBoundarySide)
        # squares = []
        # Xs, Ys = [v.x for v in cameraBoundary], [v.y for v in cameraBoundary]
        # for x in range(min(Xs), max(Xs) + 1):
        #     for y in range(min(Ys), max(Ys) + 1):
        #         pass

        squares = []
        for x in range(int(cameraPos.x+farPlane), int(cameraPos.x-farPlane), -1):
            squares.extend( [(x, y) for y in range(int(cameraPos.y-squaresPerRow), int(cameraPos.y+squaresPerRow))] )

        squares.sort(key=lambda s: (cameraPos.x - s[0])**2 + (cameraPos.y - s[1])**2, reverse=True)
        squares = map(lambda s: getSquareTriangles(*s), squares)
        triangles = []
        [triangles.extend(s) for s in squares]

        triangles = map(lambda t: t.render(cameraPos, cameraRotation, screenSize), triangles)
        triangles = list(filter(lambda t: t.shouldDraw(screenSize), triangles))

        display.fill((2,204,254))
        drawTerrainCollored(triangles, display)

        pygame.display.update()
        pygame.time.Clock().tick(20)
    pygame.quit()


if __name__ == '__main__':
    main()