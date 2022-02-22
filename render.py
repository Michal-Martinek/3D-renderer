from pygame.math import Vector3, Vector2
from pygame import draw

point = Vector3

invalidPoint = Vector2(-1000)

# render ------------------------------
def renderPoint(p: point, cameraPos: point, cameraRotation: Vector3, screenSize: int):
    '''renders point onto screen, if it\'s behind screen returns invalid point'''
    p = p - cameraPos
    p.rotate_z_ip_rad(-cameraRotation.z)
    p.rotate_y_ip_rad(-cameraRotation.y)
    p.rotate_x_ip_rad(-cameraRotation.x)    
    if p.x > 0:
        p = (p / p.x) * (screenSize//2)
        p = Vector2(p.y, -p.z) + Vector2(screenSize//2)
        return p
    return invalidPoint

# TODO: remove
def renderPoints(points: tuple[point], cameraPos: point, cameraRotation: Vector3, screenSize: int):
    return tuple([renderPoint(p, cameraPos, cameraRotation, screenSize) for p in points])

# draw ----------------------
def drawTriangles(triangles: tuple[tuple[point]], display, color=(0, 160, 30), boundaryColor=(0, 0, 0)):
    for t in triangles:
        draw.polygon(display, color, t)
        # draw.polygon(display, boundaryColor, t, 3)

def drawTerrainCollored(triangles: tuple[tuple[point]], display, boundaryColor=(0, 0, 0)):
    for t in triangles:
        color = (0, 100 + int(t.originalHeight * 15), 30)
        # color = (0, 180, 30)
        try:
            draw.polygon(display, color, t.points)
            # draw.polygon(display, boundaryColor, t.points, 1)
        except ValueError: # TODO: this should be handled properly
            print(color)
            exit(1)



# classes ----------------------
class Triangle2:
    def __init__(self, p1: Vector2, p2: Vector2, p3: Vector2, originalHeight=None) -> None:
        self.points = [p1, p2, p3]
        self.originalHeight = originalHeight
    def onScreen(self, screenSize) -> bool:
        return any([0 <= p.x < screenSize + 10 and 0 <= p.y < screenSize + 10 for p in self.points])
    def clockwise(self) -> bool:
        '''returns if the triangle has clockwise order of points (True) or anticlockwise (False)'''
        ab = self.points[1] - self.points[0]
        ac = self.points[2] - self.points[0]
        return ab.cross(ac) >= 0
    def shouldDraw(self, screenSize) -> bool:
        '''returns whether this triangle should be drawn'''
        return self.onScreen(screenSize) and self.clockwise()

class Triangle3:
    def __init__(self, p1: point, p2: point, p3: point) -> None:
        self.points = [p1, p2, p3]
    
    def render(self, cameraPos: point, cameraRotation: tuple[float, float, float], screenSize: int) -> Triangle2:
        p1, p2, p3 = [renderPoint(p, cameraPos, cameraRotation, screenSize) for p in self.points]
        return Triangle2(p1, p2, p3, self.points[0].z)


# old code -----------------------------------
# def renderTriangle(triangle: tuple[point, point, point], cameraPos: point, cameraRotation: tuple[float, float, float], screenSize: int):
#     points = renderPoints(triangle, cameraPos, cameraRotation, screenSize)
#     A = min(points, key=lambda v: v.y)
#     i = points.index(A)
#     B = points[(i + 1) % 3]
#     C = points[(i + 2) % 3]

#     AB = B-A
#     AC = C-A
#     s = AB.x * AC.y >= AB.y * AC.x
#     points = points if s else (invalidPoint, invalidPoint, invalidPoint)



#     return points

# def renderSquare(square: tuple[point, point, point, point], cameraPos: point, cameraRotation: tuple[float, float, float], screenSize: int):
#     return (renderTriangle((square[0], square[1], square[3]), cameraPos, cameraRotation, screenSize), renderTriangle(square[1:], cameraPos, cameraRotation, screenSize))
