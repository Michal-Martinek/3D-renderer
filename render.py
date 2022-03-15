from pygame.math import Vector3, Vector2
from pygame import draw
import numpy as np

point2 = Vector2
point3 = Vector3
square = tuple[point3, point3, point3, point3]

triangle3 = tuple[point3, point3, point3]

invalidPoint = Vector2(-1000)

# TODO: remove deprecated functions
# render ------------------------------
def renderPoint(p: point3, cameraPos: point3, cameraRotation: Vector3, screenSize: Vector2, fov: float = 1.):
    '''renders point onto screen, if it\'s behind screen returns invalid point'''
    p = p - cameraPos
    p.rotate_z_ip_rad(-cameraRotation.z)
    p.rotate_y_ip_rad(-cameraRotation.y)
    p.rotate_x_ip_rad(-cameraRotation.x)    
    if p.x > 0:
        p /= fov * p.x
        return Vector2(p.y * screenSize.x, p.z * -screenSize.y) + screenSize
    return invalidPoint

# draw ----------------------
def drawTriangles(triangles: list[triangle3], display, color=(0, 160, 30)):
    for t in triangles:
        draw.polygon(display, color, t)
        # draw.polygon(display, boundaryColor, t, 3)
def generateColor(height):
    v = 90 + int(height * 20)
    v = min(max(v, 0), 255)
    return (0, v, 30)
def drawTerrainCollored(triangles: np.ndarray, savedPoints: np.ndarray, display, boundaryColor=(0, 0, 0), screenSize=700):
    triangles = triangles.reshape(triangles.shape[0] * triangles.shape[1] * 2, 3, 2)
    triangles = [[(int(p[0]), int(p[1])) for p in t] for t in triangles.tolist()]
    heights = savedPoints[:-1, :-1, 2]
    heights = np.repeat( heights.reshape(heights.shape[0] * heights.shape[1]), 2)
    triangles = [Triangle2.fromArr(a) for a in triangles]
    for i, t in enumerate(triangles):
        t.color = generateColor(heights[i])
    triangles = list(filter(lambda t: t.shouldDraw(screenSize), triangles))
    for t in triangles:
        draw.polygon(display, t.color, t.toArr())
        draw.polygon(display, boundaryColor, t.toArr(), 1)

# classes ----------------------
class Triangle2:
    @ staticmethod
    def fromArr(a):
        return Triangle2(Vector2(*a[0]), Vector2(*a[1]), Vector2(*a[2]))
    def toArr(self):
        return [(int(p.x), int(p.y)) for p in self.points]

    def __init__(self, p1: Vector2, p2: Vector2, p3: Vector2) -> None:
        self.points = [p1, p2, p3]
    def onScreen(self, screenSize) -> bool:
        p1, p2, p3 = self.points
        b = (-10 <= p1.x < screenSize + 10 and -10 <= p1.y < screenSize + 10)
        b = b or (-10 <= p2.x < screenSize + 10 and -10 <= p2.y < screenSize + 10)
        b = b or (-10 <= p3.x < screenSize + 10 and -10 <= p3.y < screenSize + 10)
        return b
    def clockwise(self) -> bool:
        '''returns if the triangle has clockwise order of points (True) or anticlockwise (False)'''
        ab = self.points[1] - self.points[0]
        ac = self.points[2] - self.points[0]
        return ab.cross(ac) >= 0
    def shouldDraw(self, screenSize: float) -> bool:
        '''returns whether this triangle should be drawn'''
        return self.onScreen(screenSize) and self.clockwise()

def render(points: triangle3, cameraPos: point3, cameraRotation: tuple[float, float, float], screenSize: Vector2) -> Triangle2:
    p1 = renderPoint(points[0], cameraPos, cameraRotation, screenSize)
    p2 = renderPoint(points[1], cameraPos, cameraRotation, screenSize)
    p3 = renderPoint(points[2], cameraPos, cameraRotation, screenSize)
    return Triangle2(p1, p2, p3, points[0].z)
