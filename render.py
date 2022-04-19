from pygame.math import Vector3, Vector2
from pygame import draw
import numpy as np
import math

# TODO: remove?
point2 = Vector2
point3 = Vector3
square = tuple[point3, point3, point3, point3]

triangle3 = tuple[point3, point3, point3]

invalidPoint = Vector2(-1000)

# TODO: remove deprecated functions
# render ------------------------------
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

# NOTE: tris is a np.array of shape (n, 3, 3) not a struct array
def renderTris(tris, cameraPos, cameraRotation, screenSize, closePlaneDistance):
    tris -= np.array( ((cameraPos.xyz)) )
    # rotate
    # TODO: merge the rotations into single function
    tris = rotate_z(tris, -cameraRotation.z)
    tris = rotate_y(tris, -cameraRotation.y)
    tris = rotate_x(tris, -cameraRotation.x)

    behindTris = np.all(tris[:, :, 0] <= 0, axis=1)
    tris[:, :, 0] = np.maximum(tris[:, :, 0], 0.01)
    distances = tris[:, 0, 0] # TODO: better calculation of triangle's distance from camera

    points2D = tris[:, :, 1:]
    points2D *= closePlaneDistance / (tris[:, :, :1])
    screenSize = screenSize / 2
    points2D *= np.array((((screenSize.x, -screenSize.y))))
    points2D += np.array(((screenSize.xy)))

    # remove points which are not in front of the camera
    if np.any( np.isnan(points2D) ):
        raise RuntimeError('renderTris produced result with \'nan\'s')
    return points2D, distances, behindTris
    
# clipping --------------------------------------------
def indep_roll(arr, shifts):
    if arr.shape[0] == 0:
        return arr
    return np.array([np.roll(row, x, axis=0) for row,x in zip(arr, shifts)])

def clipTriangles1Boundary(tris, boundary, axis, boundaryTop=True):
    # count points inside the screen
    if boundaryTop:
        countInside = np.count_nonzero(tris['points'][:, :, axis] < boundary, axis=1)
    else:
        countInside = np.count_nonzero(tris['points'][:, :, axis] > boundary, axis=1)

    # moving the point so that the good points is up front
    oneIn = tris[countInside == 1]['points']
    if boundaryTop:
        inPoints = oneIn[:, :, axis] < boundary
    else:
        inPoints = oneIn[:, :, axis] > boundary
    correctPointIdx = np.nonzero(inPoints) [1]
    oneIn = indep_roll(oneIn, -correctPointIdx)

    # clipping the oneTri
    inPoint = oneIn[:, :1]
    vectors = oneIn[:, 1:] - inPoint
    if boundaryTop:
        vectors *= (boundary - inPoint[:, :, axis:axis+1]) / (vectors[:, :, axis:axis+1])
    else:
        vectors *= (-inPoint[:, :, axis:axis+1]) / (vectors[:, :, axis:axis+1])

    vectors += inPoint
    oneStructs = tris[countInside == 1].copy()
    oneStructs['points'] = np.concatenate((inPoint, vectors), axis=1)

    # two
    twoIn = tris[countInside == 2]['points']
    if boundaryTop:
        outPoints = twoIn[:, :, axis] >= boundary
    else:
        outPoints = twoIn[:, :, axis] <= boundary
    correctPointIdx = np.nonzero(outPoints) [1]
    twoIn = indep_roll(twoIn, -correctPointIdx)

    goodPoints = twoIn[:, 1:]
    vectors = twoIn[:, :1] - goodPoints
    if boundaryTop:
        vectors *= (boundary - goodPoints[:, :, axis:axis+1]) / vectors[:, :, axis:axis+1]
    else:
        vectors *= (-goodPoints[:, :, axis:axis+1]) / vectors[:, :, axis:axis+1]
    vectors += goodPoints

    twoStructs1 = (tris[countInside == 2]).copy()
    twoStructs2 = twoStructs1.copy()
    twoStructs1['points'] = np.concatenate( (twoIn[:, 1:], vectors[:, 1:]), axis=1)
    twoStructs2['points'] = np.concatenate((twoIn[:, 1:2], vectors[:, ::-1]), axis=1)

    out = np.concatenate((tris[countInside == 3], oneStructs, twoStructs1, twoStructs2), axis=0)
    return out
        
def clipTriangles(tris, screenSize):
    tris = clipTriangles1Boundary(tris, screenSize, 1, True)
    tris = clipTriangles1Boundary(tris, screenSize, 0, True)
    tris = clipTriangles1Boundary(tris, 0, 1, False)
    tris = clipTriangles1Boundary(tris, 0, 0, False)
    return tris

# pipeline ---------------------
def renderPipeline(tris, cameraPos, cameraRotation, screenSize, closePlaneDistance):
    tris2D = np.ndarray(tris.shape[0], dtype=[('color', 'u1', 3), ('points', 'f4', (3, 2)), ('camDistance', 'f4')])
    tris2D['color'] = tris['color']
    tris2D['points'], tris2D['camDistance'], behindTris = renderTris(tris['points'], cameraPos, cameraRotation, Vector2((screenSize, screenSize)), closePlaneDistance)
    
    # remove tris marked as behind the cam during rendering 
    tris2D = tris2D[np.logical_not(behindTris)]
    # TODO: don't render triangles which are fully behind another ones
    tris2D = clipTriangles(tris2D, screenSize)
    tris2D[::-1].sort(order='camDistance')
    return tris2D


# draw ----------------------
def drawTerrainCollored(triangles: np.ndarray, display, boundaryColor=(0, 0, 0)):
    points = triangles['points'].tolist()
    colors = triangles['color'].tolist()
    for p, c in zip(points, colors):
        draw.polygon(display, c, p)
        # draw.polygon(display, boundaryColor, p, 1)