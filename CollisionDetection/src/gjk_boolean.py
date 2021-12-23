# Boolean version of the GJK: it only checks whether two convex shapes are colliding with each other or not
# Similar code to the distance version, but only returns a boolean value 


# REFERENCE: https://blog.winter.dev/2020/gjk-algorithm/

import numpy as np

def findFurthest(polygon, direction):
    '''
    Takes in a polygon and a search direction and finds the furthest point in that direction.
    '''

    bestPoint = polygon[0]
    bestDot = np.dot(bestPoint, direction)

    # Loop through all the points in the polygon and update the bestPoint as you go 
    for i in range(1, len(polygon)):
        p = polygon[i]
        distance = np.dot(p, direction)

        if distance > bestDot:
            bestDot = distance
            bestPoint = p

    return bestPoint

def support(poly1, poly2, direction):
    '''
    Finds the furthest point given a search direction for the shape described by the 
    minkowski difference of poly1 and poly2.
    '''

    return np.subtract(findFurthest(poly1, direction), findFurthest(poly2, np.negative(direction)))


def collide(shape1, shape2):
    '''
    Main loop of the program. Takes in two convex shapes: shape1 and shape2 and iteratively
    constructs the minkowski difference between them. The simplex variable will hold the current convex hull.
    Returns true if shape1 and shape2 intersect, othewise returns false.
    '''

    s = support(shape1, shape2, (1, 0, 0))      # Initial search direction of (1, 0, 0) is used. This is picked randomly.
    simplex = [s]
    d = list(np.negative(s))

    while (True) :
        a = support(shape1, shape2, d)

        # no collision
        if(np.dot(a, d) <= 0):
            return False; 

        simplex.append(a)

        # If a simplex is constructed and the origin lies in the simplex, then the shapes intersect
        if doSimplex(simplex, d):
            return True 


def sameDirection(v1, v2):
    '''Returns if vectors v1 and v2 more or less lie in the same direction. In other words,
    return true if v1 dot v2 > 0, otherwise return false.'''

    return np.dot(v1, v2) > 0


def doSimplexLine(simplex, d):
    '''
    Takes in a simplex and search direction d and finds the next point 
    '''

    a = simplex[0]
    b = simplex[1]
    a0 = np.negative(a)
    ab = np.subtract(b, a)
    
    if sameDirection(ab, a0) : #check is ab and a0 are in same direction
        d = np.cross(np.cross(ab, a0), ab) #abXa0Xab (Triple vector product, quick way of determining the next direction)

    else:
        simplex = [a] # remove b from simplex
        d = a0

    return False   


def doSimplexTriangle(simplex, d):
    '''
    Takes in a simplex with 3 points in it with search direction d. There are 8 voronoi regions that need to be 
    checked (where the origin could be), but we reject 3 only verify 5. Regions are verified by checking their dot product
    and constructing a simplex out of those vectors. If no line cases can be formed, we have found the origin in our simplex 
    and we can return true.
    '''

    a = simplex[0]
    b = simplex[1]
    c = simplex[2]
    a0 = np.negative(a)
    ab = np.subtract(b, a)
    ac = np.subtract(c, a)
    abc = np.cross(ab, ac)

    if sameDirection(np.cross(abc, ac), a0):
        if sameDirection(ac, a0):
            simplex = [a, c]
            d = np.cross(np.cross(ac, a0), ac) # acXa0Xac
        
        else:
            return doSimplexLine([a, b], d)
    
    else:
        if sameDirection(np.cross(ab, abc), a0):
            return doSimplexLine([a, b], d)
        
        else:
            if(sameDirection(abc, a0)):
                d = abc
            else:
                simplex = [a, c, b]
                d = np.negative(abc)
    return False


def doSimplexTetrahedron(simplex, d):
    '''
    Takes in a simplex with 4 points and a search direction d. 15 regions require checking but we can eliminate 
    7 and only need to check 8. All these cases can be broken down into triangle cases. Returns true if origin is 
    contained in our simplex i.e no triangle cases can be formed. 
    '''

    a = simplex[0]
    b = simplex[1]
    c = simplex[2]
    d = simplex[3]
    a0 = np.negative(a)
    ab = np.subtract(b, a)
    ac = np.subtract(c, a)
    ad = np.subtract(d, a)

    abc = np.cross(ab , ac)
    acd = np.cross(ac, ad)
    adb = np.cross(ad, ab)

    #if abc is the same direction as a0 then
    if sameDirection(abc, a0):
        newSimplex = [ a,b,c]
        return doSimplexTriangle(newSimplex, d)

    #if acd is the same direction as a0 then
    if sameDirection(acd, a0):
        newSimplex = [ a, c, d]
        return doSimplexTriangle(newSimplex, d)

    #if adb is the same direction as a0 then
    if sameDirection(adb, a0):
        newSimplex = [ a,d,b]
        return doSimplexTriangle(newSimplex, d)
    
    return True

def doSimplex(simplex, d):
    '''
    Chooses which simplex case to perform based on the number of points in the simplex. Takes in the simplex given by the 
    collide() method with an initial search direction d.
    '''

    l = len(simplex)

    #LINE CASE -- 2 POINTS IN THE SIMPLEX
    if l == 2:
        return doSimplexLine(simplex, d)
    
    #TRIANGLE CASE -- 3 POINTS IN THE SIMPLEX
    elif l == 3:
        return doSimplexTriangle(simplex, d)
    
    #TETRAHEDRON CASE -- 4 POINTS IN THE SIMPLEX
    else:
        return doSimplexTetrahedron(simplex, d)
   
