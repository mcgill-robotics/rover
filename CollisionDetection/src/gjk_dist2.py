import numpy as np


def findFurthest(polygon, direction):
    '''
    Takes in a polygon and direction vector and finds the point that is furthest in that direction
    from the center of the polygon
    '''

    bestPoint = polygon[0]
    bestDot = np.dot(bestPoint, direction)

    # Loop through all the points and compute dot products as you go
    # The one which results in the greatest dot product with the direction vector is the furthest point in
    # that direction
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
    minkowski difference of the two polygons
    '''
    return np.subtract(findFurthest(poly1, direction), findFurthest(poly2, np.negative(direction)))


def collide(shape1, shape2):
    '''
    Main loop of the program. Constructs the minkowski difference iteratively and returns the distance between 
    the two convex shapes.
    '''
    s = support(shape1, shape2, (0, 0, 0))

    # The first point calculated by the support is the first point of the simplex described by the minkowski differenc
    simplex = [s]      


    d = list(np.negative(s))          # Search direction 
    distOrig =  float("inf")          # Distance variable that is going to be updates as new distances are found by the loop

    while (True) :
        a = support(shape1, shape2, d)          # Get a new support point in the direction d


        simplex.insert(0, a)           # Insert at front of the simplex so that we use this point 


        # Gets the new direction as well as updates our simplex
        x, d, simplex = doSimplex(simplex, d)       

        # Get the norm of d which is the new distance found by constructing the simplex
        # If the new distance is less than our previous distance, we update and keep the loop running
        # Otherwise, we have found the shortest distance between the two shapes and we return that distance
        dist = np.linalg.norm(d)        
        if dist < distOrig:
            distOrig = dist
        else:
            return dist
        

def sameDirection(v1, v2):
    return np.dot(v1, v2) > 0


def projection(v1, v2):
    '''Gives projection of v1 onto v2'''
    m = 0           
    for val in v2:
        m += val*val
    return np.multiply(np.divide(np.dot(v2, v1), m), v2)

def doSimplexLine(simplex, d):
    a = simplex[0]
    b = simplex[1]
    a0 = np.negative(a)
    ab = np.subtract(b, a)
    if sameDirection(ab, a0) : 

        # Triple product doesn't give info about distance, so we use vector rejection for the line case
        proj = projection(a0, ab)

      
        d = np.subtract(a0, proj)       # Closest point to O

    else:
        simplex = [a] # remove b from simplex
        d = a0

    return d, simplex 


def doSimplexTriangle(simplex, d):
    a = simplex[0]
    b = simplex[1]
    c = simplex[2]
    a0 = np.negative(a)
    ab = np.subtract(b, a)
    ac = np.subtract(c, a)
    abc = np.cross(ab, ac)

 
    if (abc[0] == 0 and abc[1] == 0 and abc[2] == 0):
        if sameDirection(ac, a0):
            simplex = [a, c]
            return doSimplexLine(simplex, d)
        elif sameDirection(ab, a0):
            simplex = [a, b]
            return doSimplexLine(simplex, d)
        else:
            simplex = [a]
            return a0

    # REGION 1
    if sameDirection(np.cross(abc, ac), a0) and sameDirection(ac, a0):
        simplex = [a, c]
        return doSimplexLine(simplex, d)

    # REGION 4
    elif sameDirection(np.cross(abc, ac), a0) and sameDirection(ab, a0):
        simplex = [a, b]
        return doSimplexLine(simplex, d)
    elif sameDirection(np.cross(ab, abc), a0) and sameDirection(ab, a0):
        simplex = [a, b]
        return doSimplexLine(simplex, d)
    
    # REGIONS 2 and 3
    elif not sameDirection(np.cross(abc, ac), a0) and not sameDirection(np.cross(ab, abc), a0):
        d = projection(a0, abc)
        if sameDirection(d, abc):
            simplex = [a, b, c]
        else:
            simplex = [a, c, b]
        return d, simplex
    else:
        simplex = [a]
        d = a0
        return d, simplex
        

def doSimplexTetrahedron(simplex, d):
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
    
    return np.zeros(3), simplex

def doSimplex(simplex, d):
    l = len(simplex)

    #LINE CASE -- 2 POINTS IN THE SIMPLEX
    if l == 2:
        d, simplex = doSimplexLine(simplex, d)
    
    #TRIANGLE CASE -- 3 POINTS IN THE SIMPLEX
    elif l == 3:
        d, simplex = doSimplexTriangle(simplex, d)
    
    #TETRAHEDRON CASE -- 4 POINTS IN THE SIMPLEX
    else:
        d, simplex = doSimplexTetrahedron(simplex, d)
    
    
    if np.array_equal(d, 0):
        return [True, np.zeros(3), simplex]
    else:
        return [False, d, simplex]
   
