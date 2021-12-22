import numpy as np

# REFERENCE: https://www.youtube.com/watch?v=ajv46BSqcK4&t=634s 

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
    '''
    Handles the line case as described in the video. A line is a simplex with two points in it.
    There are three regions where the origin can be but we only need to check two. If AB dot AO > 0, then 
    we use vector rejection to find the closest point 'd' which also gives us the next search direction.  
    '''

    a = simplex[0]
    b = simplex[1]
    a0 = np.negative(a)
    ab = np.subtract(b, a)

    if sameDirection(ab, a0) : 
        # Vector rejection if the dot product is greater than 0
        proj = projection(a0, ab)
        d = np.subtract(a0, proj)       # Closest point to the origin as well as the search direction

    # Otherwise the dot product is less than 0, so we remove point B from the simplex since A will be closer to
    # the origin
    else:
        simplex = [a] 
        d = a0

    return d, simplex 


def doSimplexTriangle(simplex, d):
    '''
    Handles the triangle simplex as described in the video. Only 5 regions need to be verified and
    they all reduce down to line cases. The general idea i.e find points in the search direction of the origin remains 
    the same. 
    '''

    # The three points in the simplex and their respective vectors 
    a = simplex[0]
    b = simplex[1]
    c = simplex[2]
    a0 = np.negative(a)
    ab = np.subtract(b, a)
    ac = np.subtract(c, a)
    abc = np.cross(ab, ac)

    # This is done to handle the edge ase where vector ABC is 0 so that the line case does not perform 
    # a vector rejection on a 0 vector
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
    '''
    This helps to extend the algorithm to 3D. The same idea follows as the triangle case i.e all cases reduce down
    to triangle cases. 
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
    
    return np.zeros(3), simplex


def doSimplex(simplex, d):
    '''
    Chooses which simplex case to perform based on the number of points in the simplex
    '''
    
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
    
    
    if np.array_equal(d, 0):        # If the shapes intersect, then the closest distance between them is 0
        return [True, np.zeros(3), simplex]     # Return the 0 vector
    else:
        return [False, d, simplex]      # Otherwise return the closest distance between the shapes
   
