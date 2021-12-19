import numpy as np

def findFurthest(polygon, direction):
    bestPoint = polygon[0]
    bestDot = np.dot(bestPoint, direction)

    for i in range(1, len(polygon)):
        p = polygon[i]
        distance = np.dot(p, direction)

        if distance > bestDot:
            bestDot = distance
            bestPoint = p

    return bestPoint

def support(poly1, poly2, direction):
    return np.subtract(findFurthest(poly1, direction), findFurthest(poly2, np.negative(direction)))


def collide(shape1, shape2):
    s = support(shape1, shape2, (1, 0, 0))
    simplex = [s]
    d = list(np.negative(s))

    while (True) :
        a = support(shape1, shape2, d)

        if(np.dot(a, d) <= 0):
            return False; # no collision

        simplex.append(a)

        if doSimplex(simplex, d):
            return True 

def sameDirection(v1, v2):
    return np.dot(v1, v2) > 0

def doSimplexLine(simplex, d):
    a = simplex[0]
    b = simplex[1]
    a0 = np.negative(a)
    ab = np.subtract(b, a)
    
    if sameDirection(ab, a0) : #check is ab and a0 are in same direction
        d = np.cross(np.cross(ab, a0), ab) #abXa0Xab

    else:
        simplex = [a] # remove b from simplex
        d = a0

    return False   

def doSimplexTriangle(simplex, d):
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
   
