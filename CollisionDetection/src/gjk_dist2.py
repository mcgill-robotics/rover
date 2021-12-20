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
    distOrig =  float("inf")

    while (True) :
        a = support(shape1, shape2, d)


        simplex.append(a)

        updatedDistance = doSimplex(simplex, d)[1]
        dist = np.linalg.norm(updatedDistance)

        print(f"Simplex: {simplex}")
        print(f"Distance: {dist}")

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

    return d   

def doSimplexTriangle(simplex, d):
    a = simplex[0]
    b = simplex[1]
    c = simplex[2]
    a0 = np.negative(a)
    ab = np.subtract(b, a)
    ac = np.subtract(c, a)
    abc = np.cross(ab, ac)

    if abc.all() == 0:
        if np.array_equal(a, b):
            simplex = [a, c]
            return doSimplexLine(simplex, d)
        elif np.array_equal(b, c):
            simplex = [a, b]
            return doSimplexLine(simplex, d)
        else:
            simplex = [a, c]
            return doSimplexLine(simplex, d)

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
    else:
        simplex = [a]
        d = a0
        




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
    
    return np.zeros(3)

def doSimplex(simplex, d):
    l = len(simplex)

    #LINE CASE -- 2 POINTS IN THE SIMPLEX
    if l == 2:
        doSimplexLine(simplex, d)
    
    #TRIANGLE CASE -- 3 POINTS IN THE SIMPLEX
    elif l == 3:
        doSimplexTriangle(simplex, d)
    
    #TETRAHEDRON CASE -- 4 POINTS IN THE SIMPLEX
    else:
        doSimplexTetrahedron(simplex, d)
    
    
    if np.array_equal(d, 0):
        return np.zeros(3)
    else:
        return [False, d]
   
