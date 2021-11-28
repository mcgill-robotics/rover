from math import sqrt

#The logic for manipulating vectors
def add(v1, v2):
    return (v1[0] + v2[0], v1[1] + v2[1])

def sub(v1, v2):
    return (v1[0] - v2[0], v1[1] - v2[1])

def neg(v):
    return (-v[0], -v[1])

def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]

def aXbXa(v1, v2):
    """
    Performs v1 X v2 X v1 where X is the cross product. The
    input vectors are (x, y) and the cross products are
    performed in 3D with z=0. The output is the (x, y)
    component of the 3D cross product.
    """

    x0 = v1[0]
    x1 = v1[1]
    x1y0 = x1 * v2[0]
    x0y1 = x0 * v2[1]
    return (x1 * (x1y0 - x0y1), x0 * (x0y1 - x1y0))

def supportPoly(polygon, direction):
    bestPoint = polygon[0]
    bestDot = dot(bestPoint, direction)

    for i in range(1, len(polygon)):
        p = polygon[i]
        d = dot(p, direction)

        if d > bestDot:
            bestDot = d
            bestPoint = p

    return bestPoint

#finds the furtherst point in the polygon in a given direction
def support(poly1, poly2, direction):
    return sub(supportPoly(poly1, direction), supportPoly(poly2, neg(direction)))


def collide(shape1, shape2):
    s = support(shape1, shape2, (-1, -1))
    simplex = [s]
    d = list(neg(s))

    for i in range(100):
        a = support(shape1, shape2, d)

        if dot(a, d) < 0:
            return False

        simplex.append(a)

        if doSimplex(simplex, d):
            return True

    raise RuntimeError("infinite loop in GJK algorithm")

def doSimplexLine(simplex, d):
    b = simplex[0]
    a = simplex[1]
    a0 = neg(a)
    ab = sub(b, a)
    
    if dot(ab, a0) >= 0:
        cross = aXbXa(ab, a0)
        d[0] = cross[0]
        d[1] = cross[1]
    else:
        simplex.pop(0)
        d[0] = a0[0]
        d[1] = a0[1]    

def doSimplexTriangle(simplex, d):
    c = simplex[0]
    b = simplex[1]
    a = simplex[2]
    a0 = neg(a)
    ab = sub(b, a)
    ac = sub(c, a)

    if dot(ab, a0) >= 0:
        cross = aXbXa(ab, a0)

        if dot(ac, cross) >= 0:
            cross = aXbXa(ac, a0)

            if dot(ab, cross) >= 0:
                return True
            else:
                simplex.pop(1)
                d[0] = cross[0]
                d[1] = cross[1]
        else:
            simplex.pop(0)
            d[0] = cross[0]
            d[1] = cross[1]
    else:
        if dot(ac, a0) >= 0:
            cross = aXbXa(ac, a0)

            if dot(ab, cross) >= 0:
                return True
            else:
                simplex.pop(1)
                d[0] = cross[0]
                d[1] = cross[1]
        else:
            simplex.pop(1)
            simplex.pop(0)
            d[0] = a0[0]
            d[1] = a0[1]

def doSimplex(simplex, d):
    l = len(simplex)

    #LINE CASE -- 2 POINTS IN THE SIMPLEX
    if l == 2:
        return doSimplexLine(simplex, d)
    #TRIANGLE CASE -- 3 POINTS IN THE SIMPLEX
    elif l == 3:
        return doSimplexTriangle(simplex, d)

    return False
    