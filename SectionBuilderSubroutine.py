import numpy as np


# this program define functions used by Abaqus-Airfoil-Section-Builder
# __author__ = 'Mr.Z'

# return the perpendicular vector(right hand side) of a given vector
def getPerp(vector_in):
    vector_out = [vector_in[1], -vector_in[0]]
    if (type(vector_in) is np.ndarray):
        vector_out = np.array(vector_out)
    return vector_out


# line segment 1 given by endpoints a1, a2
# line segment 2 given by endpoints b1, b2
# return value: [m,p]
#               m = 0 : parallel
#               m = 1 : no intersect
#               m = 2 : an unique intersect
#               p : the coordinates of the intersect point
def intersectSegments2D(a1, a2, b1, b2):
    p = np.zeros_like(a1)
    u = np.asarray(a2) - np.asarray(a1)
    v = np.asarray(b2) - np.asarray(b1)
    w = np.asarray(a1) - np.asarray(b1)
    D = np.cross(u, v)
    # if they are parallel (includes either being a point)
    if abs(D) < 1e-9:  # a and b are parallel
        return 0, p
    else:
        # get the intersect parameter for line 1
        sI = np.cross(v, w) / D
        if sI < 0 or sI > 1:
            return 1, p  # no intersect with line 1
        # get the intersect parameter for line 2
        tI = np.cross(u, w) / D
        if tI < 0 or tI > 1:
            return 1, p  # no intersect with line 2

        p = a1 + sI * u
        return 2, p


# A: n X 2 array of line sequence 1
# B: n X 2 array  of line sequence 2
# return array of intersection points
def intersectSequence2D(A, B):
    m = np.asarray(A).shape[0]  # number of points in the line sequence
    n = np.asarray(B).shape[0]
    intersect_list = []  # empty list to store the intersect points
    for i in range(m - 1):
        point_1 = A[i]
        point_2 = A[i + 1]
        for j in range(n - 1):
            point_3 = B[j]
            point_4 = B[j + 1]
            flag, point = intersectSegments2D(point_1, point_2, point_3, point_4)
            if flag == 2:  # if have intersects
                intersect_list.append(point.tolist())
    return np.array(intersect_list)


#   the airfoil is usually a close curve with or without sharp tail, which is hard
#   to generate the skin layup, so make a cut of the tail part.
#
#   cv_airfoil_o, cv_airfoil_n: old and new airfoil curve
#   cut_ratio:                  between 0~1, suppose the chord length is 1.
#       *****
#                     |
#                     |
#       *****
def airfoilPreprocess(cv_airfoil_o, cut_ratio):
    # decide if the cut_ratio is too small
    if cv_airfoil_o[0, 0] < 1.0 - cut_ratio:
        raise ValueError('The cut ratio is too small, make it bigger!')
    else:
        # find points that need to retain
        loc = np.where(cv_airfoil_o[:, 0] < 1.0 - cut_ratio)
        # find the intersection points between airfoil curve and cut line
        cv_cut = np.array([[1 - cut_ratio, -5], [1 - cut_ratio, 5]])
        intersect_list = intersectSequence2D(cv_cut, cv_airfoil_o)
        if intersect_list.shape[0] != 2:
            raise ValueError('Number of the intersection points is not right!')
        else:
            if intersect_list[1, 1] > intersect_list[0, 1]:  # tell which point is upper
                cv_airfoil_n = np.vstack((intersect_list[0], cv_airfoil_o[loc], intersect_list[1]))
            else:
                cv_airfoil_n = np.vstack((intersect_list[1], cv_airfoil_o[loc], intersect_list[0]))
            return cv_airfoil_n



# movePoints.m
#  move the points in some direction for a distance of delta
# point_old: old points location
# vector:    unit moving direction
# delta :    moving direction
def movePoints(point_old, vector, delta):
    return point_old + delta * np.array(vector)


#   get the unit moving direction
#   v1       : vector BC
#   v2       : vector BA
#               A
#              /
#       C---B
def getDirection(v1, v2):
    vv1 = np.hstack((v1, 0))
    vv2 = np.hstack((v2, 0))
    sinus = np.linalg.norm(np.cross(vv1, vv2)) / np.linalg.norm(vv1) / np.linalg.norm(vv2)
    n1 = np.array([v1[1], -v1[0]])  # normal vector of edge BC
    n2 = v1 / np.linalg.norm(v1) + v2 / np.linalg.norm(v2)
    alpha2 = np.arccos(np.dot(n1, n2) / np.linalg.norm(n1) / np.linalg.norm(n2))
    # decide the offset direction
    if alpha2 > np.pi / 2.0:
        sinus = -sinus

    return n2 / sinus


#	generate the geometry curve of skin,
#   cv_outer: the original curve (outer curve).
#   cv_inner: the curve after apply laminate structure(inner curve).
def generateSkin(cv_outer, thickness, chord):
    n = len(cv_outer)
    cv_inner = np.zeros_like(cv_outer)

    # parser the points in clockwise direction
    for i in range(1, n - 1):
        v1_x, v1_y = cv_outer[i + 1] - cv_outer[i]
        v2_x, v2_y = cv_outer[i - 1] - cv_outer[i]

        v1 = np.array([v1_x, v1_y])
        v2 = np.array([v2_x, v2_y])
        v_direc = getDirection(v1, v2)
        cv_inner[i, :] = movePoints(cv_outer[i], v_direc, thickness * chord)

    # handle the first and last points
    cv_inner[0] = movePoints(cv_outer[0], [0, 1], thickness * chord)  # move upward
    cv_inner[-1] = movePoints(cv_outer[-1], [0, -1], thickness * chord)  # move downward
    if cv_inner[0, 1] > cv_inner[-1, 1]:
        print('Intersection occurs at the tailing edge! Please adjust the skin thickness or the tail cut ratio!')
        # delete the bad points
        C = []
        m = len(cv_inner)
        for i in range(m // 2):
            if cv_inner[i, 1] > cv_inner[m - i - 1, 1]:
                C.append(i)
                C.append(m - i - 1)

        cv_inner = np.delete(cv_inner, C, 0)

    cv_all = np.vstack((cv_outer[:], np.flipud(cv_inner)[:], cv_outer[0]))
    return cv_all, cv_inner

# divide the skin curve by the location of the C-type beam
# into a c-shape curve and two tailing upper and lower curve
def divideSkinbyLines(cv_airfoil, lineF, lineA):
    point_F = intersectSequence2D(cv_airfoil, lineF)
    point_A = intersectSequence2D(cv_airfoil, lineA)
    if len(point_F) != 2 or len(point_A) != 2:
        raise ValueError('More than two intersection points!')
    else:
        index_F = np.argmin(point_F, axis = 0)[1]
        index_A = np.argmax(point_A, axis=0)[1]
        X_F = point_F[index_F][0]
        X_A = point_A[index_A][0]
        loc = np.where(np.logical_or(np.logical_and(cv_airfoil[:, 0] <= X_F, cv_airfoil[:, 1] <= 0),
                         np.logical_and(cv_airfoil[:, 0] <= X_A, cv_airfoil[:, 1] >= 0)))
        # C -shape curve
        cv_airfoil_2 = np.vstack((point_F[index_F], cv_airfoil[loc], point_A[index_A]))
        # lower tail curve
        cv_airfoil_1 = np.vstack((cv_airfoil[0:(loc[0][0]-1)], point_F[index_F]))
        # upper tail curve
        cv_airfoil_3 = np.vstack((point_A[index_A], cv_airfoil[(loc[0][-1]+1):]))

        return cv_airfoil_1, cv_airfoil_2, cv_airfoil_3


# get the area of an polygon
def getPolygonArea(vertices):
    vertice = np.copy(vertices).tolist()
    pairs = zip(vertice, vertice[1:] + vertice[0:1])
    return abs(sum(x1 * y2 - y1 * x2 for (x1, y1), (x2, y2) in pairs)) * 0.5


#   generate the C-Type leading part beam.
#             *********A
#          *        *
#        *       * D
#          *        *
#            ********* F
#   Shape: a 2-D array stores the airfoil inner shape line
#   X_A, X_F: the x coordinates of point A and F
#   X_D, Y_D: the coordinates of point D
#   chord:    the length of airfoil(as a reference value)
#   k_area :    the C type beam's area (percentage of C beam of the whole section)
#   thick  : the offset between control point A, F and the skin.
#   line_1 :  edge DA, it has 4 points, 2 at the end, 2 for adjustments
#   line_2 :  edge DF
def generateCtypeBeam(Shape, C_outer, X_D, Y_D, k_area, thick):

    section_area = getPolygonArea(Shape)

    aim_area = k_area * section_area

    # control points position
    Ax, Ay = C_outer[-1]
    Fx, Fy = C_outer[0]
    point_A = [Ax, Ay - thick]
    point_D = [X_D, Y_D]
    point_F = [Fx, Fy + thick]
    C_outer = np.vstack(([Fx, Fy], C_outer, [Ax, Ay]))
    area_1 = getPolygonArea(C_outer)  # total area of C parts
    # generate adjust points; 2 points between A and D; 2 points between D and F
    x1 = np.linspace(X_D, Ax, 4)
    x2 = np.linspace(X_D, Fx, 4)
    # coordinates of adjust points
    y1 = np.interp(x1, [point_D[0], point_A[0]], [point_D[1], point_A[1]])
    y2 = np.interp(x2, [point_D[0], point_F[0]], [point_D[1], point_F[1]])
    line_1 = np.vstack((x1, y1)).transpose()
    line_2 = np.vstack((x2, y2)).transpose()
    # area of the triangle formed by the control points
    area_2 = getPolygonArea(np.vstack((line_1, np.flipud(line_2))))
    # decide the moving direction of the points
    resid_area = area_1 - area_2 - aim_area
    if resid_area > 0:
        direction = 1.0
    else:
        direction = -1.0
    # get the least distance of adjust points from the shape curve
    d4min = np.linalg.norm(line_2[2] - C_outer[0])
    d3min = np.linalg.norm(line_2[1] - C_outer[0])
    d2min = np.linalg.norm(line_1[1] - C_outer[0])
    d1min = np.linalg.norm(line_1[2] - C_outer[0])
    for i in range(1, len(C_outer) - 1):
        d4 = np.linalg.norm(line_2[2] - C_outer[i])
        d3 = np.linalg.norm(line_2[1] - C_outer[i])
        d2 = np.linalg.norm(line_1[1] - C_outer[i])
        d1 = np.linalg.norm(line_1[2] - C_outer[i])
        if d4 < d4min:
            d4min = d4
        if d3 < d3min:
            d3min = d3
        if d2 < d2min:
            d2min = d2
        if d1 < d1min:
            d1min = d1
    # assign the area for each points to move
    S1 = resid_area * d1min / (d1min + d2min + d3min + d4min)
    S2 = resid_area * d2min / (d1min + d2min + d3min + d4min)
    S3 = resid_area * d3min / (d1min + d2min + d3min + d4min)
    S4 = resid_area * d4min / (d1min + d2min + d3min + d4min)
    # decide the distance need to move
    # for point 1
    v1 = line_1[1] - line_1[3]
    delta1 = S1 * 2 / np.linalg.norm(v1)
    v1_norm = getPerp(v1)
    line_1[2] = line_1[2] + direction * delta1 * v1_norm / (np.linalg.norm(v1_norm))
    # for point 2
    v2 = line_1[0] - line_1[2]
    delta2 = S2 * 2 / np.linalg.norm(v2)
    v2_norm = getPerp(v2)
    line_1[1] = line_1[1] + direction * delta2 * v2_norm / (np.linalg.norm(v2_norm))
    # for point 3
    v3 = line_2[2] - line_2[0]
    delta3 = S3 * 2 / np.linalg.norm(v3)
    v3_norm = getPerp(v3)
    line_2[1] = line_2[1] + direction * delta3 * v3_norm / (np.linalg.norm(v3_norm))
    # for point 4
    v4 = line_2[3] - line_2[1]
    delta4 = S4 * 2 / np.linalg.norm(v4)
    v4_norm = getPerp(v4)
    line_2[2] = line_2[2] + direction * delta4 * v4_norm / (np.linalg.norm(v4_norm))
    # form the C shape
    C_inner = np.vstack(([Ax, Ay], line_1[3], line_1[2], line_1[1], line_2, [Fx, Fy]))
    # decide if the C_inner intersects with the airfoil shape ( > 2 intersection points)
    if len(intersectSequence2D(C_outer, C_inner)) > 2:
        raise ValueError(
            'C-beam inner curve intersects with the outer curve! Please adjust the Point D Location or the Area Ratio!')

    C_shape = np.vstack((C_outer, C_inner))
    return C_shape, C_inner


#  generate the shape of the foam. ( suitable for simply configuration: C_type beam and filler)
#  cv_: short for curve; p_: short for points
#  cv_airfoil : the array stores the coordiantes of airfoil shape points.
#  cv_left, cv_right : the left  and right part control curve of the foam
#         *************************
#       *                                            *
#      *                                                           *
#      *                                             *
#        * *************************
def generateFoam(cv_airfoil, cv_left, cv_right):
    # direction of cv_left  and cv_right are from upper to lower
    p_left_up = cv_left[0]  # left upper corner points
    p_left_down = cv_left[-1]
    p_right_up = cv_right[0]
    p_right_down = cv_right[-1]

    n = len(cv_airfoil)
    m = n / 2
    cv_airfoil_up = cv_airfoil[m + 1:]
    cv_airfoil_down = cv_airfoil[:m]

    loc1 = np.where(np.logical_and(cv_airfoil_up[:, 0] > p_left_up[0], cv_airfoil_up[:, 0] < p_right_up[0]))
    cv_foam_up = np.vstack((p_left_up, cv_airfoil_up[loc1], p_right_up))

    loc2 = np.where(np.logical_and(cv_airfoil_down[:, 0] > p_left_down[0], cv_airfoil_down[:, 0] < p_right_down[0]))
    cv_foam_down = np.vstack((p_right_down, cv_airfoil_down[loc2], p_left_down))

    cv_foam = np.vstack((cv_foam_down, np.flipud(cv_left), cv_foam_up, cv_right))

    return cv_foam, cv_foam_up, cv_foam_down
