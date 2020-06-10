#!/usr/bin/python3
import math

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Given three colinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
def onSegment(p, q, r):
    if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
           (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
        return True
    return False

#check orientation of the ordered tripplet
def orientation(p, q, r):

    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
    if (val > 0):

        # Clockwise orientation
        return 1
    elif (val < 0):

        # Counterclockwise orientation
        return 2
    else:

        # Colinear orientation
        return 0

#function to check crossing between hippo and hound path. A path is from one node to another node.
# The main function that returns true if the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1,q1,p2,q2):

    # Find the 4 orientations required for
    # the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True

    # Special Cases

    # p1 , q1 and p2 are colinear and p2 lies on segment p1q1
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        return True

    # p1 , q1 and q2 are colinear and q2 lies on segment p1q1
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True

    # p2 , q2 and p1 are colinear and p1 lies on segment p2q2
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True

    # p2 , q2 and q1 are colinear and q1 lies on segment p2q2
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True

    # If none of the cases
    return False

#p1 is from hound, p2 is from hippo.
#return true if for center p1 of hound, center p2 of hippo, then collide
def check_two_circles_intersect(p1,radius1,p2,radius2):
    if pow(radius1 - radius2,2) <= (pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2)) <= pow(radius1 + radius2,2):
        return True
    else:
        return False

#it is a line segment
def check_line_intersect_circle(p1,p2,center,radius):
    ax = p1.x
    ay = p1.y
    bx = p1.x
    by = p1.y
    cx = center.x
    cy = center.y
    r = radius

    ax = ax - cx
    ay = ay - cy
    bx = bx - cx
    by = by - cy
    a = pow((bx - ax),2) + pow((by - ay),2)
    b = 2*(ax*(bx - ax) + ay*(by - ay))
    c = pow(ax,2) + pow(ay,2) - pow(r,2)
    disc = pow(b,2) - 4*a*c
    if disc <= 0:
        return False
    sqrtdisc = math.sqrt(disc)
    t1 = (-b + sqrtdisc)/(2*a)
    t2 = (-b - sqrtdisc)/(2*a)
    if (0 < t1 and t1 < 1) or (0 < t2 and t2 < 1):
        return True
    return False


def point_line_intersect_circles(m,d,center,r):
    a,b = center.x, center.y
    alpha = pow(r,2)*(1+pow(m,2)) - pow(b-m*a-d,2)
    if alpha < 0 or alpha == 0:
        print("error")
    x1,x2 = (a+b*m-d*m+math.sqrt(alpha))/(1+pow(m,2)), (a+b*m-d*m-math.sqrt(alpha))/(1+pow(m,2))
    y1,y2 = (d+a*m+b*pow(m,2)+m*math.sqrt(alpha))/(1+pow(m,2)), (d+a*m+b*pow(m,2)-m*math.sqrt(alpha))/(1+pow(m,2))
    return Point(x1,y1), Point(x2,y2)

def sgn(x):
    if x < 0:
        return -1
    else:
        return 1

def between_value(a,b,c):
    return a <= b <= c

def point_lines_intersect(m1,b1,m2,b2):
    x = (b2-b1)/(m1-m2)
    y = m1 * x + b1
    return x,y

def isinsiderect(p1,p2,A,B,C,D,center):
    cx, cy = center.x, center.y
    if (A.y - B.y) == 0 or (C.y - D.y) == 0:
        #vertical rectangle
        if (between_value(A.x,cx,B.x) or between_value(B.x,cx,A.x)) and (between_value(A.y,cy,C.y) or between_value(C.y,cy,A.y)) :
            return True
        else:
            return False
    if (A.x - B.x == 0) or (C.x - D.x == 0):
        #horizontal rectangle
        if (between_value(A.x,cx,C.x) or between_value(C.x,cx,A.x)) and (between_value(A.y,cy,B.y) or between_value(B.y,cy,A.y)) :
            return True
        else:
            return False
    my_m = (p2.y-p1.y)/(p2.x-p1.x)
    my_b = cy - my_m*cx
    sectAB_m = -1/my_m
    sectAB_b = p1.y - sectAB_m*p1.x
    sectCD_m = -1/my_m
    sectCD_b = p2.y - sectCD_m*p2.x
    if sectAB_m - my_m == 0 or sectCD_m - my_m == 0:
        #parallel, not intersect
        return False
    x1,y1 = point_lines_intersect(my_m,my_b,sectAB_m,sectAB_b)
    x2,y2 = point_lines_intersect(my_m,my_b,sectCD_m,sectCD_b)
    if between_value(A.x,x1,B.x) or between_value(B.x,x1,A.x) or between_value(C.x,x2,D.x) or between_value(D.x,x2,C.x):
        sectBC_m = (C.y-B.y)/(C.x-B.x)
        if sectBC_m == my_m:
            sectBC_b = B.y - sectBC_m * B.x
            x3,y3 = point_lines_intersect(sectBC_m,sectBC_b,-1/my_m, (cy - (-1/my_m)*cx))
            if between_value(B.x,x3,C.x) or between_value(C.x,x3,B.x):
                return True
        else:
            sectBD_m = my_m
            sectBD_b = B.y - sectBD_m * B.x
            x3, y3 = point_lines_intersect(sectBD_m,sectBD_b,-1/my_m, (cy - (-1/my_m)*cx))
            if between_value(B.x,x3,D.x) or between_value(D.x,x3,B.x):
                return True
    return False



def clic(p1,p2,center,r):
    #print("original : ",p1.x, " ", p1.y, " ", p2.x, " ", p2.y," ")
    p1x, p1y, p2x, p2y = p1.x, p1.y, p2.x, p2.y
    p1x = p1x - (center.x)
    p1y = p1y - (center.y)
    p2x = p2x - (center.x)
    p2y = p2y - (center.y)
    dx = p2x - p1x
    dy = p2y - p1y
    dr = math.sqrt(dx*dx+dy*dy)
    D = p1x*p2y - p2x*p1y
    discriminant = r*r*dr*dr - D*D
    if discriminant < 0:
        return False
    else:
        x1 = (D*dy+sgn(dy)*dx*math.sqrt(discriminant))/(dr*dr)
        x2 = (D*dy-sgn(dy)*dx*math.sqrt(discriminant))/(dr*dr)
        y1 = (-D*dx+abs(dy)*math.sqrt(discriminant))/(dr*dr)
        y2 = (-D*dx-abs(dy)*math.sqrt(discriminant))/(dr*dr)
    #print(x1," ", y1, " ", x2, " ", y2)
    #print(p1x, " ", p1y, " ", p2x, " ", p2y, " ")

    if p1x == p2x:
        if (between_value(p1y,y1,p2y) or between_value(p2y,y1,p1y) or between_value(p1y,y2,p2y) or between_value(p2y,y2,p1y)):
            return True
        return False
    elif p1y == p2y:
        if (between_value(p1x,x1,p2x) or between_value(p2x,x1,p1x) or between_value(p1x,x2,p2x) or between_value(p2x,x2,p1x)):
            return True
        return False
    else:
        if (between_value(p1x,x1,p2x) or between_value(p2x,x1,p1x) or between_value(p1x,x2,p2x) or between_value(p2x,x2,p1x)):
            return True
        return False

    return False

#check if a rectangle intersect with a stationed circle
def intersect(P,R,A,B,C,D,p1,p2):
    if isinsiderect(p1,p2,A,B,C,D,P) or clic(A,B,P,R) or clic(B,C,P,R) or clic(C,D,P,R) or clic(B,D,P,R) or clic(A,C,P,R) or clic(A,D,P,R):
       return True
    return False



def get_rectangle_points(p1,p2,r):
    if (p2.x - p1.x) == 0:
        #verticle movement
        A = Point(p1.x+r,p1.y)
        B = Point(p1.x-r,p1.y)
        C = Point(p2.x+r,p2.y)
        D = Point(p2.x-r,p2.y)
        return A,B,C,D
    else:
        m = (p2.y-p1.y)/(p2.x-p1.x)
        if m == 0:
            A = Point(p1.x,p1.y+r)
            B = Point(p1.x,p1.y-r)
            C = Point(p2.x,p2.y+r)
            D = Point(p2.x,p2.y-r)
            return A,B,C,D
        else:
            m1 = -1/m
            b1 = p1.y - m1*p1.x
            #print("m1 = ",m1, "b1 = ",b1)
            A, B = point_line_intersect_circles(m1,b1,p1,r)
            m2 = m1
            b2 = p2.y - m2*p2.x
            #print("m2 = ", m2, "b2 = ",b2)
            C, D = point_line_intersect_circles(m2,b2,p2,r)
            return A,B,C,D

#robot 1 is moving, while robot 2 is stationary. check if a rectangle overlap the circle or not.
def check_robot_is_moving(p1,p2,radius1,center,radius2):
    A,B,C,D = get_rectangle_points(p1,p2,radius1)
    #print("my A, B, C, D: ", A.x," ",A.y, " ", B.x, " ", B.y, " ", C.x, ' ', C.y, ' ', D.x, ' ', D.y)
    #clic(B,C,center,radius2)
    return intersect(center,radius2, A,B,C,D,p1,p2)


def check_if_two_path_overlaps(p1,p2,r1,p3,p4,r2):
	#get rectangles points for robot r1
	A1,B1,C1,D1 = get_rectangle_points(p1,p2,r1)
	A2, B2, C2, D2 = get_rectangle_points(p3,p4,r2)
	
	if check_two_circles_intersect(p1,r1,p3,r2):
		return True
	if intersect(p1,r1,A2,B2,C2,D2,p3,p4):
		return True
	if check_two_circles_intersect(p1,r1,p4,r2):
		return True
	if intersect(p3,r2,A1,B1,C1,D1,p1,p2):
		return True
	if doIntersect(p1,p2,p3,p4):
		return True
	if intersect(p4,r2,A1,B1,C1,D1,p1,p2):
		return True
	if check_two_circles_intersect(p2,r1,p3,r2):
		return True
	if intersect(p2,r1,A2,B2,C2,D2,p3,p4):
		return True
	if check_two_circles_intersect(p2,r1,p4,r2):
		return True
	return False
	
#if __name__ == '__main__':
#    A,B,C,D = get_rectangle_points(Point(-1,-1),Point(-4,-2),0.5)
#    if isinsiderect(Point(-1,-1),Point(1,5),A,B,C,D,Point(1,4)):
#        print('yes')