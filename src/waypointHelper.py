def chop(x, xmin, xmax):
    if x < xmin:
        return xmin
    elif x > xmax:
        return xmax
    else:
        return x

def distance(x1,y1,x2,y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def angle_diff(a0, a1):
    return (((a0 - a1) + 3 * math.pi) % (2 * math.pi)) - math.pi

def getTwist(pose,newx,newy):
    linearTune = 10.0
    angularTune = 1.0

    d = distance(pose.x, pose.y,newx,newy)

    heading_to_p = math.atan2(newy - pose.y, newx - pose.x)
    print("Heading to Point: " + str(heading_to_p))
    heading_error = angle_diff(pose.theta, heading_to_p)
    print("Error: " + str(heading_error))
    w = chop(-angularTune * heading_error, -0.2, 0.2)
    v = chop(1.0 / (linearTune * abs(heading_error)), 0.0, .75)

    dpose = Twist()
    dpose.linear.x = v
    dpose.angular.z = w

    return dpose

