
# Import required packages
from RRT import RRT
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm
import random,sys,math
from scipy import interpolate

MIN_NUM_VERT = 20  # Minimum number of vertex in the graph
MAX_NUM_VERT = 1500  # Maximum number of vertex in the graph
STEP_DISTANCE = 20  # Maximum distance between two vertex
SEED = None  # For random numbers


def rapidlyExploringRandomTree(ax, img, start, goal, seed=None):
    hundreds = 100
    random.seed(seed)
    points = []
    graph = []
    points.append(start)
    graph.append((start, []))

    occupied = True
    phaseTwo = False

    # Phase two values (points 5 step distances around the goal point)
    minX = max(goal[0] - 5 * STEP_DISTANCE, 0)
    maxX = min(goal[0] + 5 * STEP_DISTANCE, len(img[0]) - 1)
    minY = max(goal[1] - 5 * STEP_DISTANCE, 0)
    maxY = min(goal[1] + 5 * STEP_DISTANCE, len(img) - 1)

    i = 0
    while (goal not in points) and (len(points) < MAX_NUM_VERT):

        if (len(points) % hundreds) == 0:
            hundreds = hundreds + 100

        while occupied:
            if phaseTwo and (random.random() > 0.8):
                point = [random.randint(minX, maxX), random.randint(minY, maxY)]
            else:
                    point = [random.randint(0, len(img[0]) - 1), random.randint(0, len(img) - 1)]

            if img[point[1]][point[0]] * 255 == 255:
                occupied = False

        occupied = True

        nearest = findNearestPoint(points, point)
        newPoints = connectPoints(point, nearest, img)
        addToGraph(ax, graph, newPoints, point)
        newPoints.pop(0)  # The first element is already in the points list
        points.extend(newPoints)
        ax.plot(start[0], start[1], 'sr')
        ax.plot(goal[0], goal[1], 'sr')
        plt.draw()
        i = i + 1

        if len(points) >= MIN_NUM_VERT:
            phaseTwo = True

        if phaseTwo:
            nearest = findNearestPoint(points, goal)
            newPoints = connectPoints(goal, nearest, img)
            addToGraph(ax, graph, newPoints, goal)
            newPoints.pop(0)
            points.extend(newPoints)
            plt.draw()

    if goal in points:
        print('Goal found, total vertex in graph:', len(points), 'total random points generated:', i)
        path = searchPath(graph, start, [start])

        for i in range(len(path) - 1):
            ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], color='g', linestyle='-', linewidth=2)
            plt.draw()

        print('The final path is made from:', len(path), 'connected points')
    else:
        path = None
        print('Reached maximum number of vertex and goal was not found')
        print('Total vertex in graph:', len(points), 'total random points generated:', i)

    plt.show()
    return path

def searchPath(graph, point, path):
    for i in graph:
        if point == i[0]:
            p = i

    if p[0] == graph[-1][0]:
        return path

    for link in p[1]:
        path.append(link)
        finalPath = searchPath(graph, link, path)

        if finalPath != None:
            return finalPath
        else:
            path.pop()

def addToGraph(ax, graph, newPoints, point):
    if len(newPoints) > 1:  # If there is anything to add to the graph
        for p in range(len(newPoints) - 1):
            nearest = [nearest for nearest in graph if (nearest[0] == [newPoints[p][0], newPoints[p][1]])]
            nearest[0][1].append(newPoints[p + 1])
            graph.append((newPoints[p + 1], []))

            if not p == 0:
                ax.plot(newPoints[p][0], newPoints[p][1], '+k')  # First point is already painted
            ax.plot([newPoints[p][0], newPoints[p + 1][0]], [newPoints[p][1], newPoints[p + 1][1]], color='k',
                    linestyle='-', linewidth=1)

        if point in newPoints:
            ax.plot(point[0], point[1], '.g')  # Last point is green
        else:
            ax.plot(newPoints[p + 1][0], newPoints[p + 1][1], '+k')  # Last point is not green

def connectPoints(a, b, img):
    newPoints = []
    newPoints.append([b[0], b[1]])
    step = [(a[0] - b[0]) / float(STEP_DISTANCE), (a[1] - b[1]) / float(STEP_DISTANCE)]

    # Set small steps to check for walls
    pointsNeeded = int(math.floor(max(math.fabs(step[0]), math.fabs(step[1]))))

    if math.fabs(step[0]) > math.fabs(step[1]):
        if step[0] >= 0:
            step = [1, step[1] / math.fabs(step[0])]
        else:
            step = [-1, step[1] / math.fabs(step[0])]

    else:
        if step[1] >= 0:
            step = [step[0] / math.fabs(step[1]), 1]
        else:
            step = [step[0] / math.fabs(step[1]), -1]

    blocked = False
    for i in range(pointsNeeded + 1):  # Creates points between graph and solitary point
        for j in range(STEP_DISTANCE):  # Check if there are walls between points
            coordX = round(newPoints[i][0] + step[0] * j)
            coordY = round(newPoints[i][1] + step[1] * j)

            if coordX == a[0] and coordY == a[1]:
                break
            if coordY >= len(img) or coordX >= len(img[0]):
                break
            if img[int(coordY)][int(coordX)] * 255 < 255:
                blocked = True
            if blocked:
                break

        if blocked:
            break
        if not (coordX == a[0] and coordY == a[1]):
            newPoints.append([newPoints[i][0] + (step[0] * STEP_DISTANCE), newPoints[i][1] + (step[1] * STEP_DISTANCE)])

    if not blocked:
        newPoints.append([a[0], a[1]])
    return newPoints

def findNearestPoint(points, point):
    best = (sys.maxsize, sys.maxsize, sys.maxsize)
    for p in points:
        if p == point:
            continue
        dist = math.sqrt((p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2)
        if dist < best[2]:
            best = (p[0], p[1], dist)
    return (best[0], best[1])

def planner(Pc, Pg, O, B=[-0.05, 0.65], delta=0.02, **args):
    print("PLANNER!!!")
    """

    Args:
        Pc: start point (x_s, y_s) --> list: len=2 OR np.array(): shape=(2,)
        Pg: end point (x_g, y_g) --> list: len=2 OR np.array(): shape=(2,)
        O: [(x_obs_1, y_obs_2, radius_obs_1), ..., (x_obs_N, y_obs_N, radius_obs_N)
        B: this is the area where you plan the path in. both x and y should be in between these boundaries.
        delta: Path resolution.
        **args: add additional arguments as you please such as whether to plot a path or not.

    Returns:
        path: [[x_1, y_1], [x_2, y_2], ..., [x_M, y_M]] --> List of lists of size 2 (x, y).
                Important: Make sure that your output 'path' is in the right order (from start to goal)
    """

    multiplier = 500  # Precision of the path
    divider = 6  # The higher the number, the more point we trace on the graph

    points = np.zeros((abs(int(multiplier * B[0])) * int(multiplier * delta / divider),
                       abs(int(multiplier * B[1])) * int(multiplier * delta / divider)))

    # For every point in the area spaced by delta/2 check collision and distance to the closest obstacle
    for i in range(0, int(multiplier * B[0]), int(multiplier * delta / divider)):
        for j in range(0, int(multiplier * B[1]), int(multiplier * delta / divider)):
            for k in range(len(O)):
                x = np.array([float(i / multiplier), float(j / multiplier)])  # Coordinate we are checking

                if np.linalg.norm(x - np.array([O[k][0], O[k][1]])) - O[k][2] <= 0:  # If collision, pain a black pixel
                    points[i][j] = 0
                    break
                if k == len(O) - 1:  # If there are no collisions with any of the obstacles
                    points[i][j] = 1  # Add the x to the coordinates vector

    # Plot
    fig = plt.gcf()
    fig.clf()
    ax = fig.add_subplot(1, 1, 1)
    ax.imshow(points, cmap=cm.Greys_r)
    start_pixels = [Pc[0] * abs(int(multiplier * B[0])) * int(multiplier * delta / divider),
                    Pc[1] * abs(int(multiplier * B[1])) * int(multiplier * delta / divider)]
    goal_pixels = [Pg[0] * abs(int(multiplier * B[0])) * int(multiplier * delta / divider),
                   Pg[1] * abs(int(multiplier * B[1])) * int(multiplier * delta / divider)]

    path = RRT.planning(ax, points, start_pixels, goal_pixels, seed=SEED)

    for i in range(len(path)):
        path[i][0] = path[i][0] / (abs(multiplier * B[0]) * int(multiplier * delta / divider))
        path[i][1] = path[i][1] / (abs(multiplier * B[1]) * int(multiplier * delta / divider))
    print('Final path:', path)


    return path

def steering_angle(A_cam_robot, A_cam_base, p_i_base):
    print("STEERING!!!")
    """

    Args:
        A_cam_robot: Homogeneous matrix from car to camera frame
        A_cam_base: Homogeneous matrix from origin to camera frame
        p_i_base: 2d vector of next point in the path with respect to base (origin) frame: (x, y)

    Returns:
        p_i_car: 2d vector of next point in path with respect to car frame: (x, y)
        alpha: Steering angle to next point [degrees].
    """
    p_i_base=np.append(p_i_base,1) #turn Pi to a 4x1 shape
    A_base_cam=np.linalg.inv(A_cam_base)
    A_base_robot=np.matmul(A_base_cam,A_cam_robot)
    A_robot_base=np.linalg.inv(A_base_robot)
    p_i_robot=np.matmul(A_robot_base,p_i_base) #robot coordinates in base frame
    p_i_car=p_i_robot[:2] #keep only x,y coordinates
    alpha=np.arctan2(p_i_car[0],p_i_car[1]) #angle is relative to car y axis
    alpha=np.rad2deg(alpha) #convert to degrees
    print(p_i_car,alpha)
    return p_i_car,alpha


    pass

if __name__ == "__main__":
    # Testing
    Obstacles = np.array([[0.5, 0.3,0.1], [0.2, 0.2, 0.2], [0.6,0.6,0.1], [0.35, 0.7, 0.1] ])
    planner([0.5, 0], [0.1, 0.7], Obstacles, [1, 1])

    # A_cam_robot = np.array([[-0.1614, -0.6982, 0.6975, 0.412], [0.9769, -0.0127, 0.2133, 0.218], [-0.1400, 0.7158, 0.6841, 0.797], [0., 0., 0., 1.]])
    # A_cam_base = np.array([[-0.7537, 0.6208, 0.2157, 0.112], [-0.1910, -0.5292, 0.8246, 0.801], [0.6261, 0.5783, 0.5230, 0.797],[0., 0., 0., 1.]])
    # p_i_base = np.array([0.5, 1.1, 0.])
    #print(steering_angle(A_cam_robot,A_cam_base,p_i_base))

