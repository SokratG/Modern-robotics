import numpy as np
import modern_robotics as mr
import math as math
from random import uniform


class Node(object):
    """
        :class Node: representes a node of graph
        :prop ID: integer define a ID of node in graph
        :prop coord: tuple defines x,y coordinate in grid
        :prop parentNode: object Node defines connected node in path graph
    """
    def __init__(self, ID, x, y):
        self.ID = ID
        self.coord = (x, y)
        self.parentNode = None

class Edge(object):
    """
        :class Edge: representes a edge of graph
        :prop nodes: a set of 2 connected nodes
        :prop weight: float defines a weight of edge between nodes 
    """
    def __init__(self, nodeobj1, nodeobj2, weight):
        self.nodes = set([nodeobj1, nodeobj2])
        self.weight = weight


def EuclideanDist(coord_1, coord_2):
    """
        :param coord_1, coord_2: two tuple with x,y - coordinate
        :return float: a euclidean distance between two coordinates
    """
    dx = coord_1[0] - coord_2[0]
    dy = coord_1[1] - coord_2[1]
    return np.sqrt((dx**2) + (dy**2))


def FindNearest(nodes, sample):
    """
        :param nodes: list of object Node
        :param sample: object Node
        :return integer: ID of node the nearest to sample 
    """
    short_dist = np.inf
    nearist_id = 0
    for n in nodes:
        temp = EuclideanDist(n.coord, sample)
        if temp < short_dist:
            short_dist = temp
            nearist_id = n.ID

    return nearist_id



def GetNewCoord(near_coord, sample, ddist=0.1):
    """
        :param near_coord: 
        :param sample: 
        :param ddist: 
        :return tuple:
    """
    temp_dist = EuclideanDist(near_coord, sample)
    if temp_dist < ddist:
        return sample
    
    dx = sample[0] - near_coord[0]
    dy = sample[1] - near_coord[1]
    # new coord (x, y)
    return (ddist * dx/temp_dist + near_coord[0], ddist * dy/temp_dist + near_coord[1])


def CollisionCircle(p1, p2, obs):
    """
        :param p1, p2:  x,y - coordinates in grid 
        :param obs: list of obstacles
        :return bool: return true if given point collision with a one of obstacle in list, else return false
    """
    a = p1[1] - p2[1]
    b = p2[0] - p1[0]
    c = (p1[0]-p2[0])*p1[1] + (p2[1]-p1[1])*p1[0]

    denom = np.sqrt(a**2 + b**2)

    if denom == 0: # avoid divivde by 0
        denom += 1e-7

    for o in obs:
        x_, y_= o[0], o[1]
        r = o[2] / 2
        dist_ = abs(a*x_ + b*y_ + c) / denom
        if (dist_ < r):
            return True

    return False



def writeResult(rrtree, nodes, edges):
    """
        :param rrtree: complete RRT contains a minimal cost path
        :param nodes: list of object Node 
        :param edges: list of object Edge
        :return None:
        write given collection in csv files
    """

    if (len(nodes) == 0 or len(rrtree) == 0 or len(edges) == 0):
        return

    with open('nodes.csv', 'w') as fs:
        for n in nodes:
            data = '{:d}, {:.4f}, {:.4f}\n'.format(n.ID, n.coord[0], n.coord[1])
            fs.write(data)
    
    with open('edges.csv', 'w') as fs:
        for e in edges:
            n2, n1 = e.nodes.pop(), e.nodes.pop()
            data = '{:d}, {:d}, {:.5f}\n'.format(n1.ID, n2.ID, e.weight)
            fs.write(data)
    
    with open('path.csv', 'w') as fs:
        data = ', '.join(map(str, rrtree))
        fs.write(data)
        
    return



def runRRT(fileobstacles):
    """
        :param fileobstacles: filepath with csv file contain a coordinate and diameter of obstacles
        :return None:
        Build a rapidly-exploring random trees with feasible nodes and edges on grid. Find minimum cost path in the builded graph and write result
        in csv file - nodes.csv, edges.csv, path.csv
    """

    obstacles = []
    with open(fileobstacles) as fileobs:
        line = fileobs.readline()
        while line:
            x, y, d = (line.split(','))
            obstacles.append((float(x), float(y), float(d)))
            line = fileobs.readline()
    #print(obstacles)

    near_distance = 0.1
    converge_dist = 0.1
    max_iters = 999
    max_nodes = 200

    nodes = []
    edges = []
    start, next_node, iter = Node(1, -0.5, -0.5), 1, 0
    end = Node(-1, 0.5, 0.5)
    nodes.append(start)

    cmpr = lambda n, near: n.ID == near
    # iterate until distance will be converge or reach to limit iterations
    while len(nodes) < max_nodes and iter < max_iters:

        sample = (uniform(-0.5, 0.5), uniform(-0.5, 0.5))
        n_nearst = FindNearest(nodes, sample)
        
        near_id = next((i for i, x in enumerate(nodes) if cmpr(x, n_nearst)), None)

        n_new = GetNewCoord(nodes[near_id].coord, sample, near_distance)
    
        isCollided = CollisionCircle(nodes[near_id].coord, n_new, obstacles)
        
        if not isCollided:
            next_node += 1
            nnode_ = Node(next_node, sample[0], sample[1])
            nodes.append(nnode_)
            nnode_.parentNode = n_nearst
            edges.append(Edge(nodes[near_id], nnode_, EuclideanDist(nodes[near_id].coord, nnode_.coord)))
            # check converge algorithm
            if EuclideanDist(nnode_.coord, end.coord) < converge_dist:
                print("complete build tree")
                break

        iter += 1

    #print(len(nodes))

    minpath = [nnode_.ID]
    while not nnode_.parentNode == None:
        minpath.append(nnode_.parentNode)
        id_ = next((i for i, x in enumerate(nodes) if cmpr(x, nnode_.parentNode)), None)
        nnode_ = nodes[id_]

    minpath.reverse()
    #print(minpath)
    writeResult(minpath, nodes, edges)

    return




runRRT('obstacles.csv')
 
