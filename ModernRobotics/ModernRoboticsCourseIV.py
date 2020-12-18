import numpy as np
import modern_robotics as mr
import math as math
from random import uniform



# ------------------------------------ A start ------------------------------------
def GraphPrepare(edges):
    """
        :param edges: list of pair nodes and them weight
        :return dict{node: list((nodes, weight))}: traversal graph with nodes and weight of edges
        build graph - dictionary with node and list of tuple(neighboring nodes, edge weight)
    """
    graph = {}
    for e in edges:
        node1, node2, weight = e # unpack data
        if graph.get(node1) == None:
            graph[node1] = []
        if graph.get(node2) == None:
            graph[node2] = []
        graph[node1].append((node2, weight))
        graph[node2].append((node1, weight))

    return graph



def buildMinPath(parent_nodes, start, end):
    """
        :param parents_nodes: list of minimal cost path contains nodes and None values
        :param start: start position for search
        :param end: end position for search
        :return tupel(bool, list): return search result, if result not find, 1-st parameter is False and list is start element, else 1-st parameter True and list contains minmal cost path
    """
    min_path = []
    next = end
    
    while next is not None:
        min_path.append(next)
        next = parent_nodes[next-1]

    if len(min_path) == 0:
        return (False, start)
   
    min_path.reverse()
    return (True, min_path)


     

def Astart(nodes, edges, start, end):
    """
        :param nodes: list of tuple contain next structure - (ID node, x-coord, y-coord, heuristic-cost-to-go)
        :param edges: dictionary with information about node - key and list of neighbors nodes with weight edge between them - value
        :param start: start node for find minimum cost path
        :param end: end node for find minimum cost path
        :return tuple: tuple with information of path is found and list of nodes(contains minimal cost path) 
    """
    N = len(nodes)
    parent_nodes = [None] * N
    est_total_cost = [np.inf] * N
    est_total_cost[0] = nodes[0][3]
    heuristic_opt = []
    for n in nodes:
        heuristic_opt.append(n[3])
    past_cost = [np.inf] * N
    past_cost[0] = 0 # start? 

    OPEN = []
    OPEN.append((start, float(nodes[0][3])))
    CLOSE = set()

    while(len(OPEN) > 0):
        current = OPEN.pop()

        if current[0] in CLOSE:
                continue
        CLOSE.add(current[0])
        if current[0] == end:
            break 

        subedges = edges[current[0]]
        for se in subedges:
            node = se[0]
            weight = se[1]
            if node in CLOSE:
                continue
            tentative_past_cost = past_cost[current[0] - 1] + weight
       
            if (tentative_past_cost < past_cost[node-1]):
                past_cost[node-1] = tentative_past_cost
                parent_nodes[node-1] = current[0]
                est_total_cost[node-1] = tentative_past_cost + heuristic_opt[node-1]
                OPEN.append((node, est_total_cost[node-1]))
                OPEN.sort(key=lambda val: val[1], reverse=True)

    return buildMinPath(parent_nodes, start, end)



def writeResult(result, start):
    """
        :return None: 
        save result data in csv file
    """
    filename = 'path.csv'
    res = result[0]

    with open(filename, 'w') as fs:
        if res == False:
            fs.write(str(start))
        else:
            data = ', '.join(map(str, result[1]))
            fs.write(data)
    return



def runA(pathnodes, pathedges):
    """
        :param pathnodes: file path to node.csv contains information about nodes of graph
        :param pathedges: file path to edge.csv contains information about edge of graph
        :return None:
        build minimal cost path by algorithm A* and save this path in file path.csv
    """
    nodes = []
    with open(pathnodes) as filenodes:
        line = filenodes.readline()
        while line:
            node, x, y, heuristic = (line.split(','))
            nodes.append([int(node), float(x), float(y), float(heuristic)])
            line = filenodes.readline()
    #print(nodes)

    edges_ = []
    with open(pathedges) as fileedges:
        line = fileedges.readline()
        while line:
            node1, node2, weight = (line.split(','))
            edges_.append((int(node1), int(node2), float(weight)))
            line = fileedges.readline()
    edges = GraphPrepare(edges_)
    #print(edges)
    start = 1
    end = len(nodes)
    result = Astart(nodes, edges, start, end)

    writeResult(result, start)

    return

def Week1Project():
    runA('nodes.csv', 'edges.csv')
    return




# ------------------------------------ RRT ------------------------------------

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
            n1, n2 = e[0][0], e[0][1]
            weight = e[1]
            data = '{:d}, {:d}, {:.5f}\n'.format(n1.ID, n2.ID, weight)
            fs.write(data)
    
    with open('path.csv', 'w') as fs:
        data = ', '.join(map(str, rrtree))
        fs.write(data)
        
    return

def swapMax(node1, node2):
    if (node1.ID > node2.ID):
        return [node2, node1]
    else:
        return [node1, node2]

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
    complete = False
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
            edges.append((swapMax(nodes[near_id], nnode_), EuclideanDist(nodes[near_id].coord, nnode_.coord)))

            # check converge algorithm
            if EuclideanDist(nnode_.coord, end.coord) < converge_dist:
                print("complete build tree")
                complete = True
                break

        iter += 1

    #print(len(nodes))
    if complete == False:
        print("Fail build a tree...try next time")
        return

    minpath = [nnode_.ID]
    while not nnode_.parentNode == None:
        minpath.append(nnode_.parentNode)
        id_ = next((i for i, x in enumerate(nodes) if cmpr(x, nnode_.parentNode)), None)
        nnode_ = nodes[id_]

    minpath.reverse()
    #print(minpath)
    writeResult(minpath, nodes, edges)

    return



def Week2Project():
    runRRT('obstacles.csv')
    return


