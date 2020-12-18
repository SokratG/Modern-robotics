import numpy as np
import modern_robotics as mr
import math as math


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


runA('nodes.csv', 'edges.csv')