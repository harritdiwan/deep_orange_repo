import networkx as nx

def findpath(start, goal, graph):
    return nx.astar_path(graph,start,goal,heuristic=euclidean_distance)

def euclidean_distance(u,v):
    return u.dist(v)
