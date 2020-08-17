# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,extra)

import math
import itertools
import heapq

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_corner": astar_corner,
        "astar_multi": astar_multi,
        "extra": extra,
    }.get(searchMethod)(maze)


def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here

    # goal 
    if(len(maze.getObjectives()) == 0):
        return []
    goal = maze.getObjectives()[0]
    # starting position
    if(maze.getStart() != None):
        start = maze.getStart()
    else:
        return []
    # position tracker
    k = start
    # queue
    queue = []
    queue.append(k)
    # parent dict
    parent = {}
    # visited list
    visited = {}
    visited[start] = 0

    # BFS
    while(len(queue) != 0):
        # pop first entry in queue
        k = queue[0]  
        queue.pop(0)
        # list of neighbors 
        neighbor = maze.getNeighbors(k[0], k[1])
        # reach goal
        if(k == goal):
            break
        # loop through all neigbors
        for i in neighbor:
            tmp = visited[k] + 1
            if(i not in visited):
                visited[i] = visited[k] + 1
                parent[i] = k
                queue.append(i)

    # return value
    result = []
    # add path to result
    j = goal
    while(j != start):
        result.append(j)
        j = parent[j]  
    result.append(start)    

    return result[::-1]



# helper function for calculating heuristic value
# dest and src are tuples of coordinates
def h(dest, src):
    if(dest == None or src == None):
        return 0
    return abs(dest[0] - src[0]) + abs(dest[1] - src[1])



def second(elem):
    if(elem == None):
        return 0   
    return elem[1]



def astar(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here

    # goal 
    if(len(maze.getObjectives()) == 0):
        return []
    goal = maze.getObjectives()[0]
    # starting position
    if(maze.getStart() != None):
        start = maze.getStart()
    else:
        return []
    # position tracker
    k = start
    # parent dict
    parent = {}
    # list of g scores
    g = {}
    g[start] = 0
    # list of function score and corresponding nodes
    f = {}
    f[start] = h(goal, start)
    # priority queue
    queue = []
    queue.append((k,f[start]))

    # A* search
    while(len(queue) != 0):
        queue = sorted(queue, key=second)
        k = queue[0][0]
        queue.pop(0)
        # reach the goal
        if(k == goal):
            break
        # loop through neighbors
        neighbor = maze.getNeighbors(k[0],k[1])
        for i in neighbor:
            tmp = g[k] + 1
            if(i in f):
                if(tmp < g[i]):
                    parent[i] = k
                    g[i] = tmp
                    f[i] = tmp + h(goal, i)
                    if (i not in queue):
                        queue.append((i, f[i]))
            else:
                parent[i] = k
                g[i] = tmp
                f[i] = tmp + h(goal, i)
                if (i not in queue):
                    queue.append((i, f[i]))

    # return value
    result = []
    # # add path to result
    j = goal
    while(j != start):
        result.append(j)
        j = parent[j]  
    result.append(start)   

    return result[::-1]



# revised version of a*star
def astar_revised(maze, start, goal):
    if(start == None):
        return []
    if(len(goal) == 0):
        return []
    # position tracker
    k = start
    # parent dict
    parent = {}
    # list of g scores
    g = {}
    g[start] = 0
    # list of function score and corresponding nodes
    f = {}
    f[start] = h(goal, start)
    # priority queue
    queue = []
    queue.append((k,f[start]))

    # A* search
    while(len(queue) != 0):
        queue = sorted(queue, key=second)
        k = queue[0][0]
        queue.pop(0)
        # reach the goal
        if(k == goal):
            break
        # loop through neighbors
        neighbor = maze.getNeighbors(k[0],k[1])
        for i in neighbor:
            tmp = g[k] + 1
            if(i in f):
                if(tmp < g[i]):
                    parent[i] = k
                    g[i] = tmp
                    f[i] = tmp + h(goal, i)
                    if (i not in queue):
                        queue.append((i, f[i]))
            else:
                parent[i] = k
                g[i] = tmp
                f[i] = tmp + h(goal, i)
                if (i not in queue):
                    queue.append((i, f[i]))

    # return value
    result = []
    # # add path to result
    j = goal
    while(j != start):
        result.append(j)
        j = parent[j]  
    result.append(start)   

    return (result[::-1], g[goal])



#################################################################################################################################################



# calculate the minimum cost of spanning tree
# @path: dict, key is node and val is tuple
# @nodes: a list of nodes 
# @return the total of MST
def MST_cost(cost, nodes):
    if(len(nodes) == 0 or len(nodes) == 1):
        return 0
    # visited vertices
    visited = []
    visited.append(nodes[0])
    # remaining
    remain = []
    for i in nodes:
        if(i != nodes[0]):
            remain.append(i)
    # total
    total = 0
    # starting spanning
    while(len(remain) != 0):
        minCost = math.inf
        cur = remain[0]
        for i in visited: 
            for j in cost[i]:
                if(j[0] in remain and j[1] < minCost):
                    minCost = j[1]
        total += minCost
        remain.remove(cur)
        visited.append(cur)
    return total



# path cost generator
def path_cost(maze):
    # cost list
    cost = {}
    # all goals possible
    goal_set = maze.getObjectives()
    goal_set.append(maze.getStart())
    # generate all costs 
    for i in goal_set:
        for j in goal_set:
            if(i != j):
                tmp = (i,j)
                cost[tmp] = astar_revised(maze, i, j)
    return cost



def astar_corner(maze):
    """
    Runs A star for part 2 of the assignment in the case where there are four corner objectives.
        
    @param maze: The maze to execute the search on.
        
    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    # TODO: Write your code here

    # starting node
    start = maze.getStart()

    # a dict of path cost
    # keys: a pair of goal and starting node
    # vals: val[0] is the required path, val[1] is the cost of path
    pathCost = path_cost(maze)

    # list for all destination
    vertices = []
    for i in maze.getObjectives():
        vertices.append(i)
    vertices.append(start)

    # generate dict of nodes, val is corresponding destination and cost
    cost = {}
    for i in vertices:
        cost[i] = []
    for j in pathCost:
        pt1 = j[0]
        pt2 = j[1]
        val = pathCost[j][1]
        if((pt2, val) not in cost[pt1]):
            cost[pt1].append((pt2, val))
        if((pt1, val) not in cost[pt2]):
            cost[pt2].append((pt1, val))

    # generate dict of paths, val is corresponding set of parents
    path = {}
    for i in vertices:
        path[i] = []
    for j in pathCost:
        pt1 = j[0]
        pt2 = j[1]
        val = pathCost[j][0]
        if((pt2, val) not in path[pt1]):
            path[pt1].append((pt2, val))
        if((pt1, val) not in path[pt2]):
            path[pt2].append((pt1, val))
    
    # position tracker
    k = start
    # list of g scores
    g = {}
    g[start] = 0
    minE = math.inf
    for i in cost[start]:
        if(i[1] < minE):
            minE = i[1]
            k = i[0]
    for i in cost[start]:
        if(i[0] == k):
            g[k] = i[1]
            break
    # list of function score and corresponding nodes
    f = {}
    f[start] = 0
    f[k] = g[k]

    # priority queue
    queue = []
    queue.append((k,f[k]))
    # list to track each nodes taken
    visited = []
    visited.append(start)
    visited.append(k)
    # unvisied
    unvisited = []
    for i in vertices:
        if(i != k and i != start):
            unvisited.append(i)

    # A* search
    while(len(visited) != len(maze.getObjectives()) + 1):
        minHeu = math.inf
        nextNode = (0,0)
        if(len(unvisited) == 1):
            visited.append(unvisited[0])
            break
        for i in unvisited:
            # second argument for mst_cost
            mstNode = []
            for j in unvisited:
                if(j != i):
                    mstNode.append(j)
            # distance
            for x in cost[k]:
                if(x[0] == i):
                    distance = x[1]
            heu = MST_cost(cost, mstNode) + distance
            if(heu < minHeu):
                minHeu = heu
                nextNode = i
        visited.append(nextNode)
        unvisited.remove(nextNode)
        k = nextNode
    
    # the result
    result = []
    for i in range(len(visited) - 1):
        dest = visited[i+1]
        src = visited[i]
        p = pathCost[(src,dest)][0]
        if(i + 2 == len(visited)):
            for x in range(len(p)):
                result.append(p[x])
        else:
            for x in range(len(p) - 1):
                result.append(p[x])
    
    return result



###############################################################################################################################################

def costHelper(pathCost, vertices):
    cost = {}
    for i in vertices:
        cost[i] = []
    for j in pathCost:
        pt1 = j[0]
        pt2 = j[1]
        val = pathCost[j][1]
        if((pt2, val) not in cost[pt1]):
            cost[pt1].append((pt2, val))
        if((pt1, val) not in cost[pt2]):
            cost[pt2].append((pt1, val))
    return cost

def closestLength(src, nodes):
    goal = ()
    minLength = math.inf
    for i in nodes:
        l = h(i,src)
        if(l < minLength):
            goal = i
            minLength = l
    return (goal, minLength)

def getState(node, obj):
    new_unvisited = list(obj)
    if(node in obj):
        new_unvisited.remove(node)
    return (node, tuple(new_unvisited))

def MST_cost_2(cost, nodes):
    if(len(nodes) == 0 or len(nodes) == 1):
        return 0
    # visited vertices
    visited = [nodes[0]]
    remain = nodes
    remain.pop(0)
    # total
    total = 0
    # starting spanning
    while(len(remain) != 0):
        minCost = math.inf
        cur = remain[0]
        for i in visited: 
            for j in cost[i]:
                if(j[0] in remain and j[1] < minCost):
                    minCost = j[1]
                    cur = j[0]
        total += minCost
        visited.append(cur)
        remain.remove(cur)
    return total

def astar_multi(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    
    # pre stored value
    pathCost = path_cost(maze)
    vertices = maze.getObjectives()
    vertices.append(maze.getStart())
    cost = costHelper(pathCost, vertices)
    # state representation and transition models
    start = maze.getStart()
    unvisited = maze.getObjectives()
    start_state = (start, tuple(unvisited))
    g = {start_state: 0}
    f = {start_state: 0}
    openSet = [(start_state, f[start_state])]
    parent = {}
    final_state = ()
    mst_lib = {}
    # a* search
    while(len(openSet) != 0):
        # print(len(openSet))
        openSet = sorted(openSet, key=second)
        current_state = openSet.pop(0)[0]
        cur_node = current_state[0]
        cur_unvisited = current_state[1]
        if(len(cur_unvisited) == 0):
                final_state = current_state
                break
        # traverse neighbor states
        neighbor = maze.getNeighbors(cur_node[0], cur_node[1])
        for n in neighbor:
            neighbor_state = getState(n, cur_unvisited)
            neighbor_unvisited = neighbor_state[1]
            tmp_g = g[current_state] + 1
            if(neighbor_state not in g or tmp_g < g[neighbor_state]):
                parent[neighbor_state] = current_state
                g[neighbor_state] = tmp_g
                # heuristic
                if(neighbor_unvisited not in mst_lib.keys()):
                    mst_lib[neighbor_unvisited] = MST_cost_2(cost, list(neighbor_unvisited))
                f[neighbor_state] = tmp_g + mst_lib[neighbor_unvisited]
                openSet.append((neighbor_state, f[neighbor_state]))          
    # get the result
    result = []
    temp_state = final_state
    while(parent[temp_state] != start_state):
        result.append(temp_state[0])
        temp_state = parent[temp_state]
    result.append(temp_state[0])
    result.append(start_state[0])
    return result[::-1]



def extra(maze):
    """
    Runs extra credit suggestion.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []