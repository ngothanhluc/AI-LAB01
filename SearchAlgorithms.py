from Space import *
from Constants import *
import math
def DFS(g:Graph, sc:pygame.Surface):
    print('Implement DFS algorithm')

    open_set = [g.start.value]
    closed_set = []
    father = [-1]*g.get_len()
    while open_set:
        current = open_set.pop()
        g.grid_cells[current].set_color(yellow)
        g.draw(sc)
        if g.is_goal(g.grid_cells[current]):
            return get_path(g,father, g.start, g.goal, sc)
        closed_set.append(g.grid_cells[current].value)
        g.grid_cells[current].set_color(blue)
        g.draw(sc)
        for neighbor in g.get_neighbors(g.grid_cells[current]):
            if neighbor.value in closed_set:
                continue
            if neighbor.value not in open_set:
                open_set.append(neighbor.value)
                g.grid_cells[neighbor.value].set_color(red)
                father[neighbor.value] = g.grid_cells[current]
    return None

def BFS(g:Graph, sc:pygame.Surface):
    print('Implement BFS algorithm')
    open_set = [g.start.value]
    closed_set = []
    father = [-1]*g.get_len()
    while open_set:
        current = open_set.pop(0)
        g.grid_cells[current].set_color(yellow)
        g.draw(sc)
        if g.is_goal(g.grid_cells[current]):
            return get_path(g,father, g.start, g.goal, sc)
        closed_set.append(g.grid_cells[current].value)
        g.grid_cells[current].set_color(blue)
        g.draw(sc)
        for neighbor in g.get_neighbors(g.grid_cells[current]):
            if neighbor.value in closed_set:
                continue
            if neighbor.value not in open_set:
                open_set.append(neighbor.value)
                g.grid_cells[neighbor.value].set_color(red)
                father[neighbor.value] = g.grid_cells[current]
    return None

def UCS(g:Graph, sc:pygame.Surface):
    print('Implement UCS algorithm')

    open_set = {}
    open_set[g.start.value] = 0
    closed_set:list[int] = []
    father = [-1]*g.get_len()
    cost = [100_000]*g.get_len()
    cost[g.start.value] = 0

    
    while open_set:
            current = min(open_set, key = lambda k: open_set[k])
            del open_set[current]
            g.grid_cells[current].set_color(yellow)
            g.draw(sc)
            if g.is_goal(g.grid_cells[current]):
                return get_path(g, father, g.start, g.goal, sc)
            closed_set.append(current)
            g.grid_cells[current].set_color(blue)
            g.draw(sc)
            for neighbor in g.get_neighbors(g.grid_cells[current]):
                if neighbor.value in closed_set:
                    continue
                tentative_cost = cost[current] + get_edge_weights(g.grid_cells[current], neighbor)

                if neighbor.value not in open_set or tentative_cost < cost[neighbor.value]:
                    cost[neighbor.value] = tentative_cost
                    open_set[neighbor.value] = tentative_cost
                    g.grid_cells[neighbor.value].set_color(red)
                    father[neighbor.value] = g.grid_cells[current]
    return None

def AStar(g:Graph, sc:pygame.Surface):
    print('Implement A* algorithm')

    open_set = {}
    open_set[g.start.value] = 0
    closed_set:list[int] = []
    father = [-1]*g.get_len()
    cost = [100_000]*g.get_len()
    cost[g.start.value] = 0

    while open_set:
            current = min(open_set, key = lambda k: open_set[k])
            del open_set[current]
            g.grid_cells[current].set_color(yellow)
            g.draw(sc)
            if g.is_goal(g.grid_cells[current]):
                return get_path(g, father, g.start, g.goal, sc)
            closed_set.append(current)
            g.grid_cells[current].set_color(blue)
            g.draw(sc)
            for neighbor in g.get_neighbors(g.grid_cells[current]):
                if neighbor.value in closed_set:
                    continue
                tentative_g_score = cost[current] + get_edge_weights(g.grid_cells[current], neighbor)

                if neighbor.value not in open_set:
                    open_set[neighbor.value] = tentative_g_score + euclidean_distance(neighbor, g.goal)
                elif tentative_g_score >= cost[neighbor.value]:
                    continue
                cost[neighbor.value] = tentative_g_score
                g.grid_cells[neighbor.value].set_color(red)
                father[neighbor.value] = g.grid_cells[current]
    return None
def get_path(g:Graph, father, start, goal,sc:pygame.Surface):
    path = [goal.value]
    while path[-1] != start.value:
        path.append(father[path[-1]].value)
    path.reverse()
    for i in range(len(path)-1):
        g.grid_cells[path[i]].set_color(grey)
        node1 = path[i]
        node2 = path[i+1]
        pygame.draw.line(sc, green, (g.grid_cells[node1].x, g.grid_cells[node1].y),(g.grid_cells[node2].x, g.grid_cells[node2].y), 2)
        g.draw(sc)
    g.grid_cells[path[0]].set_color(orange)
    g.grid_cells[path[-1]].set_color(purple)
    g.draw(sc)
    return path
def get_edge_weights(current:Node,neighbor:Node):
    return math.sqrt((current.x - neighbor.x)**2 + (current.y - neighbor.y)**2)
def euclidean_distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)