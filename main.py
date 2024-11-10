import time
import sys

graph = {
    'Letterkenny': {'Sligo': 130},
    'Sligo': {'Letterkenny': 130, 'Castlebar': 67, 'Dundalk': 214},
    'Castlebar': {'Sligo': 67, 'Galway': 77},
    'Galway': {'Castlebar': 77, 'Athlone': 85, 'Limerick': 112},
    'Athlone': {'Galway': 85, 'Dublin': 124, 'Tipperary': 126},
    'Dublin': {'Athlone': 124, 'Dundalk': 81, 'Carlow': 90, 'Wexford': 141},
    'Dundalk': {'Dublin': 81, 'Belfast': 83, 'Sligo': 214},
    'Belfast': {'Dundalk': 83},
    'Carlow': {'Dublin': 90, 'Tipperary': 80},
    'Tipperary': {'Carlow': 80, 'Limerick': 39, 'Waterford': 89, 'Athlone': 126},
    'Limerick': {'Tipperary': 39, 'Galway': 112},
    'Waterford': {'Tipperary': 89, 'Wexford': 59, 'Cork': 121},
    'Wexford': {'Waterford': 59, 'Dublin': 141},
    'Cork': {'Waterford': 121, 'Killarney': 88},
    'Killarney': {'Cork': 88, 'Limerick': 110}
}

def bfs(graph, start, goal):
    queue = [(start, [start])]
    visited = set()
    
    while queue:
        (vertex, path) = queue.pop(0)
        if vertex in visited:
            continue
        
        visited.add(vertex)
        for neighbor in graph[vertex]:
            if neighbor == goal:
                return path + [neighbor]
            else:
                queue.append((neighbor, path + [neighbor]))
    
    return None

def dijkstra(graph, start, goal):
    shortest_distances = {node: float('inf') for node in graph}
    shortest_distances[start] = 0
    previous_nodes = {node: None for node in graph}
    unvisited_nodes = set(graph.keys())

    while unvisited_nodes:
        current_node = min(unvisited_nodes, key=lambda node: shortest_distances[node])
        if current_node == goal or shortest_distances[current_node] == float('inf'):
            break
        for neighbor, distance in graph[current_node].items():
            new_distance = shortest_distances[current_node] + distance
            if new_distance < shortest_distances[neighbor]:
                shortest_distances[neighbor] = new_distance
                previous_nodes[neighbor] = current_node
        unvisited_nodes.remove(current_node)

    path, current_node = [], goal
    while previous_nodes[current_node] is not None:
        path.insert(0, current_node)
        current_node = previous_nodes[current_node]
    if path:
        path.insert(0, current_node)

    return shortest_distances[goal], path if path else None

start = 'Tipperary'
goal = 'Sligo'

start_time = time.time()
bfs_path = bfs(graph, start, goal)
bfs_time = (time.time() - start_time) * 1000
print("BFS Path from Tipperary to Sligo:", bfs_path)
print(f"BFS Time taken: {bfs_time:.2f} ms")

start_time = time.time()
dijkstra_cost, dijkstra_path = dijkstra(graph, start, goal)
dijkstra_time = (time.time() - start_time) * 1000 
print("Dijkstra's Path and Costa ", dijkstra_path, dijkstra_cost)
print(f" Time taken: {dijkstra_time:.2f} ms")
