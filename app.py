from flask import Flask, request, jsonify
import osmnx as ox
from flask_cors import CORS
import heapq
import random
import networkx as nx

app = Flask(__name__)
CORS(app)

class ShortestPathSolver:
    def __init__(self, graph):
        self.graph = graph
        self.total_distance = 0  # Initialize total distance

    def add_edge_lengths(self):
        # Check if the graph is not projected, and project it
        if 'crs' not in self.graph.graph:
            self.graph = ox.project_graph(self.graph)

        for u, v, data in self.graph.edges(data=True):
            try:
                lat_u = self.get_node_attribute(u, 'y')
                lon_u = self.get_node_attribute(u, 'x')
                lat_v = self.get_node_attribute(v, 'y')
                lon_v = self.get_node_attribute(v, 'x')

                if lat_u is not None and lon_u is not None and lat_v is not None and lon_v is not None:
                    data['length'] = ox.distance.great_circle_vec(lat_u, lon_u, lat_v, lon_v)
                else:
                    data['length'] = 1  # Default length if coordinates are missing
            except KeyError:
                # Handle the case where node attributes are stored in a nested dictionary
                lat_u = self.get_node_attribute(u, 0, 'y')
                lon_u = self.get_node_attribute(u, 0, 'x')
                lat_v = self.get_node_attribute(v, 0, 'y')
                lon_v = self.get_node_attribute(v, 0, 'x')

                if lat_u is not None and lon_u is not None and lat_v is not None and lon_v is not None:
                    data['length'] = ox.distance.great_circle_vec(lat_u, lon_u, lat_v, lon_v)
                else:
                    data['length'] = 1  # Default length if coordinates are missing

    def get_node_attribute(self, node, *keys):
        # Helper function to get node attributes, handling nested dictionaries
        try:
            value = self.graph.nodes[node]
            for key in keys:
                value = value[key]
            return value
        except KeyError:
            return None

    def dijkstra(self, start_coords, end_coords):
        # Add edge lengths to the graph
        self.add_edge_lengths()

        start_node = ox.distance.nearest_nodes(self.graph, start_coords[1], start_coords[0])
        end_node = ox.distance.nearest_nodes(self.graph, end_coords[1], end_coords[0])

        distances = {node: float('inf') for node in self.graph.nodes}
        distances[start_node] = 0
        previous_nodes = {}

        priority_queue = [(0, start_node)]

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_node == end_node:
                path = []
                while current_node:
                    path.insert(0, current_node)
                    current_node = previous_nodes.get(current_node)
                return path, self.total_distance  # Return both path and total distance

            if current_distance > distances[current_node]:
                continue

            for neighbor in self.graph.neighbors(current_node):
                # Consider the edge length as the distance
                edge_data = self.graph[current_node][neighbor]
                distance = distances[current_node] + edge_data.get('length', 1)

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))
                    # Update total distance
                    self.total_distance = distance

        return [], 0  # Return empty path and 0 distance if no path is found



class AStarPathSolver:
    def __init__(self, graph):
        self.graph = graph
        self.add_edge_costs()

    def add_edge_costs(self):
        # Check if the graph is not projected, and project it
        if 'crs' not in self.graph.graph:
            self.graph = ox.project_graph(self.graph)

        for u, v, data in self.graph.edges(data=True):
            data['cost'] = data.get('length', 1)
            try:
                lat_u, lon_u = self.get_node_coordinates(u)
                lat_v, lon_v = self.get_node_coordinates(v)

                if lat_u is not None and lon_u is not None and lat_v is not None and lon_v is not None:
                    # Use great-circle distance as the heuristic (H value)
                    h_value = ox.distance.great_circle_vec(lat_u, lon_u, lat_v, lon_v)
                    data['cost'] = h_value
                else:
                    data['cost'] = 1  # Default cost if coordinates are missing
            except KeyError:
                # Handle the case where node attributes are stored in a nested dictionary
                lat_u, lon_u = self.get_node_coordinates(u, 0)
                lat_v, lon_v = self.get_node_coordinates(v, 0)

                if lat_u is not None and lon_u is not None and lat_v is not None and lon_v is not None:
                    h_value = ox.distance.great_circle_vec(lat_u, lon_u, lat_v, lon_v)
                    data['cost'] = h_value
                else:
                    data['cost'] = 1  # Default cost if coordinates are missing

    def get_node_coordinates(self, node, key=0):
        # Helper function to get node coordinates, handling nested dictionaries
        try:
            value = self.graph.nodes[node]
            for _ in range(key + 1):
                value = value[key]
            return value.get('y', 0), value.get('x', 0)
        except KeyError:
            return 0, 0

    def a_star(self, start_coords, end_coords):
        # Add edge costs to the graph
        self.add_edge_costs()

        # Find the nearest nodes based on coordinates
        start_node = ox.distance.nearest_nodes(self.graph, start_coords[1], start_coords[0])
        end_node = ox.distance.nearest_nodes(self.graph, end_coords[1], end_coords[0])

        print("Start node:", start_node)
        print("End node:", end_node)
        print("Edges around start node:", list(self.graph.edges(start_node, data=True)))
        print("Edges around end node:", list(self.graph.edges(end_node, data=True)))

        distances = {node: float('inf') for node in self.graph.nodes}
        distances[start_node] = 0
        previous_nodes = {}

        priority_queue = [(1,0, start_node)]

        while priority_queue:
            _, g_cost, current_node = heapq.heappop(priority_queue)

            if current_node == end_node:
                path = []
                while current_node:
                    path.insert(0, current_node)
                    current_node = previous_nodes.get(current_node)
                return path

            if g_cost > distances[current_node]:
                continue

            for neighbor in self.graph.neighbors(current_node):
                edge_data = self.graph[current_node][neighbor]
                total_cost = g_cost + edge_data.get('cost', 1)  # Use 'cost' attribute as the A* heuristic
                if total_cost < distances[neighbor]:
                    distances[neighbor] = total_cost
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(priority_queue, (total_cost + self.heuristic(neighbor, end_node), total_cost, neighbor))

        return []

    def find_nearest_node(self, coords):
        return ox.get_nearest_node(self.graph, coords)
    def heuristic(self, node, goal):
        # This is a simple Euclidean distance heuristic
        x1, y1 = self.get_node_coordinates(node)
        x2, y2 = self.get_node_coordinates(goal)
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def assign_random_weights(graph):
    weighted_graph = graph.copy()
    for u, v, data in weighted_graph.edges(data=True):
        weighted_graph[u][v]['weight'] = random.uniform(1, 1000)
    return weighted_graph







@app.route('/shortest_path', methods=['GET'])
def shortest_path():
    north, south, east, west = 13.0878, 12.9630, 80.2520, 80.1206
    graph = ox.graph_from_bbox(north, south, east, west, network_type="all")
    mutable_copy = nx.Graph(graph)
    weighted_graph = assign_random_weights(mutable_copy)
    weighted_graph2=weighted_graph

    solver = ShortestPathSolver(weighted_graph)
    start_coords = request.args.get('start_coords')
    end_coords = request.args.get('end_coords')
    start_coords = tuple(map(float, start_coords.split(',')))
    end_coords = tuple(map(float, end_coords.split(',')))

    shortest_path1, total_distance1 = solver.dijkstra(start_coords, end_coords)

    solver = AStarPathSolver(weighted_graph2)

    try:
        shortest_path2 = solver.a_star(start_coords, end_coords)
        

        # Display the distance in kilometers
        total_distance_km = sum(weighted_graph[u][v]['length'] for u, v in zip(shortest_path2[:-1], shortest_path2[1:]))
        
        print("Total Distance (km):", total_distance_km)

    except ValueError as e:
        print(e)

    if(total_distance1<total_distance_km):
        shortest_path=shortest_path1
    else:
        shortest_path=shortest_path2

    coordinates = {node: (graph.nodes[node]['y'], graph.nodes[node]['x']) for node in shortest_path}
    shortest_path_coord = [coordinates[node] for node in shortest_path]
    shortest_path_coord.insert(0,start_coords)
    shortest_path_coord.append(end_coords)
    return jsonify(shortest_path_coord)

if __name__ == '__main__':
    app.run(port=5000,debug=True)