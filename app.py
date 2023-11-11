from flask import Flask, request, jsonify
import osmnx as ox
from flask_cors import CORS
import heapq

app = Flask(__name__)
CORS(app)

class ShortestPathSolver:
    def __init__(self, graph):
        self.graph = graph
    def dijkstra(self, start_coords, end_coords):
        start_node = ox.distance.nearest_nodes(self.graph, start_coords[1], start_coords[0])
        end_node = ox.distance.nearest_nodes(self.graph, end_coords[1], end_coords[0])
        distances = {node: float('inf') for node in self.graph.nodes}
        distances[start_node] = 0
        previous_nodes = {}
        pq = [(0, start_node)]
        while pq:
            current_distance, current_node = heapq.heappop(pq)
            if current_node == end_node:
                path = []
                while current_node:
                    path.insert(0, current_node)
                    current_node = previous_nodes.get(current_node)
                return path
            if current_distance > distances[current_node]:
                continue
            for neighbor in self.graph.neighbors(current_node):
                distance = distances[current_node] + self.graph[current_node][neighbor].get('length', 1)
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))
        return []





@app.route('/shortest_path', methods=['GET'])
def shortest_path():
    north, south, east, west = 13.0878, 12.9630, 80.2520, 80.1206
    graph = ox.graph_from_bbox(north, south, east, west, network_type="all")
    solver = ShortestPathSolver(graph)
    start_coords = request.args.get('start_coords')
    end_coords = request.args.get('end_coords')
    start_coords = tuple(map(float, start_coords.split(',')))
    end_coords = tuple(map(float, end_coords.split(',')))
    shortest_path = solver.dijkstra(start_coords, end_coords)
    coordinates = {node: (graph.nodes[node]['y'], graph.nodes[node]['x']) for node in shortest_path}
    shortest_path = [coordinates[node] for node in shortest_path]
    return jsonify(shortest_path)

if __name__ == '__main__':
    app.run(port=5000,debug=True)