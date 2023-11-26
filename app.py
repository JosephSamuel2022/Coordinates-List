from flask import Flask, request, jsonify
import osmnx as ox
from flask_cors import CORS
import heapq
import random
import networkx as nx

app = Flask(__name__)
CORS(app)




class AStarPathSolver:
    def __init__(self, graph):
        self.graph = graph

    

    def get_node_coordinates(self, node, key=0):
        # Helper function to get node coordinates, handling nested dictionaries
        try:
            value = self.graph.nodes[node]
            for _ in range(key + 1):
                value = value[key]
            return value.get('y', 0), value.get('x', 0)
        except KeyError:
            return 0, 0

    def a_star(self, start_node, end_node):
        # Add edge weights to the graph

        print("Start node:", start_node)
        print("End node:", end_node)
        print("Edges around start node:", list(self.graph.edges(start_node, data=True)))
        print("Edges around end node:", list(self.graph.edges(end_node, data=True)))

        distances = {node: float('inf') for node in self.graph.nodes}
        distances[start_node] = 0
        previous_nodes = {}

        priority_queue = [(1, 0, start_node)]

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
                total_cost = g_cost + edge_data.get('weight', 1)  # Use 'weight' attribute as the A* heuristic
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

   
    start_coords = request.args.get('start_coords')
    end_coords = request.args.get('end_coords')
    start_coords = tuple(map(float, start_coords.split(',')))
    end_coords = tuple(map(float, end_coords.split(',')))

    start_node = ox.distance.nearest_nodes(graph, start_coords[1], start_coords[0])
    goal_node = ox.distance.nearest_nodes(graph, end_coords[1], end_coords[0])

    solver = AStarPathSolver(weighted_graph2)
    
    try:
        
        shortest_path2 = solver.a_star(start_node, goal_node)
        

        # Display the distance in kilometers
        total_distance_km = sum(weighted_graph[u][v]['length'] for u, v in zip(shortest_path2[:-1], shortest_path2[1:]))
        
        print("Total Distance (km):", total_distance_km)

    except ValueError as e:
        print(e)

    
    shortest_path=shortest_path2

    coordinates = {node: (graph.nodes[node]['y'], graph.nodes[node]['x']) for node in shortest_path}
    shortest_path_coord = [coordinates[node] for node in shortest_path]
    shortest_path_coord.insert(0,start_coords)
    shortest_path_coord.append(end_coords)
    return jsonify(shortest_path_coord)

@app.route('/priority', methods=['GET'])
def priority():
    north, south, east, west = 13.0878, 12.9630, 80.2520, 80.1206
    graph = ox.graph_from_bbox(north, south, east, west, network_type="all")
    mutable_copy = nx.Graph(graph)
    weighted_graph = assign_random_weights(mutable_copy)
    weighted_graph2=weighted_graph

   
    start_coords = request.args.get('start_coords')
    end_coords = request.args.get('end_coords')
    start_coords = tuple(map(float, start_coords.split(',')))
    end_coords = tuple(map(float, end_coords.split(',')))

    start_node = ox.distance.nearest_nodes(graph, start_coords[1], start_coords[0])
    goal_node = ox.distance.nearest_nodes(graph, end_coords[1], end_coords[0])

    # For intermediate stop
    intermediate_coords = request.args.get('intermediate_coords')
    intermediate_coords = tuple(map(float, intermediate_coords.split(','))) 

    # For blockage nodes
    n1= ox.distance.nearest_nodes(graph, intermediate_coords[1], intermediate_coords[0])
    print(n1)
    end_node1=n1



    #Astar
    # Use the AStarPathSolver
    solver = AStarPathSolver(weighted_graph2)

    # Find the shortest path using A*
    try:
        shortest_path2 = solver.a_star(start_node, end_node1)

        # Display the distance in kilometers
        total_distance_km = sum(weighted_graph[u][v]['weight'] for u, v in zip(shortest_path2[:-1], shortest_path2[1:]))

    except ValueError as e:
        print(e)

    start_node1=n1

    # Find the shortest path using A*
    try:
        shortest_path4 = solver.a_star(start_node1, goal_node)
        sp2=shortest_path2+shortest_path4
        sp2.remove(n1)
        

        # Display the distance in kilometers
        total_distance_km += sum(weighted_graph[u][v]['weight'] for u, v in zip(shortest_path2[:-1], shortest_path2[1:]))
        print("Total Distance (km):", total_distance_km)



    except ValueError as e:
        print(e)
    
    shortest_path=sp2

    coordinates = {node: (graph.nodes[node]['y'], graph.nodes[node]['x']) for node in shortest_path}
    shortest_path_coord = [coordinates[node] for node in shortest_path]
    shortest_path_coord.insert(0,start_coords)
    shortest_path_coord.append(end_coords)
    return jsonify(shortest_path_coord)


@app.route('/blockage', methods=['GET'])
def blockage():
    north, south, east, west = 13.0878, 12.9630, 80.2520, 80.1206
    graph = ox.graph_from_bbox(north, south, east, west, network_type="all")
    mutable_copy = nx.Graph(graph)
    weighted_graph = assign_random_weights(mutable_copy)
    weighted_graph2=weighted_graph

   
    start_coords = request.args.get('start_coords')
    end_coords = request.args.get('end_coords')
    start_coords = tuple(map(float, start_coords.split(',')))
    end_coords = tuple(map(float, end_coords.split(',')))

    start_node = ox.distance.nearest_nodes(graph, start_coords[1], start_coords[0])
    goal_node = ox.distance.nearest_nodes(graph, end_coords[1], end_coords[0])

    blockage1_coords = request.args.get('blockage1_coords')
    blockage2_coords = request.args.get('blockage2_coords')

    blockage1_coords = tuple(map(float, blockage1_coords.split(','))) 
    blockage2_coords = tuple(map(float, blockage2_coords.split(','))) 

    n1= ox.distance.nearest_nodes(graph, blockage1_coords[1], blockage1_coords[0])
    n2= ox.distance.nearest_nodes(graph, blockage2_coords[1], blockage2_coords[0])


    end_node1 = n1



    #Astar
    # Use the AStarPathSolver
    solver = AStarPathSolver(weighted_graph2)

    # Find the shortest path using A*
    try:
        shortest_path2 = solver.a_star(start_node, end_node1)

        # Display the distance in kilometers
        total_distance_km = sum(weighted_graph[u][v]['weight'] for u, v in zip(shortest_path2[:-1], shortest_path2[1:]))

    except ValueError as e:
        print(e)
        
    edge_to_modify = (n1, n2)
    new_weight = 100000000  # Replace with the desired new weight

    if weighted_graph.has_edge(*edge_to_modify):
        weighted_graph[edge_to_modify[0]][edge_to_modify[1]]['weight'] = new_weight
    else:
        print(f"The edge {edge_to_modify} does not exist in the graph.")


    start_node1 = n1


    #Astar
    # Use the AStarPathSolver
    solver = AStarPathSolver(weighted_graph)

    # Find the shortest path using A*
    try:
        shortest_path4 = solver.a_star(start_node1,goal_node)
        sp2=shortest_path2+shortest_path4
        sp2.remove(n1)
        print("Shortest Path:", sp2)

        # Display the distance in kilometers
        total_distance_km += sum(weighted_graph[u][v]['weight'] for u, v in zip(shortest_path2[:-1], shortest_path2[1:]))
        print("Total Distance (km):", total_distance_km)

    except ValueError as e:
        print(e)

    shortest_path=sp2

    coordinates = {node: (graph.nodes[node]['y'], graph.nodes[node]['x']) for node in shortest_path}
    shortest_path_coord = [coordinates[node] for node in shortest_path]
    shortest_path_coord.insert(0,start_coords)
    shortest_path_coord.append(end_coords)
    return jsonify(shortest_path_coord)

@app.route('/blockagenpriority', methods=['GET'])
def blockagenpriority():
    north, south, east, west = 13.0878, 12.9630, 80.2520, 80.1206
    graph = ox.graph_from_bbox(north, south, east, west, network_type="all")
    mutable_copy = nx.Graph(graph)
    weighted_graph = assign_random_weights(mutable_copy)
    weighted_graph2=weighted_graph

   
    start_coords = request.args.get('start_coords')
    end_coords = request.args.get('end_coords')
    start_coords = tuple(map(float, start_coords.split(',')))
    end_coords = tuple(map(float, end_coords.split(',')))

    start_node = ox.distance.nearest_nodes(graph, start_coords[1], start_coords[0])
    goal_node = ox.distance.nearest_nodes(graph, end_coords[1], end_coords[0])

    blockage1_coords = request.args.get('blockage1_coords')
    blockage2_coords = request.args.get('blockage2_coords')

    blockage1_coords = tuple(map(float, blockage1_coords.split(','))) if blockage1_coords else None
    blockage2_coords = tuple(map(float, blockage2_coords.split(','))) if blockage2_coords else None

    intermediate_coords = request.args.get('intermediate_coords')
    intermediate_coords = tuple(map(float, intermediate_coords.split(','))) if intermediate_coords else None

    
    n1= ox.distance.nearest_nodes(graph, intermediate_coords[1], intermediate_coords[0])

    
    n2= ox.distance.nearest_nodes(graph, blockage1_coords[1], blockage1_coords[0])
    n3= ox.distance.nearest_nodes(graph, blockage2_coords[1], blockage2_coords[0])
    end_node1=n1
    end_node2=n2

    
    solver = AStarPathSolver(weighted_graph2)

    # Find the shortest path using A*
    try:
        shortest_path2 = solver.a_star(start_node, end_node1)
        shortest_path3 = solver.a_star(start_node, end_node2)
        # Display the distance in kilometers
        total_distance_km1 = sum(weighted_graph[u][v]['weight'] for u, v in zip(shortest_path2[:-1], shortest_path2[1:]))
        total_distance_km2 = sum(weighted_graph[u][v]['weight'] for u, v in zip(shortest_path3[:-1], shortest_path3[1:]))
        if(total_distance_km1<total_distance_km2):
            end_node3=n1
            end_node4=n2
            shortest_path4 = solver.a_star(end_node3, end_node4)
            total_distance_km1+=sum(weighted_graph[u][v]['weight'] for u, v in zip(shortest_path4[:-1], shortest_path4[1:]))
            edge_to_modify = (n2, n3)
            new_weight = 100000000  # Replace with the desired new weight
            if weighted_graph.has_edge(*edge_to_modify):
                weighted_graph[edge_to_modify[0]][edge_to_modify[1]]['weight'] = new_weight
            else:
                print(f"The edge {edge_to_modify} does not exist in the graph.")
            shortest_path5 = solver.a_star(end_node4,goal_node)
            total_distance_km1+=sum(weighted_graph[u][v]['weight'] for u, v in zip(shortest_path5[:-1], shortest_path5[1:]))
        else:
            end_node3=n2
            end_node4=n1
            edge_to_modify = (n1, n2)
            new_weight = 100000000  # Replace with the desired new weight
            if weighted_graph.has_edge(*edge_to_modify):
                weighted_graph[edge_to_modify[0]][edge_to_modify[1]]['weight'] = new_weight
            else:
                print(f"The edge {edge_to_modify} does not exist in the graph.")
            shortest_path4 = solver.a_star(end_node3, end_node4)
            total_distance_km2+=sum(weighted_graph[u][v]['weight'] for u, v in zip(shortest_path4[:-1], shortest_path4[1:]))
            shortest_path5 = solver.a_star(end_node4,goal_node)
            total_distance_km2+=sum(weighted_graph[u][v]['weight'] for u, v in zip(shortest_path5[:-1], shortest_path5[1:]))
            total_distance_km1=total_distance_km2
        

    except ValueError as e:
        print(e)
    
    print("Total Distance:",total_distance_km1)
#insert code
    
    shortest_path=shortest_path5

    coordinates = {node: (graph.nodes[node]['y'], graph.nodes[node]['x']) for node in shortest_path}
    shortest_path_coord = [coordinates[node] for node in shortest_path]
    shortest_path_coord.insert(0,start_coords)
    shortest_path_coord.append(end_coords)
    return jsonify(shortest_path_coord)



if __name__ == '__main__':
    app.run(port=5000,debug=True)