import setup_path
import airsim
import math
import heapq

# Define the start and end positions for the drone
start_pos = airsim.Vector3r(0, 0, -10)
end_pos = airsim.Vector3r(30, 30, -10)

# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# Take off and set the initial drone position
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

# Define a function to calculate the Euclidean distance between two points
def distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

# Define the road network as a graph, with edges connecting adjacent road points
road_points = client.simListSceneObjects("Road.*")
graph = {point.name: {} for point in road_points}
for i, point1 in enumerate(road_points):
    for j, point2 in enumerate(road_points):
        if i != j:
            dist = distance(point1.position, point2.position)
            graph[point1.name][point2.name] = dist
            graph[point2.name][point1.name] = dist

# Define the A* algorithm for finding the shortest path
def astar(start, end, graph):
    heap = []
    visited = set()
    heapq.heappush(heap, (0, start, [start]))
    while heap:
        cost, node, path = heapq.heappop(heap)
        if node == end:
            return path
        if node not in visited:
            visited.add(node)
            for neighbor, weight in graph[node].items():
                heapq.heappush(heap, (cost + weight, neighbor, path + [neighbor]))

# Use the A* algorithm to find the shortest path along the road network
path = astar(start_pos, end_pos, graph)

# Fly the drone along the path
for i in range(len(path) - 1):
    start = client.simGetObjectPose(path[i]).position
    end = client.simGetObjectPose(path[i+1]).position
    distance = math.sqrt((end.x-start.x)**2 + (end.y-start.y)**2 + (end.z-start.z)**2)
    speed = 5.0 # m/s
    duration = distance / speed
    client.moveToPositionAsync(end.x, end.y, end.z, duration).join()

# Land the drone
client.landAsync().join()


