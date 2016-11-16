import socket
import sys
import time
import threading
import pickle
import collections
import uuid

NodeInfo = collections.namedtuple('NodeInfo', ['cost', 'port'])
HEARTBEAT_INTERVAL = 0.1
UPDATE_INTERVAL = 1.0
ROUTE_UPDATE_INTERVAL = 30.0
HOST_IP = "127.0.0.1"

class Heartbeat_Packet:
    def __init__(self, origin_node):
        self.origin_node = origin_node

class LS_Packet:
    def __init__(self, origin_node, neighbours):
        self.uuid = uuid.uuid4()
        self.origin_node = origin_node
        self.neighbours_of_origin = neighbours

class Node:
    def __init__(self, node_id, node_port, config_txt):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((HOST_IP, node_port))
        self.node_id = node_id
        self.num_of_neighbour_nodes = False
        self.neighbours = {}
        self.topology = {}
        self.heartbeats = {}
        self.broadcasted_packets = set()
        self.process_config_txt(config_txt)
        
    def process_config_txt(self, config_txt):
        with open(config_txt, 'r') as config:
            self.num_of_neighbour_nodes = int(config.readline())
            for line in config:
                info_on_neighbour = line.split()
                neighbour_node_id = info_on_neighbour[0]
                neighbour_cost = float(info_on_neighbour[1])
                neighbour_port = int(info_on_neighbour[2])
                self.neighbours[neighbour_node_id] = NodeInfo(cost = neighbour_cost, port = neighbour_port)
                self.topology[self.node_id] = self.neighbours
                self.heartbeats[neighbour_node_id] = 0
        return
    
    def send_packet(self, destination_node_id, packet_as_bytes):
        self.socket.sendto(packet_as_bytes, (HOST_IP, self.neighbours[destination_node_id].port))
        return
        
    def process_ls_packet(self, ls_packet):
        if ls_packet.uuid in self.broadcasted_packets:
            return
        self.topology[ls_packet.origin_node] = ls_packet.neighbours_of_origin
        self.broadcasted_packets.add(ls_packet.uuid)
        ls_packet_as_bytes = pickle.dumps(ls_packet)
        neighbours_ids = list(self.neighbours.keys())[:]
        for neighbour_id in neighbours_ids:
            self.send_packet(neighbour_id, ls_packet_as_bytes)
        return
    
    def broadcast_packet(self, packet):
        packet_as_bytes = pickle.dumps(packet)
        neighbours_ids = list(self.neighbours.keys())[:]
        for neighbour_id in neighbours_ids:
            self.send_packet(neighbour_id, packet_as_bytes)
        return
    
    def print_least_cost_paths(self, distances, previous_nodes):
        source_node = self.node_id
        for node in distances.keys():
            if distances[node] == float("inf"):
                continue
            route = ''
            if node == source_node:
                continue
            current_node = node
            while current_node != False:
                route = current_node + route
                current_node = previous_nodes[current_node]
            print("least-cost path to node {}: {} and the cost is {}".format(node, route, distances[node]))
        return
    
    def calculate_least_cost_paths(self):
        unvisited_nodes = set()
        
        current_topology = self.topology.copy()
        nodes_in_topology = current_topology.keys()
        distances = {}
        previous_nodes = {}
        for node in nodes_in_topology:
            distances[node] = float("inf")
            previous_nodes[node] = False
            unvisited_nodes.add(node)
        distances[self.node_id] = 0
        while unvisited_nodes:
            node_with_min_cost = min(unvisited_nodes, key = distances.get)
            unvisited_nodes.remove(node_with_min_cost)
            min_node_neighbour_ids = current_topology[node_with_min_cost].keys()
            for neighbour_of_min_node in min_node_neighbour_ids:
                alternative_cost = distances[node_with_min_cost] + current_topology[node_with_min_cost][neighbour_of_min_node].cost
                if alternative_cost < distances[neighbour_of_min_node]:
                    distances[neighbour_of_min_node] = alternative_cost
                    previous_nodes[neighbour_of_min_node] = node_with_min_cost
        return self.print_least_cost_paths(distances, previous_nodes)
            
        
    def monitor_heartbeats(self):
        neighbour_ids = list(self.heartbeats.keys())[:]
        for neighbour_id in neighbour_ids:
            self.heartbeats[neighbour_id] += 1
            if self.heartbeats[neighbour_id] == 5:
                self.neighbours.pop(neighbour_id, None)
                self.topology.pop(neighbour_id, None)
                self.heartbeats.pop(neighbour_id, None)
                current_topology = self.topology.keys()
                for node_id in current_topology:
                    self.topology[node_id].pop(neighbour_id, None)
        return
                    
def lsr(argv):
    node_id = argv[1]
    node_port = int(argv[2])
    config_txt = argv[3]
    node = Node(node_id, node_port, config_txt)
    broadcast_ls_packet_timer = threading.Timer(UPDATE_INTERVAL, node.broadcast_packet, [LS_Packet(node.node_id, node.neighbours)])
    monitor_heartbeats_timer = threading.Timer(UPDATE_INTERVAL, node.monitor_heartbeats)
    broadcast_heartbeat_packet_timer = threading.Timer(HEARTBEAT_INTERVAL, node.broadcast_packet, [Heartbeat_Packet(node.node_id)])
    calculate_least_cost_paths_timer = threading.Timer(ROUTE_UPDATE_INTERVAL, node.calculate_least_cost_paths)
    broadcast_ls_packet_timer.start()
    monitor_heartbeats_timer.start()
    calculate_least_cost_paths_timer.start()
    broadcast_heartbeat_packet_timer.start()
    while True:
        if not broadcast_ls_packet_timer.isAlive():
            broadcast_ls_packet_timer = threading.Timer(UPDATE_INTERVAL, node.broadcast_packet, [LS_Packet(node.node_id, node.neighbours)])
            broadcast_ls_packet_timer.start()
        if not calculate_least_cost_paths_timer.isAlive():
            calculate_least_cost_paths_timer = threading.Timer(ROUTE_UPDATE_INTERVAL, node.calculate_least_cost_paths)
            calculate_least_cost_paths_timer.start()
        if not broadcast_heartbeat_packet_timer.isAlive():
            broadcast_heartbeat_packet_timer = threading.Timer(HEARTBEAT_INTERVAL, node.broadcast_packet, [Heartbeat_Packet(node.node_id)])
            broadcast_heartbeat_packet_timer.start()
        if not monitor_heartbeats_timer.isAlive():
            monitor_heartbeats_timer = threading.Timer(UPDATE_INTERVAL, node.monitor_heartbeats)
            monitor_heartbeats_timer.start()
        packet_recv = node.socket.recv(4096)
        packet = pickle.loads(packet_recv)
        if isinstance(packet, LS_Packet):
            if packet.origin_node in node.heartbeats.keys():
                node.heartbeats[packet.origin_node] = 0
            node.process_ls_packet(packet)
        elif isinstance(packet, Heartbeat_Packet):
            node.heartbeats[packet.origin_node] = 0

    
if __name__ == "__main__":
    lsr(sys.argv)
