The program initializes by setting up a Node object containing all of the socket and data information about the topology and its neighbours. It processes the config textfile during this phase.
The program uses several threaded timers to run several processes including:

1. LS packet broadcaster – transmits link state packet to all of its live neighbours
2. Least cost paths – calculates least cost paths in the topology using the Djikstra’s algorithm once the global state has been established
3. Heartbeat broadcaster – transmits heartbeat packets periodically to the neighbours to remain in “Keep Alive” state
4. Heartbeat monitor – monitors the “heartbeat” of direct neighbours by incrementing a counter if a message was not received by a certain timeframe and if a certain amount of heartbeats were not received in a specific time window, the neighbour is deleted from the used data structure

The program uses a socket to receive packets from other nodes and processes them accordingly depending if they are either a heartbeat packet which resets the heartbeat counter for that node or a link state packet which also acts as a heartbeat packet and its information is stored internally. The socket is also used to send LS and Heartbeat packets to other nodes.
These processes are looped forever until user terminates.

Representation of the topology:

Each node holds information about its neighbours in a dictionary. The dictionary key is the node ID and its value contains the information (the direct cost to that neighbour node and its port number).
The node also contains a topology which is a dictionary that holds the neighbours of its respective node. In other words, the key is the node ID of that node and its value is the neighbours (using the same data structure described in the above paragraph). We can intuitively represent edge costs as “topology[from_node][to_node].cost”.

Representation of the link state packet:

The link state packet contains three variables.

1. The UUID variable: The UUID is a universally unique identifier using the uuid Python module. Each node stores each UUID that the node has broadcast in the past. This means that if the node receives another LS packet with the same UUID, the node will not broadcast that packet again, restricting excessive LS broadcasts.
2. The origin_node variable: The node ID of where the packet was first created and originated from
3. The neighbours_of_origin variable: This holds the dictionary data structure as described earlier in this report holding the information about the neighbours of the originating node.

When a node receives a link state packet for the first time, the node updates its own topology data structure by changing the value of the dictionary key of the originating node ID to the neighbours  given in the LS packet. The node then broadcast the packet to all of its neighbours. If the node receives it again, the node will not process it and drops it.
When a node receives ANY packet, the node will access the heartbeat dictionary where the key is the node and the value is its heartbeat counter and reset it back to zero. Recall that the heartbeat counter is incremented every time a packet has not been received from that specific node. If the counter reaches 5 in this instance, the node is presumed dead (regardless if it is true or not) and removed from all of the data structures described above.
