# Nodes

Each node is an infinite repeating asynchronous loop, kind of like a mini ROS node. They run in parallel and communicate via shared files in `/tmp/o.*`. File locking ensures no corrupted reads and writes.
