# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

std_msgs/Header header
byte level
string reporting_node
string status
byte[] data