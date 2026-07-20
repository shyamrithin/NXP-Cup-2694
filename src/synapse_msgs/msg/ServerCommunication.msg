uint8 src        # ID for the source component (Buggy-1, Server-2)
uint8 dest       # ID for the destination component (Buggy-1, Server-2)
uint8 uid        # Message ID / Unique identifier counter (Different Base counter for Buggy and Server)
uint8 ack        # Status field (0 for Blank, 1 for Ack)
string msg       # Variable-length byte array for the payload string
