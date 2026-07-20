uint8 ARMING_UNKNOWN = 0
uint8 ARMING_DISARMED = 1
uint8 ARMING_ARMED = 2

uint8 FUEL_UNKNOWN = 0
uint8 FUEL_CRITICAL = 1
uint8 FUEL_LOW = 2
uint8 FUEL_NOMINAL = 3
uint8 FUEL_OVER_CAPACITY = 4

uint8 JOY_UNKNOWN = 0
uint8 JOY_DISABLED = 1
uint8 JOY_LOSS = 2
uint8 JOY_NOMINAL = 3

uint8 MODE_UNKNOWN = 0
uint8 MODE_MANUAL = 1
uint8 MODE_AUTO = 2
uint8 MODE_CMD_VEL = 3
uint8 MODE_CALIBRATION = 4

uint8 SAFETY_UNKNOWN = 0
uint8 SAFETY_SAFE = 1
uint8 SAFETY_UNSAFE = 2

uint8 SYNAPSE_UNKNOWN = 0
uint8 SYNAPSE_DISABLED = 1
uint8 SYNAPSE_LOSS = 2
uint8 SYNAPSE_NOMINAL = 3

std_msgs/Header header
uint8 arming
uint8 joy
uint8 fuel
uint8 mode
uint8 safety
uint8 synapse
string status_message
uint8 fuel_percentage
float32 power
uint32 request_seq
bool request_rejected
