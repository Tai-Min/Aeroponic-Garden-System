uint8 TYPE_TEMPERATURE=0
uint8 TYPE_PRESSURE=1
uint8 TYPE_HUMIDITY=2
uint8 TYPE_CLASSIFICATION=3
uint8 TYPE_UNKNOWN=255

std_msgs/Header header
uint8 type
float64 value