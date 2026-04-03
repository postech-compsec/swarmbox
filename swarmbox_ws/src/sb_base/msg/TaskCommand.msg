
uint64 timestamp
int16 orig_id # commander's id
int16 dest_id # who should execute this command
int8 code
bool yaw_control # true: controls yaw, false: no yaw control given

uint8 wp_type # 0(default) for position, 1 for velocity, 2 for acceleration
float32 wp_n
float32 wp_e
float32 wp_d
float32 wp_yaw

string data