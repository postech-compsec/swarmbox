

uint64 timestamp
int16 orig_id        # identity of reporter
int8 stage         # status of its swarm
    # -1: dead or aborted
    # 0: idle
    # 1: icebreak - time to know who are my inferiors.
bool complete        # true if all members have reported complete

uint8 total_count    # count of my descendants + ME
uint8 total_alive    # count of my descendants alive + ME
uint8[] descendants # the ids of my descendants (superior can reach to those id through me)

float32 pos_x       # centroid?
float32 pos_y       # centroid?
float32 pos_z       # centroid?
float32 vel_x       # velocity?
float32 vel_y       # velocity?
float32 vel_z       # velocity?
float32 discrep_x   # discrepancy
float32 discrep_y   # discrepancy
float32 discrep_z   # discrepancy

string data
