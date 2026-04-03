uint64 timestamp
int16 orig_id # commander's id
int16 dest_id # who should execute this command

# data of warning (list of drones that are likely to collide will be given: (drone id), (x,y,z,vx,vy,vz) will be passed)
int16[]   prox_ids
float32[] prox_data