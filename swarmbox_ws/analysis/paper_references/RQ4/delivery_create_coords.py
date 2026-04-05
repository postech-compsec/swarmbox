# create deliverypoints.txt
# randomly generate 50 delivery points, in range of (-100, 100)
# save them as: "(north, east)" each on a new line

import random
import math
import os

rseed = 12
WORKSPACE = os.getcwd()

def generate_delivery_points(num_points=100, range_min=-100, range_max=100, sort=False, seed=None):
    delivery_points = []
    if seed is not None:
        random.seed(seed)
    for i in range(num_points):
        north = random.randint(range_min, range_max)
        east = random.randint(range_min, range_max)
        # format as "0:[33,6]"
        delivery_points.append(f"{i}: [{north},{east}]")

    if sort:
        # if sort, sort by atan2 of the coordinates
        delivery_points.sort(key=lambda point: math.atan2(float(point.split(',')[0][1:]), float(point.split(',')[1][:-1])))
    return delivery_points

def save_delivery_points(filename=os.path.join(WORKSPACE, f'config/deliverypoints_{rseed}.txt'), points=None):
    if points is None:
        points = generate_delivery_points(sort=False, seed=rseed)
    with open(filename, 'w') as file:
        for point in points:
            file.write(point + '\n')

if __name__ == "__main__":
    # create 20 sample datasets, using seeds 1 to 20
    for i in range(1, 21):
        save_delivery_points(filename=os.path.join(WORKSPACE, f'config/deliverypoints_{i}.txt'), points=generate_delivery_points(num_points=100, seed=i))
    print("Delivery points generated and saved to deliverypoints_*.txt")

