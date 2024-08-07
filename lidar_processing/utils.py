

def process_sick_data(sick_width, sick_fov, sick_data):
    HALF_AREA = 20  # check 20 degrees wide middle area
    sumx = 0
    collision_count = 0
    obstacle_dist = 0.0

    for x in range(sick_width // 2 - HALF_AREA, sick_width // 2 + HALF_AREA):
        range_val = sick_data[x]
        if range_val < 20.0:
            sumx += x
            collision_count += 1
            obstacle_dist += range_val

    # if no obstacle was detected...
    if collision_count == 0:
        return 99999.99

    obstacle_dist /= collision_count
    return ((sumx / collision_count / sick_width) - 0.5) * sick_fov