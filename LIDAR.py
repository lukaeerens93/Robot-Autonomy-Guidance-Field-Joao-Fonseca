# Processing the LIDAR data

def LIDAR(hard_obstacle_coords, State_Value):

    # Decode state0
    x0 = State_Value[1]
    y0 = State_Value[2]

    # Number of points detected by Lidar
    size_hard = hard_obstacle_coords.size


    distance = []
    bearing = []

    for i in range(0, size_hard, 1):
        x_difference = hard_obstacle_coords[i][0] - x0
        y_difference = hard_obstacle_coords[i][1] - y0
        x_difference = math.pow(x_difference, 2)
        y_difference = math.pow(y_difference, 2)
        distance[i] = math.pow(x_difference + y_difference, 0.5)
        bearing[i] = math.atan2(H_Obs[i][1] - y0, H_Obs[i][0] - x0)

    return [distance, bearing]
