import math
class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the reinvent22 champ track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[2.89403, 0.70184, 4.0, 0.07751],
                        [3.16466, 0.693, 4.0, 0.06769],
                        [3.43314, 0.68823, 4.0, 0.06713],
                        [3.73805, 0.68548, 3.86434, 0.07891],
                        [4.10749, 0.68438, 3.04208, 0.12144],
                        [4.41121, 0.68403, 2.58508, 0.11749],
                        [4.70859, 0.68388, 2.28608, 0.13009],
                        [5.32, 0.68405, 2.0678, 0.29568],
                        [5.47294, 0.68837, 1.89981, 0.08053],
                        [5.73669, 0.70621, 1.76535, 0.14975],
                        [5.99188, 0.74397, 1.65417, 0.15595],
                        [6.2125, 0.80009, 1.55679, 0.14622],
                        [6.40159, 0.8723, 1.55679, 0.13002],
                        [6.56417, 0.95947, 1.51833, 0.1215],
                        [6.70337, 1.06122, 1.46857, 0.11741],
                        [6.8203, 1.17773, 1.46857, 0.1124],
                        [6.91413, 1.30941, 1.45, 0.11151],
                        [6.98181, 1.45656, 1.45, 0.1117],
                        [7.02175, 1.61695, 1.45, 0.11399],
                        [7.02831, 1.78832, 1.45, 0.11828],
                        [6.99394, 1.9655, 1.45, 0.12447],
                        [6.91406, 2.13948, 1.45, 0.13203],
                        [6.78253, 2.29682, 1.75686, 0.11673],
                        [6.6155, 2.43265, 1.95903, 0.10989],
                        [6.42189, 2.544, 2.24954, 0.09929],
                        [6.21112, 2.6322, 2.72445, 0.08386],
                        [5.99094, 2.70254, 3.45702, 0.06686],
                        [5.76663, 2.76273, 2.62851, 0.08835],
                        [5.56291, 2.81599, 2.62851, 0.08011],
                        [5.36026, 2.87264, 2.62851, 0.08005],
                        [5.15931, 2.93486, 2.62851, 0.08003],
                        [4.96058, 3.00487, 2.62851, 0.08016],
                        [4.76448, 3.08511, 2.62851, 0.08061],
                        [4.57237, 3.18404, 2.94025, 0.07349],
                        [4.38341, 3.29902, 3.39458, 0.06516],
                        [4.19707, 3.42683, 3.49899, 0.06458],
                        [4.01268, 3.56362, 3.16088, 0.07264],
                        [3.82932, 3.70508, 2.90432, 0.07973],
                        [3.67897, 3.81731, 2.70001, 0.06949],
                        [3.52789, 3.92462, 2.53098, 0.07322],
                        [3.37566, 4.02541, 2.38594, 0.07652],
                        [3.22171, 4.11836, 2.26023, 0.07956],
                        [3.06532, 4.20236, 2.15075, 0.08254],
                        [2.90547, 4.27644, 2.05477, 0.08574],
                        [2.74071, 4.33958, 1.96267, 0.0899],
                        [2.56887, 4.39046, 1.87873, 0.09539],
                        [2.38649, 4.42699, 1.79858, 0.10342],
                        [2.18768, 4.44523, 1.67283, 0.11934],
                        [1.96237, 4.43671, 1.67283, 0.13479],
                        [1.69985, 4.38288, 1.67283, 0.16019],
                        [1.42039, 4.26083, 1.67283, 0.18229],
                        [1.16503, 4.06146, 1.67283, 0.19367],
                        [0.96753, 3.78363, 1.67283, 0.20377],
                        [0.87363, 3.43687, 2.17532, 0.16515],
                        [0.85453, 3.09651, 2.57041, 0.13262],
                        [0.8766, 2.81168, 2.49602, 0.11446],
                        [0.91229, 2.57756, 2.22451, 0.10646],
                        [0.96294, 2.31103, 2.0237, 0.13406],
                        [1.00825, 2.10289, 1.86727, 0.11408],
                        [1.0623, 1.90085, 1.73978, 0.12021],
                        [1.12998, 1.70432, 1.73978, 0.11947],
                        [1.21209, 1.52228, 1.73978, 0.11479],
                        [1.30759, 1.3607, 1.73978, 0.10788],
                        [1.41609, 1.22064, 1.73978, 0.10183],
                        [1.53931, 1.10095, 1.73978, 0.09874],
                        [1.68365, 1.00024, 2.01876, 0.08718],
                        [1.85113, 0.91238, 2.20054, 0.08594],
                        [2.04923, 0.83633, 2.43916, 0.087],
                        [2.28992, 0.77293, 2.77377, 0.08973],
                        [2.58494, 0.72608, 3.2875, 0.09086]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)