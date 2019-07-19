steering_cmd_poss = [i for i in range(50, 101, 1)]
steering_real_poss = [i for i in range(80, 130, 1)]

A = steering_cmd_poss[0]
B = steering_cmd_poss[-1]
a = steering_real_poss[0]
b = steering_real_poss[-1]
    
for steering_cmd_pos in steering_cmd_poss:
    real = (steering_cmd_pos - A) * (b - a) / (B - A) + a
    print("{} -> {}".format(steering_cmd_pos, real))

    

