import numpy as np

# winning_score = 50 for a full game of molkky
winning_score = 10
back_score = winning_score//2

# x and z position that Sawyer throws from
throw_x_pos = 0.7
throw_z_pos = 0.3

# The limits of the camera field of view
max_x_pixel_left = 0
max_x_pixel_right = 600

# The limits of sawyers's movement corresponding to field of view
max_sawyer_left = 0
max_sawyer_right = 0.35

color_dict = {
    "white":  [ 223.34875017, 223.83849807, 225.89653249],
    "yellow": [ 247.74516109, 223.6023088 ,  29.2894295 ],
    "green":  [  56.2907913 , 148.41884933, 118.51122211],
    "red":    [ 238.99080433,  50.30322062,  72.59746392]
}

color_points = {
    "white":  1,
    "yellow": 2,
    "green":  3,
    "red":    4
}
largest_pin_value = max(color_points.values())
