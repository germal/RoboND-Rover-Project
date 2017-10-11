import numpy as np
import math
import random
# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with

    print(Rover)

    # prime the last pos and time
    if Rover.last_pos is None or Rover.picking_up:
        Rover.last_pos = Rover.pos
        Rover.last_pos_time = Rover.total_time
        Rover.last_unstuck_time = Rover.total_time
        Rover.last_reverse_time = Rover.total_time

    if Rover.rock_angles is not None and Rover.mode not in ('stuck', 'reverse'):
        if len(Rover.rock_angles) > 3:
            mean_dists = np.mean(Rover.rock_dists)
            print("rock %s %s mean dists %f" % (Rover.rock_angles, Rover.rock_dists, mean_dists))

            # if its far away ignore it
            if mean_dists > 100:
                Rover.mode = 'forward'
            elif mean_dists >= 20:
                Rover.mode = 'rock'
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
            # we must be right near it so break
            elif Rover.near_sample:
                Rover.mode = 'rock'
                Rover.throttle = 0.0
                Rover.brake = Rover.brake_set
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
            else:
                # coast closer
                Rover.mode = 'rock'
                Rover.throttle = 0.05
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)

        else:
            if Rover.mode == 'rock':
                Rover.mode = 'forward'

    if Rover.near_sample == 1 and Rover.mode != 'rock':
        Rover.mode = 'stop'

    # if we are moving update the last known position with time
    def dl(a, b):
            return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    if (Rover.pos != Rover.last_pos and dl(Rover.pos, Rover.last_pos) > .1) or Rover.picking_up:
        Rover.last_pos = Rover.pos
        Rover.last_pos_time = Rover.total_time
    else:
        stuck_time = int(Rover.total_time - Rover.last_pos_time)

        # if we've been stuck for a while try stop
        if stuck_time > 15 and Rover.mode in ('stuck', 'reverse'):
            Rover.mode = 'stop'
            Rover.last_pos = Rover.pos
            Rover.last_pos_time = Rover.total_time
            Rover.last_unstuck_time = Rover.total_time
            Rover.last_reverse_time = Rover.total_time

        # if stuck in the same place
        elif stuck_time > 10 and Rover.mode == 'stuck':
            Rover.mode = 'reverse'
            Rover.last_reverse_time = Rover.total_time
            Rover.reverse_yaw = Rover.yaw - random.uniform(30, 180)
            if Rover.reverse_yaw < 0:
                Rover.reverse_yaw += 360

        elif stuck_time > 5 and Rover.mode != 'reverse':
            Rover.mode = 'stuck'

    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                nav_degrees = np.multiply(Rover.nav_angles, 180./np.pi)
                nav_mean = np.mean(nav_degrees)
                nav_std = np.std(nav_degrees)
                # wait until we've got out of the center or unstuck before hugging left
                if Rover.total_time > 10 and Rover.total_time - Rover.last_unstuck_time > 5:
                    Rover.steer = np.clip(nav_mean + nav_std*0.65, -15, 15)
                else:
                    Rover.steer = np.clip(nav_mean, -15, 15)
                print ("steer %d nav_degrees_len %s mean %f std %f time %f"
                       % (Rover.steer, len(nav_degrees), nav_mean, nav_std, Rover.total_time))
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stuck':
            # Now we're stopped and we have vision data to see if there's a path forward

            if len(Rover.nav_angles) < Rover.go_forward:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = 15 # Could be more clever here about which way to turn
            # If we're stopped but see sufficient navigable terrain in front then go!
            if len(Rover.nav_angles) >= Rover.go_forward and Rover.vel <= 0.2 and Rover.steer not in (-15.0, 15.0):
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle in opposite direction
                nav_degrees = np.multiply(Rover.nav_angles, 180./np.pi)
                nav_mean = np.mean(nav_degrees)
                nav_std = np.std(nav_degrees)
                # move to the right
                Rover.steer = np.clip(nav_mean - nav_std*0.65, -15, 15)
                # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

            if len(Rover.nav_angles) >= Rover.go_forward and Rover.vel <= 0.2 and Rover.steer in (-15.0,15.0):
                Rover.throttle = 0
                if Rover.vel > 0.0:
                    Rover.brake = Rover.brake_set
                else:
                    Rover.brake = 0
                nav_degrees = np.multiply(Rover.nav_angles, 180./np.pi)
                nav_mean = np.mean(nav_degrees)
                nav_std = np.std(nav_degrees)
                # move to the right
                Rover.steer = np.clip(nav_mean - nav_std*0.65, -15, 15)

            if len(Rover.nav_angles) >= Rover.go_forward and Rover.vel > 0.2:
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle in opposite direction
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                Rover.last_unstuck_time = Rover.total_time
                Rover.mode = 'forward'

        elif Rover.mode == 'reverse':
            reverse_time = Rover.total_time - Rover.last_reverse_time
            print ("reverse %f %f time %f" %(Rover.yaw, Rover.reverse_yaw, reverse_time))
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0

            if Rover.vel <= 0.2:
                Rover.throttle = 0
                Rover.brake = 0
                if Rover.reverse_yaw < Rover.yaw:
                    Rover.steer = -15
                else:
                    Rover.steer = +15

            if abs(Rover.reverse_yaw - Rover.yaw) <= 15.0:
                Rover.last_pos = Rover.pos
                Rover.last_pos_time = Rover.total_time
                Rover.last_unstuck_time = Rover.total_time
                Rover.mode = 'forward'

            # if we cant reverse then try something else
            if reverse_time > 5:
                Rover.last_pos = Rover.pos
                Rover.last_pos_time = Rover.total_time
                Rover.last_unstuck_time = Rover.total_time
                Rover.last_reverse_time = Rover.total_time
                Rover.mode = 'stop'

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    print("Decision ", Rover)
    return Rover
