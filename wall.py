import rone, sys, math, math2, leds, velocity, poseX

########  part 0: helper functions ########
# global PWM setting for all motion
# you might need to increase this a bit if your robot doesn't move
MOTOR_PWM = 65

# stops the robot for the argument time
# arguments: time
# return: nothing
def move_stop(time):
    # student code start
    rone.motor_brake('l')
    rone.motor_brake('r')
    sys.sleep(time) 
    # student code start
    

# moves forward for the argument time
# arguments: time
# return: nothing
def move_forward(time):
    # student code start
    rone.motor_set_pwm('l',MOTOR_PWM)
    rone.motor_set_pwm('r',MOTOR_PWM)
    sys.sleep(time)
    rone.motor_brake('l')
    rone.motor_brake('r')
    # student code end

# moves backward for the argument time
# arguments: time
# return: nothing
def move_backward(time):
    # student code start
    rone.motor_set_pwm('l',-MOTOR_PWM)
    rone.motor_set_pwm('r',-MOTOR_PWM)
    sys.sleep(time)
    rone.motor_brake('l')
    rone.motor_brake('r')
    # student code end

    
# rotate right for the argument time
# arguments: time
# return: nothing
def move_rotate_right(time):
    # student code start
    rone.motor_set_pwm('l',MOTOR_PWM)
    rone.motor_brake('r')
    sys.sleep(time)
    rone.motor_brake('l')
    # student code end
    

# rotate left for the argument time
# arguments: time
# return: nothing
def move_rotate_left(time):
    # student code start
    rone.motor_set_pwm('r',MOTOR_PWM)
    rone.motor_brake('l')
    sys.sleep(time)
    rone.motor_brake('r')
    # student code end
    
# backwards rotate right for the argument time
# arguments: time
# return: nothing
def back_rotate_right(time):
    # student code start
    rone.motor_set_pwm('l',-MOTOR_PWM)
    rone.motor_brake('r')
    sys.sleep(time)
    rone.motor_brake('l')
    # student code end
    
# backwards rotate left for the argument time
# arguments: time
# return: nothing
def back_rotate_left(time):
    # student code start
    rone.motor_set_pwm('r',-MOTOR_PWM)
    rone.motor_brake('l')
    sys.sleep(time)
    rone.motor_brake('r')
    # student code end

#### Pose estimator ####
WHEEL_BASE = 78
ENCODER_MM_PER_TICKS = 0.0625

#### controller constants ####
MOTION_TV_MAX = 100
MOTION_RV_GAIN = 1300
MOTION_RV_MAX = 7000

#### mode constants ####
MODE_INACTIVE = 0
MODE_ACTIVE = 1
LED_BRIGHTNESS = 40

#### ir sensor constants ####
ccw = [1, 2]
ccw_convex = [2, 1, 0, 7]
ccw_reflex = [1]
ccw_reflex_back = [3]

cw = [6, 5]
cw_convex = [5, 6, 7, 0]
cw_reflex = [6]

#### straight line following ####
heading_to_goal = 0

#update the pose state
def pose_update(pose_state):
    # 1. Get the left and right encoder ticks
    left = rone.encoder_get_ticks("l")
    right = rone.encoder_get_ticks("r")
    
    # 2. Compute the left and right delta ticks
    # Don't forget to use encoder_delta_ticks() to properly compute the change in tick values
    dleft = velocity.encoder_delta_ticks(left, pose_state['ticksL'])
    dright = velocity.encoder_delta_ticks(right, pose_state['ticksR'])

    # 3. compute the left and right distance for each wheel
    # cast the delta ticks from step 2 to floats before you do the distance computation
    dl = float(dleft) * ENCODER_MM_PER_TICKS
    dr = float(dright) * ENCODER_MM_PER_TICKS

    # 4. save the left and right ticks to pose_state so we can measure the difference next time
    pose_state['ticksL'] = left
    pose_state['ticksR'] = right
    
    # 5. Compute the distance traveled by the center of the robot in millimeters
    center = (dr + dl) / 2.0

    # 6. Add the distance to the odometer variable in pose_state
    pose_state['odometer'] = pose_state['odometer'] + abs(center)
    
    # 7. compute the arc angle in radians
    # don't call atan here, use the small angle approximation: arctan(theta) ~ theta
    dtheta = (dr - dl) / float(WHEEL_BASE)
    
    # 8. finally, update x, y, and theta, and save them to the pose state
    # use math2.normalize_angle() to normalize theta before storing it in the pose_state
    l = ((dr - dl) / 2.0) * math.sin(90 - dtheta)
    ntheta = pose_state['theta'] + dtheta
    pose_state['x'] = (center + l) * math.cos(ntheta) + pose_state['x']
    pose_state['y'] = (center + l) * math.sin(ntheta) + pose_state['y']
    pose_state['theta'] = math2.normalize_angle(ntheta)
    return 0

def ir_bits():
    rone.ir_comms_send_message();
    sys.sleep(20)
    msg = rone.ir_comms_get_message()
    if msg != None:
        (msg, recv_list, xmit_list, range_bits) = msg
        return recv_list

def check(obs_bits):
    recv_bits = ir_bits()
    if recv_bits == None:
        return 0
    
    for bit in obs_bits:
        if not(bit in recv_bits):
            return 0
    return 1

def not_check(obs_bits):
    recv_bits = ir_bits()
    if recv_bits == None:
        return 0
    
    for bit in obs_bits:
        if not(bit in recv_bits):
            return 1
    return 0

def convex_ccw_turn():
    print 'convex ccw turn'
    previous_bearing = poseX.get_theta()
    while check(ccw_convex):
        move_rotate_right(250)
    poseX.update()
    print 'convex angle', math.degrees(math2.normalize_angle(previous_bearing - poseX.get_theta()))
    return poseX.get_theta()

# add 180 degrees to robot turning angle to get reflex vertex angle
def reflex_ccw_turn():
    print 'reflex ccw turn'
    sys.sleep(20)
    previous_bearing = poseX.get_theta()
    while not_check(ccw_reflex):
        move_forward(50)
        move_rotate_left(200)
    print 'complete rest of the turn'
    sys.sleep(20)
    if not_check(ccw_reflex_back):
        move_forward(100)
        move_rotate_left(200)
    poseX.update()
    print 'reflex_angle', math.degrees(math2.normalize_angle(poseX.get_theta() - previous_bearing))
    return poseX.get_theta()

# Compute the smallest angle difference between two angles
# This difference will lie between (-pi, pi]
# Ryan Spring - rds4
def smallest_angle_diff(current_angle, goal_angle):
    # student code start
    if (current_angle >= 0 and goal_angle >= 0) or (current_angle < 0 and goal_angle < 0):
        return math2.normalize_angle(goal_angle - current_angle)
    elif (current_angle >= 0):
        return -math2.normalize_angle(abs(current_angle) + abs(goal_angle))
    else:
        return math2.normalize_angle(abs(current_angle) + abs(goal_angle))
    # student code end

def update_motion():
    global heading_to_goal
    if check(ccw_convex):
        heading_to_goal = convex_ccw_turn()
    elif not_check(ccw_reflex):
        heading_to_goal = reflex_ccw_turn()
    else:
        bearing_error = smallest_angle_diff(poseX.get_theta(), heading_to_goal)
        rv = math2.bound(MOTION_RV_GAIN * bearing_error, MOTION_RV_MAX)
        velocity.set_tvrv(MOTION_TV_MAX, rv)
        velocity.update()

def wall_following():
    # init class variables 
    velocity.init(0.22, 40, 0.5, 0.1)
    leds.init()
    poseX.init(pose_update)

    # init variables
    wall_follow_time = 0
    pose_estimator_print_time = sys.time()
    mode = MODE_INACTIVE
    pose_old = (0.0, 0.0, 0.0)

    while True:
        # update the LED animations
        leds.update()

        # update the pose estimator
        poseX.update()
        
        if mode == MODE_ACTIVE:
            update_motion()
        else:
            move_stop(100)

        # print status every 250ms
        if sys.time() > pose_estimator_print_time:
            pose_estimator_print_time += 1000
            print 'pose', poseX.get_pose(), 'odo', poseX.get_odometer()
            if mode == MODE_INACTIVE:
                if (math2.pose_subtract(poseX.get_pose(), pose_old) != (0.0, 0.0, 0.0)): 
                    leds.set_pattern('r', 'blink_fast', int(LED_BRIGHTNESS * 1.5))
                else:
                    leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            pose_old = poseX.get_pose()

        # check the buttons.  If the red button is pressed, start wall following
        if rone.button_get_value('r'):
            if mode == MODE_INACTIVE:
                print 'start wall following'
                poseX.set_pose(0, 0, 0)
                heading_to_goal = 0
                mode = MODE_ACTIVE
                wall_follow_time = sys.time() + 8000 # 8 seconds
            
        # stop wall following after time limit
        if mode == MODE_ACTIVE:
            leds.set_pattern('g', 'blink_fast', LED_BRIGHTNESS)
            if sys.time() > wall_follow_time:
                print 'stop wall following'
                mode = MODE_INACTIVE
                move_stop(100)

wall_following()
                    

