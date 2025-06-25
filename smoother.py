#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent, Led, Sound


# CONSOLE OUTPUT CONTROL
ENABLE_CONSOLE_OUTPUT = True  # Set to False to disable console prints


smoothed_linear = 0.0
smoothed_angular = 0.0
desired_linear = 0.0
desired_angular = 0.0


stop_by_bumper = True
allow_backward = True
flash_led_backward = True
emergency_brake = False
smoothing_mode = 1


bumper_hit = False
cliff_detected = False
wheel_dropped = False


led1_pub = None
led2_pub = None
sound_pub = None
twist_pub = None


ECO_MAX_STEP_LINEAR = 0.02
ECO_MAX_STEP_ANGULAR = 0.03
SPORT_MAX_STEP_LINEAR = 0.04
SPORT_MAX_STEP_ANGULAR = 0.06


def twist_callback(data):
    global desired_linear, desired_angular
    desired_linear = data.linear.x
    desired_angular = data.angular.z


def command_callback(data):
    global stop_by_bumper, allow_backward, flash_led_backward, emergency_brake, smoothing_mode
    array = data.data
    if len(array) >= 5:
        old_backward = allow_backward
        old_flash = flash_led_backward
        old_bumper_stop = stop_by_bumper
        old_emergency = emergency_brake
        old_smoothing = smoothing_mode
        
        stop_by_bumper = bool(array[0])
        allow_backward = bool(array[1])
        flash_led_backward = bool(array[2])
        emergency_brake = bool(array[3])
        smoothing_mode = array[4]
        
        # Provide feedback when ANY setting changes
        settings_changed = False
        if old_bumper_stop != stop_by_bumper:
            if ENABLE_CONSOLE_OUTPUT:
                rospy.loginfo("*** BUMPER STOP: {} ***".format("ENABLED" if stop_by_bumper else "DISABLED"))
            settings_changed = True
        if old_backward != allow_backward:
            if ENABLE_CONSOLE_OUTPUT:
                rospy.loginfo("*** BACKWARD MOVEMENT: {} ***".format("ENABLED" if allow_backward else "DISABLED"))
            settings_changed = True
        if old_flash != flash_led_backward:
            if ENABLE_CONSOLE_OUTPUT:
                rospy.loginfo("*** LED/SOUND WARNINGS: {} ***".format("ENABLED" if flash_led_backward else "DISABLED"))
            settings_changed = True
        if old_emergency != emergency_brake:
            if ENABLE_CONSOLE_OUTPUT:
                rospy.loginfo("*** EMERGENCY BRAKE: {} ***".format("ENGAGED" if emergency_brake else "DISENGAGED"))
            settings_changed = True
        if old_smoothing != smoothing_mode:
            if ENABLE_CONSOLE_OUTPUT:
                mode_names = ["OFF", "ECO", "SPORT"]
                rospy.loginfo("*** SMOOTHING MODE: {} ***".format(mode_names[smoothing_mode] if smoothing_mode < len(mode_names) else "UNKNOWN"))
            settings_changed = True
            
        # Update console display when settings change
        if settings_changed:
            print_status()


def bumper_callback(data):
    global bumper_hit
    msg = ""
    if data.bumper == 0:
        msg += "left bumper is "
    elif data.bumper == 1:
        msg += "center bumper is "
    else:
        msg += "right bumper is "


    if data.state == 0:
        msg += "released."
        bumper_hit = False
        if ENABLE_CONSOLE_OUTPUT:
            rospy.loginfo("*** BUMPER RELEASED ***")
    else:
        msg += "pressed."
        bumper_hit = True
        if ENABLE_CONSOLE_OUTPUT:
            rospy.loginfo("*** BUMPER PRESSED: Emergency reverse available ***")


    if ENABLE_CONSOLE_OUTPUT:
        rospy.loginfo(msg)


def cliff_callback(data):
    global cliff_detected
    msg = ""
    if data.sensor == 0:
        msg += "left sensor detects "
    elif data.sensor == 1:
        msg += "center sensor detects "
    else:
        msg += "right sensor detects "


    if data.state == 0:
        msg += "floor"
        cliff_detected = False
    else:
        msg += "cliff"
        cliff_detected = True


    if ENABLE_CONSOLE_OUTPUT:
        rospy.loginfo("%s - %i" % (msg, data.bottom))


def wheel_drop_callback(data):
    global wheel_dropped
    msg = ""
    if data.wheel == 0:
        msg += "Left wheel is "
    else:
        msg += "Right wheel is "


    if data.state == 0:
        msg += "raised."
        wheel_dropped = False
    else:
        msg += "dropped"
        wheel_dropped = True


    if ENABLE_CONSOLE_OUTPUT:
        rospy.loginfo(msg)


def should_stop_robot():
    # Check if we should allow backward movement when bumper is hit
    if stop_by_bumper and bumper_hit:
        if desired_linear < -0.01:  # Trying to move backward
            if ENABLE_CONSOLE_OUTPUT:
                rospy.loginfo_throttle(0.5, "*** BUMPER HIT: ALLOWING EMERGENCY REVERSE ***")
            return False  # Allow emergency reverse
        else:
            if ENABLE_CONSOLE_OUTPUT:
                rospy.loginfo_throttle(1.0, "*** BUMPER HIT: BLOCKING FORWARD/STOP MOVEMENT ***")
            return True   # Block forward or stop movement


    # Standard safety checks
    if cliff_detected:
        return True
    if wheel_dropped:
        return True
    if emergency_brake:
        return True


    return False


def flash_leds_and_sound():
    global led1_pub, led2_pub, sound_pub
    if led1_pub is not None and led2_pub is not None:
        led = Led()
        led.value = 2  # Orange color for warning
        led1_pub.publish(led)
        led2_pub.publish(led)
    if sound_pub is not None:
        sound = Sound()
        sound.value = 0  # Warning beep
        sound_pub.publish(sound)


def turn_off_leds():
    global led1_pub, led2_pub
    if led1_pub is not None and led2_pub is not None:
        led = Led()
        led.value = 0  # Turn off LEDs
        led1_pub.publish(led)
        led2_pub.publish(led)


def apply_backward_restrictions(linear_vel):
    if linear_vel < -0.01:  # Trying to move backward
        # FIXED LOGIC: Check allow_backward setting first, then bumper override
        if allow_backward:
            if ENABLE_CONSOLE_OUTPUT:
                rospy.loginfo_throttle(1.0, "*** BACKWARD ALLOWED: Normal backward movement enabled ***")
            return linear_vel
        elif bumper_hit:
            if ENABLE_CONSOLE_OUTPUT:
                rospy.loginfo_throttle(0.5, "*** BACKWARD ALLOWED: Emergency escape from bumper ***")
            return linear_vel
        else:
            if ENABLE_CONSOLE_OUTPUT:
                rospy.loginfo_throttle(1.0, "*** BACKWARD BLOCKED: Disabled and no bumper contact ***")
            if flash_led_backward:
                flash_leds_and_sound()
            return 0.0
    
    return linear_vel


def smooth_value(current_val, desired_val, max_step):
    if smoothing_mode == 0:
        return desired_val
    delta = desired_val - current_val
    if abs(delta) <= max_step:
        return desired_val
    return current_val + (max_step if delta > 0 else -max_step)


def timer_callback(event):
    global smoothed_linear, smoothed_angular
    
    if should_stop_robot():
        smoothed_linear = 0.0
        smoothed_angular = 0.0
    else:
        target_linear = apply_backward_restrictions(desired_linear)
        
        if smoothing_mode == 1:
            max_step_linear = ECO_MAX_STEP_LINEAR
            max_step_angular = ECO_MAX_STEP_ANGULAR
        elif smoothing_mode == 2:
            max_step_linear = SPORT_MAX_STEP_LINEAR
            max_step_angular = SPORT_MAX_STEP_ANGULAR
        else:
            max_step_linear = max_step_angular = 1.0


        smoothed_linear = smooth_value(smoothed_linear, target_linear, max_step_linear)
        smoothed_angular = smooth_value(smoothed_angular, desired_angular, max_step_angular)


    # Handle LED flashing while moving backward
    if flash_led_backward and smoothed_linear < -0.01:  # Moving backward AND feature enabled
        flash_leds_and_sound()
        if ENABLE_CONSOLE_OUTPUT:
            rospy.loginfo_throttle(1.0, "*** MOVING BACKWARD: LEDs flashing and sound playing ***")
    else:
        # Turn off LEDs when not reversing OR feature is disabled
        turn_off_leds()


    twist = Twist()
    twist.linear.x = smoothed_linear
    twist.angular.z = smoothed_angular
    twist_pub.publish(twist)


def get_backward_status():
    if allow_backward:
        return "ENABLED"
    elif bumper_hit:
        return "EMERGENCY ONLY (bumper pressed)"
    else:
        return "DISABLED"


def print_status():
    if not ENABLE_CONSOLE_OUTPUT:
        return
        
    print("\n" + "="*50)
    print("           SMOOTHER NODE STATUS")
    print("="*50)
    print("Bumper Stop:     {}".format("ENABLED" if stop_by_bumper else "DISABLED"))
    print("Backward Move:   {}".format(get_backward_status()))
    print("LED/Sound Warn:  {}".format("ENABLED" if flash_led_backward else "DISABLED"))
    print("Emergency Brake: {}".format("ENGAGED" if emergency_brake else "DISENGAGED"))
    print("Smoothing Mode:  {}".format(["OFF", "ECO", "SPORT"][smoothing_mode]))
    print("Current Bumper:  {}".format("PRESSED" if bumper_hit else "RELEASED"))
    print("="*50)
    print("JOYSTICK CONTROLS:")
    print("Button B: Toggle Backward Movement")
    print("Button X: Toggle LED/Sound Warnings")
    print("="*50)
    print("BACKWARD MOVEMENT LOGIC:")
    print("• Normal: Allowed when 'Backward Move' is ENABLED")
    print("• Emergency: Always allowed when bumper is pressed")
    print("  (even if 'Backward Move' is DISABLED)")
    print("="*50)
    print("")


def main():
    global twist_pub, led1_pub, led2_pub, sound_pub
    rospy.init_node('smoother')


    twist_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    led1_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
    led2_pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=10)
    sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)


    rospy.Subscriber('/robot_twist', Twist, twist_callback)
    rospy.Subscriber('/robot_command', Int32MultiArray, command_callback)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)
    rospy.Subscriber('/mobile_base/events/cliff', CliffEvent, cliff_callback)
    rospy.Subscriber('/mobile_base/events/wheel_drop', WheelDropEvent, wheel_drop_callback)


    if ENABLE_CONSOLE_OUTPUT:
        rospy.loginfo("*** SMOOTHER NODE STARTED ***")
        print_status()
    else:
        rospy.loginfo("Smoother node started (console output disabled)")
    
    rospy.Timer(rospy.Duration(0.01), timer_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



