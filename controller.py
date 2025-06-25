#!/usr/bin/env python


import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf




class SequenceController:
    def __init__(self):
        rospy.init_node('controller')
        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)




        self.x = self.y = self.yaw = 0.0


        self.moves = self.read_moves()
        
        try:
            raw_input("Press Enter to start execution...")
        except NameError:
            input("Press Enter to start execution...")


        self.execute_sequence()
        rospy.loginfo("[controller] Sequence complete, stopping.")
        self.cmd_pub.publish(Twist())




    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            quat.x, quat.y, quat.z, quat.w
        ])




    def read_moves(self):
        moves = []
        
        while True:
            try:
                try:
                    line = raw_input("Enter a move (0.0 to finish): ")
                except NameError:
                    line = input("Enter a move (0.0 to finish): ")
            except (EOFError, KeyboardInterrupt):
                print("\nExiting...")
                break
            
            line = line.strip()
            
            if line == '0.0':
                break
                
            parts = line.split()
            if len(parts) != 2:
                print("Invalid format. Enter: speed distance OR speed angle, or 0.0 to finish")
                continue
                
            try:
                speed, value = float(parts[0]), float(parts[1])
            except ValueError:
                print("Invalid numbers. Try again.")
                continue
            
            if abs(value) > 5.0:
                moves.append({'type':'turn', 'speed':abs(speed), 'value':math.radians(value)})
                direction = "right" if speed < 0 else "left"
                print("Added: Turn %s %.1f degrees at %.2f rad/s" % (direction, value, abs(speed)))
            else:
                moves.append({'type':'linear', 'speed':speed, 'value':abs(value)})
                direction = "forward" if speed > 0 else "backward"
                print("Added: Move %s %.2f meters at %.2f m/s" % (direction, abs(value), abs(speed)))
                
        return moves




    def execute_sequence(self):
        rate = rospy.Rate(20)
        for i, m in enumerate(self.moves):
            print("Executing move %d of %d..." % (i+1, len(self.moves)))
            if m['type'] == 'linear':
                self.linear_move(m['speed'], m['value'], rate)
            else:
                self.turn_move(m['speed'], m['value'], rate)




    def linear_move(self, speed, distance, rate):
        start_x, start_y = self.x, self.y
        target = distance
        direction = 1 if speed >= 0 else -1
        max_speed = abs(speed)
        acc = 0.2
        v = 0.0
        
        direction_str = "forward" if direction > 0 else "backward"
        print("Linear move: %.2f m %s at max %.2f m/s" % (distance, direction_str, max_speed))
        
        while not rospy.is_shutdown():
            traveled = math.hypot(self.x - start_x, self.y - start_y)
            remaining = target - traveled
            if remaining <= 0:
                break
            decel_dist = v*v / (2*acc)
            desired = max_speed if remaining > decel_dist else math.sqrt(2*acc*remaining)
            step = acc * (1.0/20.0)
            v = min(v + step, desired) if v < desired else max(v - step, desired)
            twist = Twist()
            twist.linear.x = direction * v
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())
        print("Linear move complete: traveled %.3f m %s" % (math.hypot(self.x - start_x, self.y - start_y), direction_str))
        rospy.sleep(0.5)




    def turn_move(self, speed, angle, rate):
        start_yaw = self.yaw
        target = abs(angle)
        direction = 1 if angle >= 0 else -1
        max_speed = abs(speed)
        acc = 0.5
        v = 0.0
        
        print("Turn move: %.1f degrees at max %.2f rad/s" % (math.degrees(angle), max_speed))
        
        while not rospy.is_shutdown():
            delta = math.atan2(math.sin(self.yaw - start_yaw),
                               math.cos(self.yaw - start_yaw))
            turned = abs(delta)
            remaining = target - turned
            if remaining <= 0:
                break
            decel_ang = v*v / (2*acc)
            desired = max_speed if remaining > decel_ang else math.sqrt(2*acc*remaining)
            step = acc * (1.0/20.0)
            v = min(v + step, desired) if v < desired else max(v - step, desired)
            twist = Twist()
            twist.angular.z = direction * v
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())
        print("Turn complete: turned %.1f degrees" % math.degrees(abs(delta)))
        rospy.sleep(0.5)




if __name__ == '__main__':
    SequenceController()



