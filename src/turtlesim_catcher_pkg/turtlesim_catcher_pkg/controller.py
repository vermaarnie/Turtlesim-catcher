#!/usr/bin/env python3
import rclpy
import math
from functools import partial
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_interfaces.msg import Turtle , TurtleArr
from my_interfaces.srv import Catch
class TurtleControllerNode(Node): 
    def __init__(self):
        super().__init__("turtle_controller")
        self.catch_closest_turtle_first = True
        self.turtle_to_catch : Turtle = None
        self.target_x = 8.0
        self.target_y = 4.0

        self.pose_ : Pose =  None   
        
        self.cmd_vel_publisher = self.create_publisher(Twist , "/turtle1/cmd_vel",10)
        self.pose_subscrber = self.create_subscription(Pose, "/turtle1/pose",self.callback_pose,10)

        self.alive_turtle_subscriber  = self.create_subscription(TurtleArr, "alive_turtles",self.callback_alive_turtles,10) 
        self.control_loop_timer = self.create_timer(0.01, self.control_loop)
        self.catch_turtle_client = self.create_client(Catch, "/catch_turtle")


    def callback_pose(self, pose: Pose):
        self.pose_ = pose

        
    # For reciveing the list of the turtles spawned
    def callback_alive_turtles(self , msg : TurtleArr):
        if len(msg.turtles) >0:
            if self.catch_closest_turtle_first:
                closest_turtle = None
                closest_turtle_dis = None

                for turtle in msg. turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math. sqrt(dist_x * dist_x + dist_y * dist_y)
                    if closest_turtle == None or distance <     closest_turtle_dis:
                        closest_turtle = turtle
                        closest_turtle_dis = distance
                self.turtle_to_catch = closest_turtle
                self.target_x = self.turtle_to_catch.x
                self.target_y = self.turtle_to_catch.y

            else:
                self.turtle_to_catch = msg.turtles[0]
                self.target_x = self.turtle_to_catch.x
                self.target_y = self.turtle_to_catch.y

    # // Main logic of thee code
    def control_loop(self):
        # distance between the target and turtle using right angled triangle
        if self.pose_ == None or self.turtle_to_catch == None:
            return
        
        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y

        distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)

        cmd = Twist()

        # the command to the turtle to reach the target
        # if the turetle is at distnac emore than 0.5 then keep moving the turtle 
        if distance > 0.5:
            # position
            cmd.linear.x = 2*distance

            #  orientation (to get the nagle to reach the target, use the triangle angles usign the x and y axis) aleways be z as the turtle is in 2d pleane and will rotate about the z axis only
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta


            # Normalize the angle  should be between -pi and +pi
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            cmd.angular.z = 6*diff
        #  target reached
        else:
            #  Stop the turtle 
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_catch.name)
            self.turtle_to_catch = None
        self.cmd_vel_publisher.publish(cmd)
        

    def call_catch_turtle_service(self , turtle_name):
        while not self.catch_turtle_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for catch turtle server...")

        request = Catch.Request()
        request.name = turtle_name


        future = self.catch_turtle_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle_service, turtle_name= turtle_name))
    
    def callback_call_catch_turtle_service(self , future , turtle_name):
        response : Catch.Response = future.result()
        if not response.success:
            self.get_logger().error("Turtle " + turtle_name + " could not be removed")
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()