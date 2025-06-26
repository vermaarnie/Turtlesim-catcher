#!/usr/bin/env python3
import rclpy
import random 
import math
from rclpy.node import Node
from functools import partial
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_interfaces.msg import Turtle,TurtleArr
from my_interfaces.srv import Catch

#send the list of turtles spawned to a topic, need to create an interface and turtle controller will use this list to catch the turtles
#using service : - spawn,kill,catch turtle
#publisher : - publishing on topic :- alive_turtle
class TurtleSpawnerNode(Node): 
    def __init__(self):
        super().__init__("turtle_spawner")
        self.turtle_name_prefix = "turtle"
        self.turtle_counter = 0
        self.alive_turtles = []
        self.spawn_client = self.create_client(Spawn , "/spawn")

        self.alive_turtles_pub = self.create_publisher(TurtleArr, "alive_turtles",10)
        self.spawn_turtle_timer = self.create_timer(0.8,self.spawn_turtle)

        self.kill_client = self.create_client(Kill , "/kill")
        self.catch_turtle_service = self.create_service(Catch, "catch_turtle", self.callback_catch_turtle)


    def callback_catch_turtle(self , request : Catch.Request, response : Catch.Response):
        # call kill the service
        self.call_kill_service(request.name)
        response.success = True
        return response

    def publish_turtles(self):
        msg = TurtleArr()
        msg.turtles = self.alive_turtles
        self.alive_turtles_pub.publish(msg)


    # spawn turtle every two second 

    def spawn_turtle(self):
        # Choosing random names and coordinates
        self.turtle_counter += 1
        name = self.turtle_name_prefix + str(self.turtle_counter)

        x= random.uniform(0.0,11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0 , 2*math.pi)

        self.call_spawn_service(name, x, y, theta)



    def call_spawn_service(self , turtle_name,x,y,theta):
        while not self.spawn_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn server...")

        request = Spawn.Request()

        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_service, request = request))

    def callback_call_spawn_service(self, future,request : Spawn.Request):
        response : Spawn.Response= future.result()

        # We get an empty string when no turtle is spawned else get the name of the turtle spawned
        if response.name != "":
            self.get_logger().info("New turtle spawned: "+ response.name)

            # Adding the turtles in the list to get them published
            new_turtle = Turtle()
            new_turtle.name = response.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta

            self.alive_turtles.append(new_turtle)
            self.publish_turtles()

    def call_kill_service(self,turtle_name):
        while not self.kill_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for KILL server...")

        request = Kill.Request()
        request.name = turtle_name


        future = self.kill_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_service, turtle_name= turtle_name))

    def callback_call_kill_service(self , future , turtle_name):
        # remove the turtle killed from the list 
        for(i , turtle) in enumerate(self.alive_turtles):
            if turtle.name == turtle_name:
                del self.alive_turtles[i]
                # Publish the new list for the controller
                self.publish_turtles()
                break



def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()