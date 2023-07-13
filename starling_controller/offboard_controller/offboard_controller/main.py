import random
import time
import numpy as np
from functools import partial
import itertools as it
from scipy.stats import circmean

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from template_msgs.msg import NotifyVehicles, TargetAngle


class Monitor(Node):

    def __init__(self):
        super().__init__('sync_monitor')

        self.angle_timeout = Duration(seconds=0.3)

        self.notify_delay_sub = self.create_subscription(TargetAngle, '/monitor/notify_angle', self.notify_angle_cb, 10)
        self.notify_vehicles_timer = self.create_timer(1.0, self.notify_vehicles_timer_cb)

        # map between vehicle and ordering id around the circle
        # Updated by observing current ROS topics
        self.current_vehicle_map = {} 

        # map between vehicle and (last arrived theta time, last arrived theta)
        # Updated by receiving messages from individual drones
        self.vehicle_theta_map = {} 

        self.get_logger().info("Initialised")
    
    def notify_vehicles_timer_cb(self):
        curr_time = self.get_clock().now()

        vehicles = self.__get_current_vehicle_namespaces()
        self.current_vehicle_map = {k:v for v, k in enumerate(sorted(vehicles))}
        num_vehicles = len(self.current_vehicle_map)

        if num_vehicles == 0:
            self.get_logger().info(f"No Vehicles Detected")
            return
        
        # Check if any current vehicle thetas are out of date and delete them if so
        for vname in self.current_vehicle_map.keys():
            if vname in self.vehicle_theta_map:
                arr_time, theta = self.vehicle_theta_map[vname]
                if curr_time - arr_time > self.angle_timeout:
                    del self.vehicle_theta_map[vname]

        # Find circular average of all vehicles using current theta map
        vname_to_circ_average = self.circular_mean_vehicles()
        # self.get_logger().info(f"v2t: {self.vehicle_theta_map}")
        # self.get_logger().info(f"v2ta: {vname_to_circ_average}")

        msg = NotifyVehicles()
        msg.header.stamp = curr_time.to_msg()
        msg.total_vehicles = num_vehicles
 
        for vname, id in self.current_vehicle_map.items():
            msg.vehicle_id = int(id)
            topic = f'/{vname}/notify_vehicles'
            pub = self.create_publisher(NotifyVehicles, topic, 10)
            pub.publish(msg)
            self.get_logger().info(f'Sent Vehicle Notfication to {topic}')

            if vname in vname_to_circ_average:
                s_msg = TargetAngle()
                s_msg.time = curr_time.to_msg()
                s_msg.vehicle_id = vname
                s_msg.theta = vname_to_circ_average[vname]
                topic = f'/{vname}/sync_angle'
                pub  = self.create_publisher(TargetAngle, topic, 10)
                pub.publish(s_msg)
                self.get_logger().info(f'Sent sync angle to {topic} : {s_msg}')

    def notify_angle_cb(self, msg):
        self.get_logger().info(f'Theta received from {msg.vehicle_id}, at theta {msg.theta}')
        self.vehicle_theta_map[msg.vehicle_id] = (Time.from_msg(msg.time), msg.theta)
    
    def circular_mean_vehicles(self):
        num_vehicles = len(self.current_vehicle_map)
        delta_theta = 2 * np.pi / num_vehicles
        vehicle_ordering = sorted(self.current_vehicle_map, key=self.current_vehicle_map.get)
        # Idea find ideal locations relative to each vehicle, take circular mean and send that to vehicles
        ideal_theta_relative_to_each_vehicle = {v: [] for v in vehicle_ordering}
        for v, (_, t) in self.vehicle_theta_map.items():
            v_idx = vehicle_ordering.index(v)
            for i in range(num_vehicles):
                vname = vehicle_ordering[(v_idx + i) % num_vehicles]
                if vname not in self.vehicle_theta_map:
                    continue # Just in case vehicle no longer in theta map
                ideal_theta = (t + delta_theta * i)
                ideal_theta_relative_to_each_vehicle[vname].append(ideal_theta)

        return {v: circmean(ts) for v, ts in ideal_theta_relative_to_each_vehicle.items() if len(ts) > 0}

    def __get_current_vehicle_namespaces(self):
        topic_list = self.get_topic_names_and_types()
        namespaces = set()
        # self.get_logger().info('Found the following topics:')
        for topic_name, _ in topic_list:
            # self.get_logger().info(topic_name)
            if 'mavros' in topic_name:
                name = topic_name.split('/')[1]
                if name == 'mavros':
                    name = ''
                namespaces.add(name)
        self.get_logger().info(f'Found {len(namespaces)} namespaces: {",".join(namespaces)}')
        return namespaces

def main(args=None):
    rclpy.init(args=args)
    mon = Monitor()
    rclpy.spin(mon)
    mon.destroy_node()
    rclpy.shutdown()