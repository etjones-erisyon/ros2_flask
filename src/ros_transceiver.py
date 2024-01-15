#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import threading

from flask import Flask, render_template, render_template_string, redirect
from std_msgs.msg import String
from lyra_types.msg import DioInPinState, DioOutPinState
import html
from typing import Any

Topic = str
# TODO: qualify RosMessage type with whatever common traits
# they have. Notably, we should be able to get a topic 
# from a message received, right?
RosMessage = Any

class RosTransceiver(Node):
    '''
    RosTransceiver is a ROS2 node that subscribes to a set of topics
    and saves their most recent values.

    It also provides a method to publish to a topic.

    TODO: I want to add a Flask/Quart route for each topic a RosTransceiver
    subscribes to, so that the web interface can get the most recent value
    of a topic.

    This would allow a web page to query for recent values, OR, would let us
    (how?) return htmx-style templated HTML that updates when the topic changes.
    This could be used with the htmx client library to populate a whole web page
    from the server-side Python process

    '''
    def __init__(self, topic_types_in: dict[str, type], topic_types_out: dict[str, type]):
        super().__init__('lyra_ros_transceiver')

        self.current_messsages: dict[Topic, RosMessage] = {}


        # create subscriptions for all topics in topic_types_in
        for topic_name, topic_type in topic_types_in.items():
            self.create_subscription(
                topic_type, topic_name, self.topic_callback, 10)

        # create publishers for all topics in topic_types_out
        for topic_name, topic_type in topic_types_out.items():
            pub = self.create_publisher(topic_type, topic_name, 10)
            # TODO: add to list of publishers so we can
            # find them by topic later and publish there
            # as requested by the web interface


    def topic_callback(self, msg: Any):
        self.current_messsages[msg.topic_name] = msg

    def publish(self, topic: Topic, msg: RosMessage):
        pub = self.get_publisher(topic)
        pub.publish(msg)