#!/usr/bin/env python

import rospy
import rospkg
import yaml

import os

from time import sleep

import numpy as np

from actionlib import SimpleActionClient
from human_arm_motion_msgs.msg import PlayGestureAction, PlayGestureGoal
from visualization_msgs.msg import Marker
from std_msgs.msg import UInt8, Empty, String

class MotionDirector():
    def __init__(self):
        rospy.init_node('human_arm_motion_director')
        self.text_frame_id = rospy.get_param('~context_info_frame_id', 'forearm_base')
        self.rec_steps = rospy.get_param('~rec_steps', None)
        self.config_subdir = rospy.get_param('~config_subdir', "default")
        self.pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        self.pub_rec_flag = rospy.Publisher('rec_flag', UInt8, queue_size=1)
        self.current_timer = None
        self.pub_rec_flag.publish(UInt8(0))

        rospy.loginfo("Connecting to motion server")
        self.motion_server = SimpleActionClient('play_gesture', PlayGestureAction)
        rospy.loginfo('Waiting for motion server')
        server_up = self.motion_server.wait_for_server(rospy.Duration(10.0))
        if not server_up:
            rospy.logerr('Timed out waiting for motion server')
            rospy.signal_shutdown('Timed out waiting for motion server')

        rospy.loginfo("Setup done, starting to play rec motion config")

        rospy.Subscriber('play_config', String, self.play_config)
        self.pub_marker_text('Motion director waiting to start')


    def start_recording(self):
        self.pub_marker_text("Recording", is_red=True)
        self.pub_rec_flag.publish(UInt8(1))


    def end_recording(self):
        self.pub_marker_text("")
        self.pub_rec_flag.publish(UInt8(0))


    def pub_marker_text(self, text, is_red=False):
        marker = Marker(
            type = Marker.TEXT_VIEW_FACING,
            text = text,
            id = 0,
            ns = "context_info"
        )

        marker.color.a = 1.0
        marker.color.r = 1.0
        if not is_red:
            marker.color.g, marker.color.b = 1.0, 1.0

        marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1
        marker.pose.position.z = 0.55
        marker.pose.orientation.w = 1.0
        marker.header.frame_id = self.text_frame_id

        self.pub_marker.publish(marker)


    def feedback_cb(self, feedback):
        if self.current_timer and self.current_timer >= feedback.seconds_left:
            self.start_recording()


    def done_cb(self, state, result):
        self.end_recording()


    def play_config(self, req):
        """Either loads a new config and plays it or just plays an already loaded config."""

        if req.data:
            rospy.loginfo("Got config: %s" % req.data)
            rospack = rospkg.RosPack()

            pkgpath = rospack.get_path('human_arm_motion_server')
            if req.data + '.yaml' not in os.listdir(os.path.join(pkgpath, 'config', self.config_subdir, 'recfiles')):
                rospy.logerr("Rec config file '%s.yaml' not found in config path (%s)" % (req.data, os.path.join(pkgpath, 'config', self.config_subdir, 'recfiles')))
                return

            # Override current config, if it exists
            try:
                rospy.delete_param('~rec_steps')
            except KeyError:
                pass

            rec_file_path = os.path.join(pkgpath, 'config', self.config_subdir, 'recfiles', req.data + ".yaml")
            with open(rec_file_path, 'r') as f:
                self.rec_steps = yaml.load(f).get('rec_steps')
            rospy.set_param('~rec_steps', self.rec_steps)
        else:
            self.rec_steps = rospy.get_param('~rec_steps', None)
            if self.rec_steps is None:
                rospy.logerr("Rec config file not provided and no config is currently loaded. Aborting...")
                return

        rospy.loginfo("Parsing and executing steps")

        rospy.logdebug('rec_steps: /n%s' % self.rec_steps)

        for step_element in self.rec_steps:
            step = step_element.get('gesture')
            rospy.loginfo('Processing step %s' % step.get('description'))

            fam_reps = int(step['familiarization_repeats'])
            repeats = int(step['repeats']) + fam_reps
            rep_period = float(step['period_seconds'])
            hold_secs = float(step.get('hold_secs', 0.0))

            if hold_secs > 0.0 and repeats > 1:
                rospy.logwarn('hold_secs and repeats parameters should not be simultaneously active! Ignoring repeats')


            if hold_secs > 0.0:
                fam_reps = 0
                repeats = 1
                self.current_timer = 2 * rep_period + hold_secs
            else:
                self.current_timer = repeats * rep_period - fam_reps * rep_period
                hold_secs = 0.0

            rospy.logdebug(f"fam_reps: {fam_reps}, reps: {repeats}, rep_period: {rep_period}, hold_secs: {hold_secs}, gesture: {step['gesture_name']}")


            if 'wait_seconds' in step.keys():
                secs = int(step.get('wait_seconds'))
                while secs > 0:
                    self.pub_marker_text('Waiting... %ds' % secs)
                    secs -= 1
                    sleep(1)

            self.pub_marker_text('Familiarization Phase')

            start_gesture_len = len(step.get('start_gesture_name', []))
            gestures_len = len(step['gesture_name'])

            motion_goal = PlayGestureGoal(
                repeats=repeats,
                hold_secs=hold_secs,
                period_seconds=rep_period,
                gesture_name=step['gesture_name'],
                gesture_weight=step.get('gesture_weight', np.repeat(1.0, gestures_len)),
                start_gesture_name=step.get('start_gesture_name', []),
                start_gesture_weight=step.get('start_gesture_weight', np.repeat(1.0, start_gesture_len))
            )

            self.motion_server.send_goal(
                motion_goal,
                feedback_cb=self.feedback_cb,
                done_cb=self.done_cb,
            )

            if self.motion_server.wait_for_result(rospy.Duration.from_sec(repeats * rep_period + 15)):
                rospy.loginfo('Done with %s' % step.get('description'))
            else:
                rospy.logerr('Timed out on %s' % step.get('description'))

        self.pub_marker_text('Motion director waiting to start')

if __name__ == "__main__":
    director = MotionDirector()
    rospy.spin()
