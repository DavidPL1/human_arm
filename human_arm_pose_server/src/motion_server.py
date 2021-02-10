#!/usr/bin/env python
from __future__ import print_function

import os
import rospy
import numpy as np
import yaml

from time import sleep

import actionlib

from sensor_msgs.msg import JointState
from human_arm_motion_msgs.srv import SaveGesture, SaveGestureResponse
from human_arm_motion_msgs.msg import PlayGestureAction, PlayGestureGoal, PlayGestureFeedback, PlayGestureResult

from rosgraph_msgs.msg import Clock

class MotionServer():
    def __init__(self):
        rospy.init_node('human_arm_motion_server')

        shadow_preview = rospy.get_param('~shadow_preview', True)
        self.sample_frequency = rospy.get_param('~sample_frequency', 20)
        self.target_prefix = target_prefix = rospy.get_param('~target_prefix', 'goal')
        self.start_prefix = start_prefix = rospy.get_param('~start_prefix', 'start')
        self.gestures = rospy.get_param('gestures/')
        self.pub_start = None
        self.pub_target = None
        self.sim_time = rospy.get_param('use_sim_time', False)

        self.sim_time = rospy.get_param('use_sim_time', False)
        self.clock_pub = rospy.Publisher('clock', Clock, queue_size=1)
        self.clock_msg = Clock()

        self.pub_model = rospy.Publisher('input/joint_states', JointState, queue_size=1)
        if shadow_preview:
            self.pub_start = rospy.Publisher('%s/input/joint_states' % start_prefix, JointState, queue_size=1)
            self.pub_target = rospy.Publisher('%s/input/joint_states' % target_prefix, JointState, queue_size=1)

        self.motion_as = actionlib.SimpleActionServer('play_gesture', PlayGestureAction,
                                                      execute_cb=self.play_gesture, auto_start=False)
        self.save_srv = rospy.Service('save_gesture', SaveGesture, self.handle_gesture_save)

        self.motion_as.start()
        rospy.loginfo("Pose server init complete, ready to save and play gestures")
        rospy.spin()


    def handle_gesture_save(self, req):
        mode = 0  # all

        if len(req.joint_whitelist) > 0:
            mode = 1  # whitelist
        if len(req.joint_blacklist) > 0 and req.joint_blacklist[0] != '':
            if mode == 1:
                rospy.logerr("joint whitelist and blacklist can't be supplied simoultaneously!")
                return SaveGestureResponse(SaveGestureResponse.ARGUMENT_ERROR)
            mode = 2  # balcklist

        current_pose_msg = rospy.wait_for_message('joint_states', JointState)

        state_dict = dict(
            joint=current_pose_msg.name,
            position=current_pose_msg.position
        )

        if mode == 1:
            curr_pose_dict = {name:pos for name,pos in zip(current_pose_msg.name, current_pose_msg.position) if name in req.joint_whitelist}
            state_dict = dict(
                joint=curr_pose_dict.keys(),
                position=curr_pose_dict.values()
            )
        if mode == 2:
            curr_pose_dict = {name:pos for name,pos in zip(current_pose_msg.name, current_pose_msg.position) if name not in req.joint_blacklist}
            state_dict = dict(
                joint=curr_pose_dict.keys(),
                position=curr_pose_dict.values()
            )

        gesture_dict = {}
        gesture_dict[req.gesture_name] = state_dict

        # upload to param server and optionally save as yaml, if filename is provided
        self.gestures[req.gesture_name] = state_dict
        rospy.set_param('gestures', self.gestures)

        if req.file_name is not None:
            if not os.path.exists(os.path.dirname(req.file_name)) and os.path.dirname(req.file_name) != '':
                return SaveGestureResponse(SaveGestureResponse.INVALID_PATH)
            with open(req.file_name, 'w') as f:
                yaml.dump(gesture_dict, f)
                rospy.loginfo('Saved gesture in: %s' % os.path.abspath(req.file_name))

        return SaveGestureResponse(SaveGestureResponse.SUCCESS)

    def play_gesture(self, goal):
        _result = PlayGestureResult()
        _feedback = PlayGestureFeedback(seconds_left=-1, repeat_counter=0)

        for gesture_name in goal.gesture_name:
            rospy.logdebug('Checking if gesture {0} exists'.format(gesture_name))
            if gesture_name not in self.gestures:
                _result.return_state = _result.TARGET_GESTURE_UNKNOWN
                self.motion_as.set_succeeded(_result)
                return

        current_pose_msg = rospy.wait_for_message('joint_states', JointState)
        start_pose_dict = {name:pos for name,pos in zip(current_pose_msg.name, current_pose_msg.position)}

        # If not provided, use current pose as start
        if len(goal.start_gesture_name) > 0:
            if len(goal.start_gesture_name) > 1:
                weights = goal.start_gesture_weight
                rospy.logdebug('starting with linear combination of gestures {0}, (weights: {1})'.format(goal.start_gesture_name, weights))
            else:
                weights = [1]
                rospy.logdebug('starting with gesture {0}'.format(goal.start_gesture_name))

            tmp_start_pose_dict = {}
            for idx, gesture_name in enumerate(goal.start_gesture_name):
                rospy.logdebug('Checking if gesture {0} exists'.format(gesture_name))
                if gesture_name in self.gestures:
                    start_gst = self.gestures.get(gesture_name)

                    start_gst_dict = {name:pos for name,pos in zip(start_gst.get('joint'), start_gst.get('position'))}
                    # Build linear combination of gestures with a weighted sum
                    start_keys = list(set(start_gst_dict.keys() + tmp_start_pose_dict.keys()))
                    tmp_start_pose_dict = {name:tmp_start_pose_dict.get(name, 0.0) + start_gst_dict.get(name, 0.0) * weights[idx] for name in start_keys}

            # Override only the joints defined by the start gesture
            start_pose_dict = {name:tmp_start_pose_dict.get(name, start_pose_dict[name]) for name in start_pose_dict}

        stamp = rospy.Time.from_sec(t) if self.sim_time else rospy.Time.now()
        self.clock_msg.clock = stamp
        self.clock_pub.publish(self.clock_msg)

        msg_syn = JointState()
        msg_syn.header.stamp = stamp
        msg_syn.name = list(start_pose_dict.keys())
        msg_syn.position = list(start_pose_dict.values())
        self.pub_start.publish(msg_syn)
        self.pub_model.publish(msg_syn)

        if len(goal.gesture_name) > 1:
            weights = goal.gesture_weight
            if len(weights) != len(goal.gesture_name): weights = np.repeat(1.0, len(goal.gesture_name))
            rospy.logdebug('target is a linear combination of gestures {0}, (weights: {1})'.format(goal.gesture_name, weights))
        else:
            weights = [1.0]
            rospy.logdebug('target gesture is {0}'.format(goal.gesture_name))

        target_pose_dict = {}
        for idx, gesture_name in enumerate(goal.gesture_name):
            target_gst = self.gestures.get(gesture_name)

            target_gst_dict = {name:pos for name,pos in zip(target_gst.get('joint'), target_gst.get('position'))}
            rospy.logdebug('Target gesture "{0}" dict:\n{1}'.format(gesture_name, target_gst_dict))
            # Override only the joints defined by the start gesture
            target_keys = list(set(target_gst_dict.keys() + target_pose_dict.keys()))
            target_pose_dict = {name:target_pose_dict.get(name, 0.0) + target_gst_dict.get(name, 0.0) * weights[idx] for name in target_keys}

        rospy.logdebug('combined target gesture dict:\n{0}'.format(target_gst_dict))
        target_pose_dict = {name:target_pose_dict.get(name, start_pose_dict[name]) for name in start_pose_dict}

        msg_syn.name = list(target_pose_dict.keys())
        msg_syn.position = list(target_pose_dict.values())
        self.pub_target.publish(msg_syn)

        rate = rospy.Rate(self.sample_frequency) # running at 10 Hz
        t = 0 # counter for elapsed time
        period = goal.period_seconds  # includes movement to target and back to start
        repetitions = goal.repeats
        end = period * repetitions

        _feedback.seconds_left = end

        # dict keys() and value() functions return the list in their own order so this is necessary to ensure
        # the different values of the same joints are being subtracted
        diff = [start_pose_dict[name] - target_pose_dict[name] for name in target_pose_dict.keys()]

        self.motion_as.publish_feedback(_feedback)

        while t < end:
            stamp = rospy.Time.from_sec(t) if self.sim_time else rospy.Time.now()
            self.clock_msg.clock = stamp
            self.clock_pub.publish(self.clock_msg)

            msg_syn.header.stamp = stamp

            for i, pos in enumerate(target_pose_dict.values()):
                msg_syn.position[i] = pos + diff[i] * (0.5 + 0.5 * np.cos(2.*np.pi/period*t))
                self.pub_model.publish(msg_syn)
            t += rate.sleep_dur.to_sec()
            sleep(rate.sleep_dur.to_sec())
            _feedback.seconds_left = end - t
            _feedback.repeat_counter = t // period
            self.motion_as.publish_feedback(_feedback)
        rospy.logdebug("Took %f seconds for %d repetitions of %d seconds" % (t, repetitions, period))

        self.pub_target.publish(msg_syn)
        self.pub_start.publish(msg_syn)
        _result.return_state  = _result.SUCCESS
        self.motion_as.set_succeeded(_result)

if __name__ == "__main__":
    motion_server = MotionServer()
