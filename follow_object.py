
import numpy as np
import std_msgs.msg
@nrp.MapSpikeSink("motors_down", nrp.brain.down, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_left", nrp.brain.left, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_up", nrp.brain.up, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_right", nrp.brain.right, nrp.leaky_integrator_alpha)
@nrp.MapRobotSubscriber("joint_state_sub", Topic("/robot/joints", sensor_msgs.msg.JointState))
@nrp.MapRobotPublisher("neck_pitch", Topic("/robot/neck_pitch/pos", std_msgs.msg.Float64))
@nrp.MapRobotPublisher("neck_yaw", Topic("/robot/neck_yaw/pos", std_msgs.msg.Float64))
@nrp.MapRobotPublisher('eye_tilt_pos', Topic('/robot/eye_tilt/pos', std_msgs.msg.Float64))
@nrp.MapRobotSubscriber("state", Topic("/group04/status", std_msgs.msg.String))
@nrp.MapVariable("previous_state", initial_value=None, scope=nrp.GLOBAL)
@nrp.Neuron2Robot()

def follow_object(t, motors_down, motors_left, motors_up, motors_right, joint_state_sub, state, neck_pitch, neck_yaw, eye_tilt_pos, previous_state):

	# we need the current joint positions
	if joint_state_sub is not None:
		joint_names = joint_state_sub.value.name
		joint_positions = joint_state_sub.value.position
		current_pitch = joint_positions[joint_names.index("neck_pitch")]
		current_yaw = joint_positions[joint_names.index("neck_yaw")]
		current_tilt = joint_positions[joint_names.index("eye_tilt")]
		# we need to move different for each state
		if (state.value is not None):
			#set current state
			current_state = state.value.data
			#if in state ready, set to initial position
			if (current_state == "ready"):
				# set head and eyes to start position, so we can see the mugs
				neck_pitch.send_message(std_msgs.msg.Float64(-0.5))
				neck_yaw.send_message(std_msgs.msg.Float64(0.0))
				eye_tilt_pos.send_message(std_msgs.msg.Float64(0.0))
			#when cup is lifted we need to center on the green ball
			elif (current_state == "lift_correct_mug"):
				#set eyes to neutral position
				eye_tilt_pos.send_message(std_msgs.msg.Float64(0.0))

				#scaling factor for motor neurons
				scaling_factor = 0.15

				#neck pitch only if neurons for up or down spike
				if (motors_up.voltage != 0.0) | (motors_down.voltage != 0.0):
					pitch = current_pitch + scaling_factor * (motors_up.voltage - motors_down.voltage)
					neck_pitch.send_message(std_msgs.msg.Float64(pitch))
				#neck yaw only if neurons for left  or right spike
				if (motors_left.voltage != 0.0) | (motors_right.voltage != 0.0):
					yaw = current_yaw + scaling_factor * ( motors_left.voltage - motors_right.voltage)
					neck_yaw.send_message(std_msgs.msg.Float64(yaw))
			#when cup is hidden and we shuffle we want to follow the red mug with the ball 
			elif ((current_state == "hide_correct_mug") | (current_state == "shuffle")):
				#scaling factor
				scaling_factor = 0.015
				
				#eye tilt to follow red cup only when up or down neurons spike
				if (motors_up.voltage != 0.0) | (motors_down.voltage != 0.0):
					tilt = current_tilt + scaling_factor * (motors_up.voltage - motors_down.voltage)
					eye_tilt_pos.send_message(std_msgs.msg.Float64(tilt))
				#neck yaw only when left or right neurons spike
				if (motors_left.voltage != 0.0) | (motors_right.voltage != 0.0):
					yaw = current_yaw + scaling_factor * ( motors_left.voltage - motors_right.voltage)
					neck_yaw.send_message(std_msgs.msg.Float64(yaw))
			# when we are in wait state, advertise our guess where the green ball is hidden
			elif (current_state == "wait"):
				#advertise mug prediction
				if (current_yaw < -0.18):
					clientLogger.advertise("Prediciton: Right Mug")
				elif (current_yaw > 0.18):
					clientLogger.advertise("Prediciton: Left Mug")
				else:
					clientLogger.advertise("Prediciton: Center Mug")
				
			#set previous state
			if (current_state != previous_state):
				previous_state = current_state
			
