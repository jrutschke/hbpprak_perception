# authors: Benjamin Alt, Felix Schneider (main idea) and Jacqueline Rutschke 
import numpy as np
import sensor_msgs.msg
import std_msgs.msg
from cv_bridge import CvBridge
@nrp.MapRobotSubscriber("camera", Topic("/icub_model/left_eye_camera/image_raw", sensor_msgs.msg.Image))
@nrp.MapSpikeSource("sensors", nrp.map_neurons(range(0, nrp.config.brain_root.resolution ** 2), lambda i: nrp.brain.sensors[i]), nrp.dc_source)
@nrp.MapRobotSubscriber("state", Topic("/group04/status", std_msgs.msg.String))
@nrp.MapRobotPublisher("debug_window_pub", Topic("/group04/debug_image", sensor_msgs.msg.Image))
@nrp.MapVariable("previous_state", initial_value=None, scope=nrp.GLOBAL)
@nrp.Robot2Neuron()
def detect_object(t, camera, sensors, state, debug_window_pub, previous_state):
	resolution = nrp.config.brain_root.resolution
	# Grab image from left eye
	image_msg = camera.value
	
	# only if we have an image
	if image_msg is not None:
		#create CvBridge and convert to rgb8 image
		cvBridge = CvBridge()
		img = cvBridge.imgmsg_to_cv2(image_msg, "rgb8")
		#get shape
		img_height, img_width, color_dim = img.shape
		#only if state is not None
		if (state.value is not None):
			#set current 
			current_state = state.value.data
			# if we lift the mug after 
			if ((previous_state.value == "wait") & (current_state == "lift_correct_mug")):
				#debug window is just to show what the algorithm is using
				debug_window_pub.send_message(cvBridge.cv2_to_imgmsg(img, encoding="rgb8"))
			# if we lift the mug we want to detect the green ball
			elif (current_state == "lift_correct_mug"):
				#debug window is just to show what the algorithm is using
				debug_window_pub.send_message(cvBridge.cv2_to_imgmsg(img, encoding="rgb8"))

				#scaling factor and threshold
				amp_scaling_factor = 32.
				green_threshold = 0.4
				
				#Read image into array
				col_width = img_width // resolution
				row_height = img_height // resolution
				for row_idx in range(resolution):
					for col_idx in range(resolution):
						#set x and y start and end points for tiled image
						x_start = col_idx * col_width
						x_end = x_start + col_width
						y_start = row_idx * row_height
						y_end = y_start + row_height
						#calculate mean values for each colour in each tile
						# img[x_of_image, y_of_image, colour_channel]
						mean_red = np.mean(img[y_start:y_end,x_start:x_end,0])
						mean_green = np.mean(img[y_start:y_end,x_start:x_end,1])
						mean_blue = np.mean(img[y_start:y_end,x_start:x_end,2])
						#since we want to detect green, calculate green proportion
						green_proportion = mean_green / float(mean_red + mean_green + mean_blue)
						#set index
						idx = row_idx * resolution + col_idx
						#set amplitude for neurons and send to sensor neurons
						amp = amp_scaling_factor * green_proportion if green_proportion > green_threshold else 0
						sensors[idx].amplitude = amp
			#if we are in hide_correct_mug or shuffle we want to detect the red mug
			elif ((current_state == "hide_correct_mug") | (current_state == "shuffle")):
				
				#scaling factor and threshold		
				red_threshold = 0.7
				amp_scaling_factor = 4.
				
				# Detect red in the center of the image
				window_factor = 5
				window_width = img_width // window_factor
				window_height = img_height // window_factor
				start_px_x = (window_factor // 2) * window_width
				start_px_y = (window_factor // 2) * window_height
				col_width = window_width // resolution
				row_height = window_height // resolution
	
				# Split central area of image into regions of same size
				# Loop over neurons in retina...
				for row_idx in range(resolution):
					for col_idx in range(resolution):
						x_start = start_px_x + col_idx * col_width
						x_end = x_start + col_width
						y_start = start_px_y + row_idx * row_height
						y_end = y_start + row_height
						#calculate mean values for each colour in each tile
						mean_red = np.mean(img[y_start:y_end,x_start:x_end,0])
						mean_green = np.mean(img[y_start:y_end,x_start:x_end,1])
						mean_blue = np.mean(img[y_start:y_end,x_start:x_end,2])
						#since we want to detect red, calculate red proportion
						red_proportion = mean_red / float(mean_red + mean_green + mean_blue)
						#set index
						idx = row_idx * resolution + col_idx
						amp = amp_scaling_factor * red_proportion if red_proportion > red_threshold else 0
						#set amplitude for neurons and send to sensor neurons
						sensors[idx].amplitude = amp
				#debug windows now shows a zoomed image
				debug_window_pub.send_message(cvBridge.cv2_to_imgmsg(img[start_px_y:start_px_y + window_height, start_px_x:start_px_x + window_width,:], encoding="rgb8"))
			#other state: just show what the robot is seeing
			else:
				debug_window_pub.send_message(cvBridge.cv2_to_imgmsg(img, encoding="rgb8"))
			
			