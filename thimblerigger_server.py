import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Empty
import time

from hbp_nrp_excontrol.logs import clientLogger

import thimblerigger_config as tc

class ThimbleriggerChallengeServer(object):
    def __init__(self):
        self.running = False
        self.challenge_started_pub = rospy.Publisher(tc.thimblerigger_started_topic, Empty, queue_size=10)
        self.step_pub = rospy.Publisher(tc.thimblerigger_step_topic, Empty, queue_size=10)

    def handle_start(self, req):
        """
        Currently not used, but could be used for timing etc.
        """
        if self.running:
            error_msg = "Challenge already started"
            clientLogger.info("StartChallengeServer: Illegal request: " + error_msg)
            return TriggerResponse(False, error_msg)
        self.running = True
        msg = "Thimblerigger challenge started!"
        start_time = time.time()
        while time.time() - start_time < 3:
            self.challenge_started_pub.publish(Empty())
            time.sleep(0.1)
        clientLogger.info(msg)
        return TriggerResponse(True, msg)

    def handle_stop(self, req):
        """
        Currently not used, but could be used for timing etc.
        """
        if not self.running:
            error_msg = "Challenge already stopped"
            clientLogger.info("StartChallengeServer: Illegal request: " + error_msg)
            return TriggerResponse(False, error_msg)
        self.running = True
        msg = "Thimblerigger challenge stopped!"
        clientLogger.info(msg)
        return TriggerResponse(True, msg)

    def handle_step(self, req):
        """
        Callback for stepping through the state machine.
        """
        msg = "Thimblerigger challenge: Stepping..."
        self.step_pub.publish(Empty())
        clientLogger.info(msg)
        return TriggerResponse(True, msg)

    def serve(self):
        clientLogger.info("Starting Thimblerigger challenge server...")
        start_service = rospy.Service(tc.thimblerigger_start_service, Trigger, self.handle_start)
        stop_service = rospy.Service(tc.thimblerigger_stop_service, Trigger, self.handle_stop)
        step_service = rospy.Service(tc.thimblerigger_step_service, Trigger, self.handle_step)
