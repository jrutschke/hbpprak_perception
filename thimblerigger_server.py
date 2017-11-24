import rospy
from std_srvs.srv import *

from hbp_nrp_excontrol.logs import clientLogger

class ThimbleriggerChallengeServer(object):
    def __init__(self):
        self.running = False

    def handle_start(self, req):
        if self.running:
            error_msg = "Challenge already started"
            clientLogger.info("StartChallengeServer: Illegal request: " + error_msg)
            return TriggerResponse(False, error_msg)
        self.running = True
        msg = "Thimblerigger challenge started!"
        clientLogger.info(msg)
        res = TriggerResponse(True, msg)

    def handle_stop(self, req):
        if not self.running:
            error_msg = "Challenge already stopped"
            clientLogger.info("StartChallengeServer: Illegal request: " + error_msg)
            return TriggerResponse(False, error_msg)
        self.running = True
        msg = "Thimblerigger challenge stopped!"
        clientLogger.info(msg)
        res = TriggerResponse(True, msg)

    def serve(self):
        clientLogger.info("Starting Thimblerigger challenge server...")
        start_service = rospy.Service("start_thimblerigger_challenge", Trigger, self.handle_start)
        stop_service = rospy.Service("stop_thimblerigger_challenge", Trigger, self.handle_stop)
