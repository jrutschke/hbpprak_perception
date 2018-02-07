import multiprocessing
import os, sys
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import time

sys.path.append(os.path.join(os.path.dirname(__file__)))
from hbp_nrp_excontrol.logs import clientLogger
from thimblerigger_server import ThimbleriggerChallengeServer

import thimblerigger_config as tc

def run_challenge():
    """
    Step through the stages of the challenge.
    """
    step_service = tc.thimblerigger_step_service
    rospy.wait_for_service(step_service)

    step_client = rospy.ServiceProxy(step_service, Trigger)
    try:
        time.sleep(3)
        res = step_client() # Lift mug
        time.sleep(3)
        res = step_client() # Hide mug
        time.sleep(3)
        res = step_client() # Shuffle
        time.sleep(5)
        res = step_client() # Show correct mug
    except rospy.ServiceException as exc:
        clientLogger.info("Service did not process request: " + str(exc))


class Stepper(object):
    """
    Dummy object for spawning a separate process for stepping through the challenge.
    This is only useful for debugging the step mode. Later, calling the step
    service will be done by the team solving the challenge.
    """
    def __init__(self):
        self.process = None

    def run_async(self):
        clientLogger.info("WARNING: using auto-stepper! Only use this in debug mode!")
        self.process = multiprocessing.Process(target=run_challenge)
        self.process.start()
