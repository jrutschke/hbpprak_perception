import multiprocessing
import os, sys
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
import time

sys.path.append(os.path.join(os.path.dirname(__file__)))
from hbp_nrp_excontrol.logs import clientLogger
from thimblerigger_server import ThimbleriggerChallengeServer

import thimblerigger_config as tc


class Stepper(object):
    """
    Autosteps through the challenge
    """
    def __init__(self):
        self.process = None

    def run_async(self):
        self.process = multiprocessing.Process(target=self.run_challenge)
        self.process.start()
        
        clientLogger.advertise("Starting Autostepper")    

    def run_challenge(self):
        step_service = tc.thimblerigger_step_service 
        rospy.wait_for_service(step_service)     
        step_client = rospy.ServiceProxy(step_service, Trigger)

        try:
            time.sleep(2)
            res = step_client() # Lift mug 
            time.sleep(8)
            res = step_client() # hide mug
            time.sleep(8)
            res = step_client() # shuffle
            time.sleep(30)
            res = step_client() # show correct mug

        except rospy.ServiceException as exc:
            clientLogger.info("Service did not process request: " + str(exc))
