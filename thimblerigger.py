import random
import functools

import rospy
from gazebo_msgs.srv import GetModelState, SetModelState, \
                            SetModelStateRequest, \
                            DeleteModel, DeleteModelRequest, \
                            SpawnEntity, SpawnEntityRequest
from std_srvs.srv import Trigger, TriggerResponse

from hbp_nrp_excontrol.logs import clientLogger

import thimblerigger_config as tc


def simple_trigger_callback(func):
    """
    Simple decorator to wrap a function that takes no arguments (but self)
    and returns a boolean into a ROS Trigger Service compatible function.

    :param func: The function to turn into a trigger service callback.
    :returns The same function, but it does not throw an error if it receives
             userdata and returns a TriggerResponse instead of a boolean.
             The message of the trigger response will contain
             the called function's name.
    """
    execution_state_msg = {True: "{} finished successfully.".format(func.__name__),
                       False: "{} failed.".format(func.__name__)}

    @functools.wraps(func)
    def as_trigger_callback(self, *args, **kwargs):
        result = func(self)
        return TriggerResponse(result, execution_state_msg[result])

    return as_trigger_callback


class Thimblerigger(object):

    def __init__(self, mug_sdf, ball_sdf, num_mugs=3, num_shuffles=1, lift_height=1., seed=None):
        """
        Thimblerigger is a game challenge for the NRP simulator.
        There are some mugs on the ground.
        The robot is shown a ball under one of the mugs.
        Then, the mugs are shuffled.
        In the end, the robot has to identify which mug the ball is under.

        :param mug_sdf: String in SDF format. This will be the model of the mugs.
                        Needs to contain the 'mug_name' placeholder for name replacement.
        :param ball_sdf: String in SDF format. This will be the model of the ball.
                         Needs to contain the 'ball_name' placeholder for name replacement.
        :param num_mugs: Number of mugs to spawn.
        :param num_shuffles: Number of permutations to go through when shuffling once.
        :param lift_height: Hight to lift the mug to when showing which mug the ball is under.
        :param seed: Seed for the random number generator controlling which mug the ball is under
                     and what shuffling permutations are generated.
        """

        self.mug_sdf = mug_sdf
        self.ball_sdf = ball_sdf

        self.rnd = random.Random()
        self.rnd.seed(seed)

        self._mug_prefix = "mug"
        self._ball_name = "ball"
        self.mug_order = ["{}{}".format(self._mug_prefix, mug)
                          for mug in range(num_mugs)]

        self.lift_height = lift_height
        self.num_shuffles = num_shuffles

        # Get ROS proxies
        self._move_proxy = rospy.ServiceProxy('gazebo/set_model_state',
                                              SetModelState,
                                              persistent=True)
        self._model_state_proxy =  rospy.ServiceProxy('gazebo/get_model_state',
                                                      GetModelState,
                                                      persistent=True)
        self._spawn_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_entity',
                                               SpawnEntity,
                                               persistent=True)
        self._despawn_proxy = rospy.ServiceProxy("gazebo/delete_model",
                                                 DeleteModel,
                                                 persistent=True)

        # Resettable values
        self._ball_visible = None
        self._ball_spawned = None
        self.mug_with_ball = None

        # Spawn mugs and setup resettable values
        self._setup()

        # Create services to control the thimblerigger
        self._offer_services()


    def _offer_services(self):
        """
        Start some ros services for different actions of the thimblerigger.
        The actions are:
        - Resetting the state of the thimblerigger
        - Showing the ball under the mug
        - Hiding the ball under the mug
        - Shuffling

        :returns None.
        """
        reset_service = rospy.Service(tc.thimblerigger_reset_service, Trigger, self.reset)
        show_correct_service = rospy.Service(tc.thimblerigger_show_correct_service, Trigger, self.show_mug_with_ball)
        hide_correct_service = rospy.Service(tc.thimblerigger_hide_correct_service, Trigger, self.hide_ball_under_mug)
        shuffle_service = rospy.Service(tc.thimblerigger_shuffle_service, Trigger, self.shuffle)

    def _setup(self):
        """
        Sets up some internal parameters and spawns the mugs.
        Don't call this from externally.

        :returns None.
        """
        self.reset()
        self._spawn_mugs()

    def _spawn_mugs(self, dx=0.5):
        """
        Spawns the mugs in the gazebo environment.
        Will align them along the global x-axis.

        :param dx: Space between two mugs in meters.
        :returns True.
        """
        clientLogger.info("Spawning {} mugs.".format(len(self.mug_order)))
        for i, mug_name in enumerate(self.mug_order):
            msg = SpawnEntityRequest()
            msg.entity_name = mug_name
            msg.entity_xml = self.mug_sdf.format(mug_name=mug_name)
            msg.initial_pose.position.x = i + dx
            msg.initial_pose.position.y = 0
            msg.initial_pose.position.z = 0
            msg.reference_frame = "world"
            self._spawn_proxy(msg)
        return True

    @simple_trigger_callback
    def reset(self):
        """
        Resets the state of the thimblerigger.
        Specifically, it:
        - Hides the ball
        - Chooses a new mug for the ball to be under

        :returns True.
        """
        clientLogger.info("(Re)setting thimblerigger experiment.")
        self._despawn_ball()
        self._hide_ball()
        self.choose_mug_for_ball()

        return True

    def choose_mug_for_ball(self):
        """
        Chooses a new mug for the ball to be under.

        :returns None.
        """
        clientLogger.info("Choosing random mug for ball.")
        self.mug_with_ball = random.choice(self.mug_order)

    @simple_trigger_callback
    def show_mug_with_ball(self):
        """
        Lifts the mug under which the ball is located up and shows the ball.

        :returns True, if the mug was lifted and the ball is visible.
        """
        clientLogger.info("Showing which mug contains the ball.")
        self._spawn_ball()
        self._show_ball()
        return self._ball_spawned and self._ball_visible

    @simple_trigger_callback
    def hide_ball_under_mug(self):
        """
        Lowers the mug under which the ball is located so the ball is invisible.

        :return True, if the mug was lowered and the ball is invisible.
        """
        clientLogger.info("Hiding ball under mug again.")
        self._hide_ball()
        self._despawn_ball()
        return not self._ball_spawned and not self._ball_visible

    def _spawn_ball(self):
        """
        Spawns the ball under the mug under which it should be, if it
        is not already there.

        :returns None.
        """
        if not self._ball_spawned:
            under_mug_pose = self._model_state_proxy(self.mug_with_ball, 'world')
            msg = SpawnEntityRequest()
            msg.entity_name = self._ball_name
            msg.entity_xml = self.ball_sdf.format(ball_name=msg.entity_name)
            msg.initial_pose.position = under_mug_pose.pose.position
            msg.initial_pose.position.z = 0
            msg.reference_frame = "world"
            self._spawn_proxy(msg)
            self._ball_spawned = True

    def _despawn_ball(self):
        """
        Despawns the ball, if it is there.

        :returns None.
        """
        if self._ball_spawned:
            msg = DeleteModelRequest()
            msg.model_name = self._ball_name
            self._despawn_proxy(msg)
            self._ball_spawned = False

    def _show_ball(self):
        """
        Lifts the mug under which the ball is, if it is not already lifted.

        :returns None.
        """
        if not self._ball_visible:
            self._move_continuously(model_name=self.mug_with_ball,
                                    dz=self.lift_height, smoothness=100)
            self._ball_visible = True

    def _hide_ball(self):
        """
        Lowers the mug under which the ball is, if it is lifted.

        :returns None.
        """
        if self._ball_visible:
            self._move_continuously(model_name=self.mug_with_ball,
                                    dz=-self.lift_height, smoothness=100)
            self._ball_visible = False

    @simple_trigger_callback
    def shuffle(self):
        """
        NOT IMPLEMENTED

        :returns True.
        """
        clientLogger.info("Shuffeling mugs now.")
        return True

    def _move_continuously(self, model_name, dx=0., dy=0., dz=0., smoothness=100):
        """
        Moves a gazebo object around in a smooth way in global coordinates.

        :param model_name: The name of the model to move around.
        :param dx, dy, dz: Displacement along the (x,y,z)-axes.
        :param smoothness: How smooth the movement should be.
                           The higher the smoother.

        :returns None.
        """
        # Lower smoothness constant means bigger jumps
        smoothness = max(1, smoothness)
        for i in range(smoothness):
            model_state = self._model_state_proxy(model_name, 'world')
            msg = SetModelStateRequest()
            msg.model_state.model_name = model_name
            msg.model_state.pose = model_state.pose
            msg.model_state.pose.position.x += float(dx) / smoothness
            msg.model_state.pose.position.y += float(dy) / smoothness
            msg.model_state.pose.position.z += float(dz) / smoothness
            msg.model_state.twist = model_state.twist
            msg.model_state.scale = model_state.scale
            msg.model_state.reference_frame = 'world'
            self._move_proxy(msg)
