import random

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState, SetModelState, \
                            SetModelStateRequest, \
                            DeleteModel, DeleteModelRequest, \
                            SpawnEntity, SpawnEntityRequest
from geometry_msgs.msg import Wrench, Vector3, Point
from hbp_nrp_excontrol.logs import clientLogger


class Thimblerigger(object):

    def __init__(self, num_mugs, mug_sdf, ball_sdf, lift_height=1., seed=None):

        self.mug_sdf = mug_sdf
        self.ball_sdf = ball_sdf

        self.rnd = random.Random()
        self.rnd.seed(seed)

        self._mug_prefix = "mug"
        self._ball_name = "ball"
        self.mug_order = ["{}{}".format(self._mug_prefix, mug)
                          for mug in range num_mugs]
        self.lift_height = lift_height



        # ROS proxies
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
        self._step_mapping = None
        self._ball_visible = None
        self._ball_spawned = None
        self.mug_with_ball = None


    def setup(self):
        self.reset()
        self.spawn_mugs()


    def reset(self):
        self._step_mapping = [self.show_mug_with_ball,
                              self.hide_ball_under_mug,
                              self.shuffle]
        self._ball_visible = False
        self._ball_spawned = False
        self.choose_mug_for_ball()

    def step(self):

        if len(self._step_mapping) == 0:
            clientLogger.info("No more steps to execute, pick a mug now.")
            return False

        action = self._step_mapping.pop(0)
        clientLogger.info("Executing thimblerigger action {}.".format(action.__name__))
        action()
        return True

    def choose_mug_for_ball(self):
        clientLogger.info("Choosing random mug for ball.")
        self.mug_with_ball = random.choice(self.mug_names)

    def spawn_mugs(self, dx=0.5):
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

    def toggle_ball_spawned(self):
        if not self._ball_spawned:
            self._spawn_ball()
        else:
            self._despawn_ball()

        return self._ball_spawned

    def show_mug_with_ball(self):
        clientLogger.info("Showing which mug containes the ball.")
        self._spawn_ball()
        self._show_ball()
        return self._ball_spawned and self._ball_visible

    def hide_ball_under_mug(self):
        clientLogger.info("Hiding ball under mug again.")
        self._hide_ball()
        self._despawn_ball()
        return not self._ball_spawned and not self._ball_visible

    def _spawn_ball(self):
        if not self._ball_spawned:
            under_mug_pose = self._model_state_proxy(self.mug_with_ball, 'world')
            msg = SpawnEntityRequest()
            msg.entity_name = self._ball_name
            msg.entity_xml = self.ball_sdf.format(ball_name=_msg.entity_name)
            msg.initial_pose.position = under_mug_pose.pose.position
            msg.initial_pose.position.z = 0
            msg.reference_frame = "world"
            self._spawn_proxy(_msg)
            self._ball_spawned = True

    def _despawn_ball(self):
        if self._ball_spawned:
            msg = DeleteModelRequest()
            msg.model_name = self._ball_name
            self._despawn_proxy(msg)
            self._ball_spawned = False

    def toggle_ball_visible(self):
        if not self._ball_visible:
            self._show_ball()
        else:
            self._hide_ball()

        return self._ball_visible

    def _show_ball(self):
        if not self._ball_visible:
            self.move_continuosly(model_name=self.mug_with_ball,
                                  dz=self.lift_height, smoothness=100)
            self._ball_visible = True

    def _hide_ball(self):
        if self._ball_visible:
            self.move_continuosly(model_name=self.mug_with_ball,
                                  dz=-self.lift_height, smoothness=100)
            self._ball_visible = False

    def shuffle(self):
        clientLogger.info("Shuffeling mugs now.")
        pass

    def move_continuosly(self, model_name, dx=0., dy=0., dz=0., smoothness=100):
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
