namespace = "thimblerigger"

thimblerigger_started_topic = namespace + "/start_challenge_signal"
thimblerigger_step_topic = namespace + "/step_challenge_signal"
thimblerigger_stopped_topic = namespace + "/stop_challenge_signal"

thimblerigger_start_service = namespace + "/start_challenge"
thimblerigger_step_service = namespace + "/step_challenge"
thimblerigger_stop_service = namespace + "/stop_challenge"

thimblerigger_reset_service = namespace + "/reset"
thimblerigger_show_correct_service = namespace + "/show_correct_mug"
thimblerigger_hide_correct_service = namespace + "/hide_correct_mug"
thimblerigger_shuffle_service = namespace + "/shuffle"

thimblerigger_training_topic = namespace + "/training_signal"
training_signal_frequency = 10

thimblerigger_state_topic = "group04/status"
thimblerigger_state_start_reset = "ready"
thimblerigger_state_lift_correct_mug = "lift_correct_mug"
thimblerigger_state_hide_correct_mug = "hide_correct_mug"
thimblerigger_state_shuffle = "shuffle"
thimblerigger_state_wait = "wait"


num_mugs = 3  # Number of mugs, should be 3 for the final challenge
num_shuffles = 1  # Number of permutations the configuration goes through while shuffling
seed = None  # Random seed for shuffling and choice of ball start position
movement_rate = 75 # None equals full speed. Lower values indicate slower movements,
                     # allowing to mitigate slow simulations where the camera misses some frames.

# Do not change these, they are descriptors for the mug and ball mesh
ball_sdf_xml = """
    <?xml version='1.0'?>
    <sdf version='1.5'>
      <model name='{ball_name}'>
        <pose>0 0 0 0 0 0</pose>
        <link name='{ball_name}'>
          <gravity>0</gravity>
          <inertial>
            <mass>0.057</mass>
          </inertial>
          <visual name='visual'>
            <geometry>
              <sphere>
                <radius>{radius}</radius>
              </sphere>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Green</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """


mug_sdf_xml = """
    <?xml version='1.0'?>
    <sdf version='1.5'>
      <model name='{mug_name}'>
        <pose>0 0 .7 0 0 0</pose>
        <link name='{mug_name}'>

          <gravity>0</gravity>
          <inertial>
            <mass>0.8</mass>
          </inertial>
          <visual name='visual'>
            <geometry>
              <cylinder>
                <radius>{radius}</radius>
                <length>{length}</length>
              </cylinder>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Red</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """
