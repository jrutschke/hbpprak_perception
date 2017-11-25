thimblerigger_started_topic = "start_thimblerigger_challenge_signal"
thimblerigger_step_topic = "step_thimblerigger_challenge_signal"
thimblerigger_stopped_topic = "stop_thimblerigger_challenge"

thimblerigger_start_service = "start_thimblerigger_challenge"
thimblerigger_step_service = "step_thimblerigger_challenge"
thimblerigger_stop_service = "stop_thimblerigger_challenge"

ball_sdf_xml = """
    <?xml version='1.0'?>
    <sdf version='1.5'>
      <model name='{ball_name}'>
        <pose>0 0 .3 0 0 0</pose>
        <link name='{ball_name}'>
          <gravity>0</gravity>
          <inertial>
            <mass>0.057</mass>
          </inertial>
          <visual name='visual'>
            <geometry>
              <sphere>
                <radius>0.15</radius>
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
                <radius>.2</radius>
                <length>.7</length>
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
