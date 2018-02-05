# Thimblerigger Challenge of the NRP

This repository defines the challenge perception "Thimblerigger".
It is part of the KIT course "Virtual Neurorobotics".

This repo is for the **challenge definition** only, to create a solution
please fork this repository and implement your solution there.

## Installing

Assuming that you have a local install of the [NRP](https://bitbucket.org/hbpneurorobotics/neurorobotics-platform),
no special setup is needed. Simply fork this repo into the **Experiments** folder.
When starting the NRP frontend, you should now see an experiment called __PerceptionChallengeKIT__.

## Running the experiment

Start the experiment. You will be in a paused state with nothing in the world but an
iCub Robot. Once you hit the play button, three red cylinders will spawn in front of the
robot. These cylinders represent 3 mugs. One of the mugs contains a green ball.
The goal of the challenge is to:

  - Check which mug contains the ball
  - Track that mug during shuffling. Shuffling computes  *n* random permutations of the mug order and
moves the mugs into the corresponding positions.
At most 2 mugs are displaced at one time, one to either side of the original lane.
  - Tell which mug contains the ball in any way, e.g.: Publish an index, point the robot to the correct mug, ...

## Interacting with the experiment

The user can manually navigate through the challenge by sending the following ROS service requests. You can do this via command line, or
`rospy.ServiceProxy` objects. (The challenge does nothing if you do not send these requests, it is an "interactive" challenge):

| Service name | Service Type | Description |
| ------------ | ------------ | ----------- |
|/thimblerigger/reset|std_srvs/Trigger|Despawns the mugs, respawns them in the starting sorted order and chooses a new mug under which the ball is contained|
|/thimblerigger/show_correct_mug|std_srvs/Trigger|Lifts up the mug under which the ball is contained and shows the ball|
|/thimblerigger/hide_correct_mug|std_srvs/Trigger|Hides the ball again under the mug if it was lifted up|
|/thimblerigger/shuffle|std_srvs/Trigger|Randomly permutes the mugs n times. n is a parameter of the thimblerigger challenge.|

You can also use a stepping mechanism (implemented in the state machine):

| Service name | Service Type | Description |
| ------------ | ------------ | ----------- |
|/thimblerigger/start_challenge|std_srvs/Trigger|Once the simulation is running, starts the thimblerigger state machine|
|/thimblerigger/step_challenge|std_srvs/Trigger|Steps through the challenge in this order (one step per service call): `Show correct mug - Hide correct mug - Shuffle - Show correct mug - (Then the state machine exits, feel free to implement a loop/reset mechanism)`|
|/thimblerigger/stop_challenge|std_srvs/Trigger| Currently not used|

### Training signal

There is a training signal for supervised training being published under the topic `/thimblerigger/training_signal`. It publishes the index of the correct mug, as soon as it is known (meaning before the `/thimblerigger/show_correct_mug` service is called, a *-1* is published).

The 3 indices `[0, 1, 2]` are arranged as follows:
`0` is closest to the default robot starting position, `2` the furthest away.
If you reset the challenge with the service call, the training signal will be reset, too.
The index switches every time the mug has reached its goal position during shuffling, but not in between.

## What am I allowed to modify?

### What you shouldn't modify

There is a configuration file `thimblerigger_config.py`, that allows to configure things like
topic names. The paragraph above assumes that you have not changed those.

You should not fiddle with the following files (and any references to one of these files in any configuration file):

 - `state_machine.exd`
 - `stepper.py`
 - `thimblerigger_server.py`
 - `thimblerigger.py` (this is where most interesting stuff happens)

Feel free to read through them for further information how the challenge works internally,
the `thimblerigger*.py` files are well documented. You can find doxygen pages in the `docs` folder.

### Things that you should definitely change

In `bibi_configuration.bibi`:

- There is a `file`-tag within `brainModel` that points to a file that load the
brain of the robot. This is relative to the *Models* folder in the NRP.
On a local install, it is best to create a new file within the experiment repo, and put a path relative to the *Models* directory in this tag. (This is to avoid splitting the solution into multiple repos).
For example:
```xml
<brainModel>
    <file>../Experiments/hbpprak_perception/my_custom_brain.py</file>
</brainModel>
```
will allow you to have the brain file within your solution fork of the challenge repo.

- Add transfer functions, spike recorders, etc. in the bibi file, see [the documentation](https://developer.humanbrainproject.eu/docs/projects/HBP%20Neurorobotics%20Platform/1.2/nrp/tutorials/transfer_function/bibi_config.html).

### Things that you might want to change

In `thimblerigger_config.py`:
  - *num_mugs*, *num_shuffles* and *seed*.  The final challenge uses 3 cups, 1 shuffle, and the seed for shuffling and choice of which mug contains the ball should be random (i.e. `seed = None`).

Additionally, you might want to modify:

In `ExDPerceptionChallengeKIT.exc`:

- The timeout in seconds (for debugging). Default is 10 minutes.
- The robot starting position (`robotPose`-tag)


## Additional Information

 - [Demo video](https://youtu.be/aice0elP7eI)
 - [NRP](https://bitbucket.org/hbpneurorobotics/neurorobotics-platform)
 - [NRP Forum](https://forum.humanbrainproject.eu/)
 - [Rospy service clients](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29#rospy_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_the_Client_Node)
