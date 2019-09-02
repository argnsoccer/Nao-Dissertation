# Nao-Dissertation
Experiments in developing Learning for a NAO 

src: Experiment code and output

If wanting to use physical robot, set up a mobile hotspot with the given details (from Instructions... not sure if this can be public knowledge or not).

If not, set IP to 127.0.0.1 for simulation.

Install Webots and naoqisim (See report for help)
(naoqisim - https://github.com/cyberbotics/naoqisim)

Experiment 1: run getandsaveanimage.py, replacing the IP with the one the robot says, or 127.0.0.1 for a simulation

Experiment 2: run AndreTrackAndKick/SearchKickAndre.py or LavyMarshallTrackAndKick/Full_Search_and_Kick.py 
      -NOTE: You will probably have to change the mask details depending on the red ball you use. You can use AndreTrackAndKick/ColorPicker.py to threshold your ball's HSV values.
      
Experiment 3: run ddpg_robot.py (changing IP as necessary). Change the agent/critic networks or algorithm and optimization as desired in ddpg_robot.py. 

  If you want your own new custom environment, copy+paste env_basic.py and remove everything in the step, init, and reset functions. Or       just create a new class in envs/ that inherits from gym.Env that has the methods: init, reset, step, and render implemented. Make sure to register any new environments in the __init__.pys.
    
  You can also just change the reward structure in env_basic.py however you want or just make a bunch of branches using the same env_basic.py with different actions/rewards/etc. You won't need to reinstall the package every time you update env_basic.py
