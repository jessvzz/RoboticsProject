Place the contents of this repo directly inside the /src folder, within your ros2 workspace directory.
To clone the the repo, run:  
```bash
git clone https://github.com/jessvzz/RoboticsProject your_ros2_ws/src
```


The workspace for now just contains an example package named 'hello_world_package', where
we can see a simple publish / subscribe architecture working in python.

To run the example, navigate to the ros2 workspace folder (cd ..) and run 'colcon build'.
After that, run 'source install/setup.bash' to make the entrypoints definded in the setup.py (inside the hello_world_package folder) file available in the terminal.

Now we can open two terminals, and run:  
Terminal 1: ```ros2 run hello_world_package publisher```  
Terminal 2: ```ros2 run hello_world_package subscriber```

> **NOTE**: If ros2 is not found as a bash command, make sure you run:  
`source /opt/ros/jazzy/setup.bash`

We should now see the two nodes starting to exchange messages.

Moving on we should create our own packages for every part of the robotic software