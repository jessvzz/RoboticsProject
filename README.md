The repo for now just contains an example package named 'hello_world_package', where
we can see a simple publish / subscribe architecture working in python.

To run the example, navigate to the workspace folder and run 'colcon build'.
After that, run 'source install/setup.bash' to make the entrypoints definded in the setup.py (inside the package folder) file available in the terminal.

Now we can open two terminals, and run:  
Terminal 1 >> `ros2 run hello_world_package publisher`  
Terminal 2 >> `ros2 run hello_world_package subscriber`

We should now see the two nodes starting to exchange messages. 