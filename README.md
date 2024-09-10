This is a repo of a couple scripts to run an automatic camera tracker using a RPI4B, 2 dc motors, and some other componenets. 
What's special about this is that the YOLO algorithm, used to process objection detection, is split onto a more powerful computer.
The reason being is the RPI4 cannot handle the YOLO algorithm itself, so we split the workload and send the movement commands to the RPI4.
