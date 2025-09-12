# Summary
The following files are used to simulate Autonomous Vehicle (AV) decision algorithms at intersections with staggered stop lines. The simulator uses the Australian driving convention - i.e. cars are driving on the left-hand side of the road.  
Cross-traffic is generated as green vehicles that change to yellow or red to demonstrate how hard they are braking. The AV is represented as the blue rectangle. The AV's field of view is displayed using a transparent triangle projecting from the forward-facing direction. Cross-traffic that is within the AV's field of view is shown as green rectangles with red circles overlaid on top.  
Multiple objects can be toggled on and off that will obstruct the AV's field of view.  
The AV also has a behaviour that can be toggled on where it will improve it's field of view by 'inching' forward.

# Running the sim
The simulator can be run by simply running:  
```
python run_sim.py
```  
Alternatively, the executable file can be used to run the simulator on Windows without the need for a python environment.  

# Controls
Arrow keys change the direction the AV is trying to drive/navigate the intersection (i.e. straight through, left turn or right turn).  
'Space' triggers the AV to try to navigate the intersection.  
'B' toggles a blocker vehicle, which is stopped ahead of the AV at the staggered stopline.  
'P' toggles a parked vehicle, which is parked on the shoulder of the road to the left.  
'O' toggles a blocking object, which is positioned at the near corner of the intersection to the right.  
'I' toggles the AV's 'inching' behaviour for improving field of view.  
'R' resets the simulator.  
'Q' quits the simulator.  

# Parameters
There are a handful of other parameters which can be changed by pressing 'S'.  
(1) Traffic flow, measured in vehicles/hour.  
(2) AV speed, measured in kilometres/hour.  
(3) Cross-traffic speed, measured in kilometres/hour.  

# Layout
This simulator uses the following layout:  

![Layout 1 img](assets/Layout-1.png)
