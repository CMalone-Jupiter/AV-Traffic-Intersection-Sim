# Summary
The following files are used to simulate AV decision algorithms at intersections with staggered stop lines. The simulator uses the Australian driving convention - i.e. cars are driving on the left-hand side of the road. Cross-traffic is generated as green vehicles that change to yellow or red to demonstrate how hard they are braking. The AV's field of view is displayed using a transparent triangle projecting out from the front of the blue rectangle representing the AV. Cross-traffic that is within the AV's field of view is shown as green rectangles with red circles overlaid on top.

# Running the sim
The simulator can be run by simply running:  
```
python run_sim.py
```  

# Controls
Arrow keys change the direction the AV is trying to drive/navigate the intersection (i.e. straight through, left turn or right turn).  
'Space' triggers the AV to try to navigate the intersection.  
'B' toggles a blocker vehicle, which is parked ahead of the AV at the staggered stopline.  
'R' resets the simulator.  

# Layout
This simulator uses the following layout:  

![Layout 1 img](assets/Layout-1.png)
