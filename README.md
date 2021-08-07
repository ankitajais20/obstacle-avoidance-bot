![Wire connections on the breadboard](https://user-images.githubusercontent.com/88271490/128607489-3c05032b-423f-4ae5-a537-b17f09abd384.jpg)
![circuit diagram](https://user-images.githubusercontent.com/88271490/128607491-14abc764-c7a0-4651-a813-a1abeb8478b1.jpg)
![Obstacle Avoidance Robot](https://user-images.githubusercontent.com/88271490/128607492-d3522d36-2e7a-453a-9d04-82c24c03b429.jpg)
# obstacle-avoidance-bot
 This is an Obstacle Avoiding Robot using Reinforcement Learning/AI.
 Algorithm used in this project: Q learning 
 
 How Obstacle Avoiding Works? 
 1.Ultrasonic sensor is used measure distance from the obstacle using its Echo and Trig Pins. 
 2.When distance is measured and if its less than 20 cm then there is an obstacle nearby otherwise robot is safe and continue Forward. 
 3.If obstacle is detected  then robot takes left or right turn depending  on the situation.  
 
 How AI based Obstacle Avoidance Works?(Q learning) 
 1.Here the 1st step from upper article remains the same.However the 2nd Step is different. 
 2.A list of actions of the robot are initialized first. For Example  in this case actions of Robot are: Left , Forward, Backward ,Stop. 
 3.When the Robot comes near an obstacle it is required to perform an action.However note that in this case Robot doesn't know which action to take as its not pre-programmed and going to learn on its own to avoid obstacles. 
 4.When Robot stops when there is an obstacle in front of it then it gets reward as 0. When Robot stops and goes backward  it receives reward of -5. When Robot continues to move forward ignoring the obstacles it receives reward of -10. When Robot just moves left as soon as obstacle is detected it gets reward of +10.
 5. In this way Robot learns on its own to avoid obstacles by Reward Mechanism.
