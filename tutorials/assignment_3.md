# Tutorial 3: SMACH (States MACHines)

# Introduction to ROS State Machines (ROS SMACH)

## Objective
The objective of this tutorial is to introduce you to ROS State Machines (ROS Smach) and demonstrate how they can be used to create complex robot behaviors in a modular and maintainable way. Ensure you have successfully completed previous tutorials as you'll be utilizing concepts introduced in them.


> ## Goals
>- Understand the concept of state machines and their relevance in robotics.
>- Learn how to use ROS Smach to design and implement state machines in ROS.
>- Create a simple example of a ROS Smach state machine to control robot behavior.

## Background
State machines are a fundamental concept in robotics and computer science, providing a way to model complex systems with a set of states and transitions between them. In ROS, the Smach (State Machine) package provides a powerful framework for creating and managing state machines for robot behavior.



## Example: Simple ROS Smach State Machine
```python

### Import ROS SMACH Dependencies
import rospy
import smach
import smach_ros
from smach_ros import IntrospectionServer

### Robot Building (Import Robot Skills)
... 


### States
class ExampleState(smach.State):
    ### Class Initializer
    def __init__(self):
        smach.State.__init__(self,
            # Outcomes: define the state transition
            outcomes=['outcome1', 'outcome2'],
            # Input Keys: Userdata attributes to take as input
            input_keys=["data_1"],
            # Output Keys: Userdata attributes to give as output
            output_keys=["data_1"],
        )

    ### State Action
    def execute(self, userdata):
        rospy.loginfo('Executing Example State')
        in_data = userdata.data_1

        # Your state logic here
        ...

        userdata.data_1 = out_data
        if condition:
            # Transition to state related to outcome 1
            return 'outcome1'  
        else:
            # Transition to state related to outcome 2
            return 'outcome2'



### SMACH Building
def getInstance():

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome3'])

    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add(
        # State ID
        'STATE1', 

        # State Class
        ExampleState(), 

        # Define State Transitios
        ## Current State outcome -> Next State ID
        transitions= {
            'outcome1':'STATE2', 'outcome2':'outcome3'
            },

        # Remap userdata attributes 
        ## Current state output -> Next state input
        remapping= {
            "data_1":"data_1"
            }
        )

        # I.E a standard state definition should look like this:
        smach.StateMachine.add('STATE2', ExampleState(), transitions={'outcome1':'STATE1', 'outcome2':'outcome3'},remapping={"data_1":"data_1"})

if __name__ == '__main__':
    rospy.init_node('smach_example')
    sm = getInstance()
    sis = IntrospectionServer('smach_example', sm, '/SMACH_EXAMPLE') #Smach Viewer
    sis.start()
    result = sm.execute()
    sis.stop()
```
---
## Instructions
Now that you have an idea of what SMACHs are, let's begin your challenge:
---
>Based on what you've learned, you'll need to devise a SMACH that allows your robot to: 
>- Check if your robot is in the "Home" position *(define any position measured in assignment 1 as your 'Home' position)*.  If not, go to the "Home" position.
>- Walk randomly to any other meassured position.
>- Upon reaching each position, prompt the user for input to determine whether the robot should continue walking or return to "Home".
>- If the user chooses to return to "Home", navigate to "Home" and exit the SMACH upon reaching it.
--- 

1. **Understanding the Code**: Before proceeding, make sure you understand the different sections of the code example and how to use the different classes and methods thet were presented.

2. **Designing the SMACH**:
   - Think about what you need your robot to do and define each state you think will be useful (i.e "go to point A", "parked", etc).
   - For each state, define which conditions will need to be met to trigger transition to other states.
   
   *You should end up with something like this:*

    ![SMACH](https://miro.medium.com/v2/resize:fit:974/1*KLIzD5j_gTrr4RD6pkkrdw.png)

3. **Code your SMACH**:
   - Navigates to the "scripts" folder in the "homebreakers_tutorials" package where youu'll find a smach template.
   - Using the coding practices shown in the example above, code the SMACH that you designed on the previous step 
   - Verify that the robot will transition to the state you want by testing your logic in an isolated way (without adding any skills or commands, just user input and your SMACH).

4. **Test it!**
    - Test your code on simulation by running `rosrun homebreaker_tutorial smach_template.py`
    - If it works, go try it on the real one!

## Summary
In this tutorial, you learned about ROS State Machines (ROS SMACH) and their significance in robotics for creating modular and maintainable robot behaviors. You gained an understanding of how state machines model complex systems with states and transitions. Through the provided example of a simple ROS SMACH state machine, you grasped the implementation of state logic and transitions. Furthermore, you were tasked with designing a SMACH to enable your robot to perform specific tasks, such as navigating to designated positions and responding to user input. By completing this tutorial, you have acquired practical knowledge in using ROS SMACH to orchestrate robot behaviors effectively.