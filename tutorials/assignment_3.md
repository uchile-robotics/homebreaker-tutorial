# Tutorial 3: SMACH (States MACHines)

# Introduction to ROS State Machines (ROS Smach)

## Objective
The objective of this tutorial is to introduce you to ROS State Machines (ROS Smach) and demonstrate how they can be used to create complex robot behaviors in a modular and maintainable way. You are going to be using things from previous tutorials so make sure you have completed them successfully.

## Goals
- Understand the concept of state machines and their relevance in robotics.
- Learn how to use ROS Smach to design and implement state machines in ROS.
- Create a simple example of a ROS Smach state machine to control robot behavior.

## Background
State machines are a fundamental concept in robotics and computer science, providing a way to model complex systems as a set of states and transitions between them. In ROS, the Smach (State Machine) package provides a powerful framework for creating and managing state machines for robot behavior.

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
            outcomes=['outcome1', 'outcome2']
            # Input Keys: Userdata attributes to take as input
            input_keys=["data_1"]
            # Output Keys: Userdata attributes to give as output
            output_keys=["data_1"]
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