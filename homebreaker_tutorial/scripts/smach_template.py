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
            # Userdata is used to share data between states
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

        userdata.data_1 = None
        if True:
            # Transition to state related to outcome 1
            return 'outcome1'  
        else:
            # Transition to state related to outcome 2
            return 'outcome2'


### SMACH Building
def getInstance():

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['your_out_comes'])

    # Open the container
    with sm:
        # Add states to the container
        pass
        
if __name__ == '__main__':
    rospy.init_node('smach_example')
    sm = getInstance()
    sis = IntrospectionServer('smach_example', sm, '/SMACH_EXAMPLE') #Smach Viewer
    sis.start()
    result = sm.execute()
    sis.stop()