#! /usr/bin/env python                                                                                 
                                                                                                       
import robot_api                                                                                       
from map_annotator import PoseRecorder
import rospy                  
                              
                       
def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass                                                                                           

def welcome():
    """print usage
    """          
    print("Welcome to the map annotator! \n \
        Commands: \n \
        list: List saved poses. \n \
        save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists. \n \
        delete <name>: Delete the pose given by <name>. \n \
        goto <name>: Sends the robot to the pose given by <name>. \n \
        help: Show this list of commands. \n \
        exit: Exit. \n")          
                                                              
                                                                                                       
def main():                                                                                            
    rospy.init_node('pose_recorder_demo')                                                               
    wait_for_time()                                                                                    
    # argv = rospy.myargv()                                                                              
    recorder = PoseRecorder()
    rospy.sleep(0.5)
    welcome()
    while(True):
        user_input = input()
        input_split = user_input.split()
        
        if input_split[0] == "list":
            poselist = recorder.ListPose()
            print("Poses:")
            for pose in poselist:
                print("\t {}".format(pose))
                           
        elif input_split[0] == "save":
            name = " ".join(input_split[1:])
            recorder.AddPose(name)
            recorder.save()
            
        elif input_split[0] == "delete":
            name = " ".join(input_split[1:])
            recorder.DelPose(name)   
            recorder.save()
            
        elif input_split[0] == "goto":
            name = " ".join(input_split[1:])
            recorder.SetGoal(name)
        
        elif input_split[0] == "exit":
            recorder.save()
            break
            
        elif input_split[0] == "help":
            welcome()
        
        else:
            welcome()
                     
if __name__ == '__main__':
    main()