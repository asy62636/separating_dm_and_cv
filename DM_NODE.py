import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid

class DmAlgo():
    def __init__(self):
        local_goal_topic = "/move_base_simple/goal_cv"
        self.final_grid_topic = "/cv/laneoccgrid"
        self.warped_img_topic = "/cv/warpedimg"
        '''
        rospy.Subscriber(final_grid_topic, Image, self.lane_occ_grid_reciever)
        rospy.Subscriber(warped_img_topic, Image, self.warped_img_reciever )
        rospy.Subscriber(/cv/x0, Int32MultiArray, self.warped_img_reciever )
        rospy.Subscriber(/cv/y0, Int32MultiArray, self.warped_img_reciever )
        rospy.Subscriber(/cv/x1, Int32MultiArray, self.warped_img_reciever )
        rospy.Subscriber(/cv/y1, Int32MultiArray, self.warped_img_reciever )
        rospy.Subscriber(/cv/x2, Int32MultiArray, self.warped_img_reciever )
        rospy.Subscriber(/cv/y2, Int32MultiArray, self.warped_img_reciever )
        '''
        #rospy.Subscriber(warped_img_topic, Int32MultiArray, self.warped_img_reciever )
        rospy.Subscriber('/cv/xylist',Int32MultiArray, )
        self.lane_occ_grid = None
        self.warped_img = None
        self.xy_list = []
        self.local_goal = PoseStamped()
        self.local_goal.header.frame_id = "occgrid"

        self.Final_Grid = OccupancyGrid()
        self.Final_Grid.header.frame_id = "occgrid"
        self.Final_Grid.info.resolution = 0.01 #1cm per pixel
        self.Final_Grid.info.width = 1280
        self.Final_Grid.info.height = 720

        self.Final_Grid.info.origin.position.x = 0
        self.Final_Grid.info.origin.position.y = 0
        self.Final_Grid.info.origin.position.z = 0

        self.Final_Grid.info.origin.orientation.x = 0
        self.Final_Grid.info.origin.orientation.y = 0
        self.Final_Grid.info.origin.orientation.z = -0.707
        self.Final_Grid.info.origin.orientation.w = 0.707

        self.local_goal_publisher = rospy.Publisher(local_goal_topic, PoseStamped , queue_size =1)
        self.rate = rospy.Rate(10)

    def lane_occ_grid_reciever(self, img_msg):
        try:
            self.lane_occ_grid = img_msg
        except:
            pass
    def warped_img_reciever(self, img_msg):
        try:
            self.warped_img = img_msg
        except :
            pass
    
    def xy_reciever(self, xy_arr):
        try:
            self.xy_list.append[xy_arr]
            if (len(self.xy_list) == 6):
                self.run_algo()
            else:
                pass
        except:
            pass
        
    def markLocalGoal(x_array,y_array, warped):
        coefficients = np.polyfit(y_array, x_array, 7)
        # print(coefficients)

        poly = np.poly1d(coefficients)
        new_y = np.linspace(y_array[0], y_array[-1])
        new_x = poly(new_y)

        # fig, ax = plt.subplots()
        # ax.imshow(warped)

        # plt.savefig("line.jpg")

        dist = -150 #-130   # tunable parameter!  
        poly_derivative = np.polyder(poly)
        new_y_offset = new_y - ( dist * np.sin(np.arctan(poly_derivative(new_y))) )
        new_x_offset = new_x + ( dist * np.cos(np.arctan(poly_derivative(new_y))) )


        # ax.plot(new_x_offset, new_y_offset, '--', linewidth=1, color='green')
        # plt.savefig("final.jpg")

        localgoal_ind = np.argmin(np.absolute(new_y_offset))

        # print("new_x_offset", new_x_offset)
        # print("new_y_offset", new_y_offset)
        # print(int(new_y_offset[localgoal_ind]),int(new_x_offset[localgoal_ind]))

        image21 = cv2.circle(warped,(int(new_x_offset[localgoal_ind]),int(new_y_offset[localgoal_ind])),1,(0,255,255),15)
        cv2.imshow("TEJU2",image21)

        return int(new_x_offset[localgoal_ind]), int(new_y_offset[localgoal_ind])

    def publishLocalGoal(self, x , y):
        self.local_goal.header.stamp = rospy.Time.now()
        self.local_goal.pose.position.x = (720 - y) * self.Final_Grid.info.resolution #- x * self.Final_Grid.info.resolution
        self.local_goal.pose.position.y = - x * self.Final_Grid.info.resolution #(720 - y) * self.Final_Grid.info.resolution
        self.local_goal.pose.position.z = 0
        self.local_goal.pose.orientation.x = 0
        self.local_goal.pose.orientation.y = 0
        self.local_goal.pose.orientation.z = 0
        self.local_goal.pose.orientation.w = 1

        self.local_goal_publisher.publish(self.local_goal)
    
    def run_algo(self, xylist):
        rospy.Subscriber(self.final_grid_topic, Image, self.lane_occ_grid_reciever)
        rospy.Subscriber(self.warped_img_topic, Image, self.warped_img_reciever )
        localgoal_x, localgoal_y = self.markLocalGoal(xylist[2],xylist[3],self.warped_img)
        self.publishLocalGoal(localgoal_x, localgoal_y)
        self.xy_list.clear()
        cv2.waitKey(1)
        self.rate.sleep()

if __name__ == '__main__':
    print("starting the node")
    try:
        print("initializing node")
        rospy.init_node("laneDetection_and_laneOccGrid_pubNode")
        print("creating obejct")
        obj = DmAlgo()
        print("entering while loop")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")