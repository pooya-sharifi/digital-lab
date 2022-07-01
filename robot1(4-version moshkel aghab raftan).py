#cj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401
import time
# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
        kp=7
        ki=10
        kd=10
        sum_Error=0
        Error_back=0
        sum_error_distance=0
        error_distance_back=0
        in_ball_direction = False
#         soale 3
#         y_flag = False
#         x_flag = False
#         angle_flag =False
#         soale3
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841
                t1 = time.time()
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841

                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841

#                 self.left_motor.setVelocity(0)
#                 self.right_motor.setVelocity(0)
                direction = utils.get_direction(ball_data["direction"])

                # If the robot has the ball right in front of it, go forward,
                # rotate otherwise
                
                
#                 x_angle_direction =1-ball_data['direction'][0]
#                 y_angle_direction=ball_data['direction'][1]
#                 angle_direction=math.atan2(y_angle_direction,x_angle_direction)
               
#                 phi_des=math.degrees(angle_direction)
#                 phi_des=math.degrees(x_angle_direction)
#                 phi_des=(x_angle_direction)
#                 heading_deg= math.degrees(heading)
                phi_des=direction
                Error= phi_des
                print(direction)
#                 Error =math.atan2(math.sin(Error*math.pi/180),math.cos(Error*math.pi/180))
#                 print(math.degrees(Error))
#                 sum_Error += Error * 0.01
#                 Error_d=( Error - Error_back)/0.01
#                 Error_back = Error
#                 if math.fabs(Error) < 0.07:
                if direction == 0:
                    in_ball_direction = True
                
                print("Error")
                print(Error)   
                if in_ball_direction !=True:
#                     v =0
#                     w = kp*Error + ki* sum_Error + kd*Error_d
#                 
#                     R=0.02
#                     L=0.08
#                 
#                     vr= (2*v - L*w)/(2*R)
#                     vl= (2*v + L*w)/(2*R)
#                     self.left_motor.setVelocity(vl)
#                     self.right_motor.setVelocity(vr)
                    
#                     if Error >0.5:
#                         print("shooroo be gardesh")
#                         sum_Error += Error * 0.1
#                         Error_d=( Error - Error_back)/0.1
#                         Error_back = Error
#                         velocity = kp * Error + ki*sum_Error+kd*Error_d
#                         
#                         self.left_motor.setVelocity(velocity)
#                         self.right_motor.setVelocity(0)
#                     if Error<0.5:
                    print("shooroo be gardesh")
                    sum_Error += Error * 0.1
                    Error_d=( Error - Error_back)/0.1
                    Error_back = Error
                    velocity = kp * Error + ki*sum_Error+kd*Error_d
                        
                    self.left_motor.setVelocity(velocity)
                    self.right_motor.setVelocity(0)
                else:
                    
                    error_distance = 220- ball_data['strength']
                    sum_error_distance += error_distance * 0.1
                    error_distance_d=( error_distance - error_distance_back)/0.1
                    kp_1=0.03
                    ki_1=0
                    kd_1=0
                    velocity = kp_1 * error_distance+ ki_1*sum_error_distance +kd_1*error_distance_d
                    print("velocity",velocity)
                    error_distance_back = error_distance
                    self.left_motor.setVelocity(velocity)
                    self.right_motor.setVelocity(velocity)
                    if (velocity<1):
                        heading_deg= math.degrees(heading)
                        y_co=0
                        x_co=-0.75
                        dot_deg=math.degrees(math.atan2((y_co-robot_pos[0]),-(x_co-robot_pos[1])))
                        Error=dot_deg-heading_deg
                        sum_Error += Error * 0.1
                        Error_d=( Error - Error_back)/0.1
                        Error_back = Error
                
                        v =0
                        w = kp*Error + ki* sum_Error + kd*Error_d
                
                        R=0.02
                        L=0.08
                
                        vr= (2*v - L*w)/(2*R)
                        vl= (2*v + L*w)/(2*R)
##                print(vr,vl)
                        self.left_motor.setVelocity(vl)
                        self.right_motor.setVelocity(vr)
                        if(Error<5):
                            self.left_motor.setVelocity(8)
                            self.right_motor.setVelocity(8)
                        
                    if math.fabs(direction != 0):
                        in_ball_direction=False
                    
                ####soale 2
#                 x_desired=-0.4
#                 error = x_desired-robot_pos[1]
#                 
#                 
#                 kp = 10
#                 ki=0.1
#                 sum_Error += error * 0.1
#                 w=-kp*error - ki* sum_Error
#                 print("$$$$")
#                 print(robot_pos[1])
#                 print(error)
#                 print(sum_Error)
#                 print(w)
#                 velocity=w
#                 self.left_motor.setVelocity(velocity)
#                 self.right_motor.setVelocity(velocity)

                ##soale 3
#                 error_y = -0.2 - robot_pos[1]
#                 error_x = -0.6 - robot_pos[0]
#                 print(error_y)
#                 print(error_x)
#                 
#                 pi2_turn_angle = 3.14 / 2 - heading
# 
#                 if math.fabs(error_y) < 0.01:
#                     y_flag = True
#                 
#                 if math.fabs(error_x) < 0.01:
#                     x_flag = True
# 
#                 if math.fabs(pi2_turn_angle) < 0.001:
#                     angle_flag = True
# 
#                 if y_flag==False:
#                     k = 20
#                     velocity = k * error_y
#                     self.set_both_velocity(-velocity)
#                 elif angle_flag==False:
#                     k = 20
#                     velocity = k * pi2_turn_angle
#                     if velocity > 10:
#                         self.left_motor.setVelocity(10)
#                     else:
#                         self.left_motor.setVelocity(velocity)
#                 elif x_flag==False:
#                     k = 10
#                     velocity = k * error_x
#                     self.set_both_velocity(-velocity)

    
                
                t2=time.time()
                if t2 - t1 < 0.0001 :
                    time.sleep(0.0001-(t2-t1))
                    
#     def set_both_velocity(self, velocity):
#         self.left_motor.setVelocity(velocity)
#         self.right_motor.setVelocity(velocity)
                    
        '''# Compute the speed for motors
                direction = utils.get_direction(ball_data["direction"])

                # If the robot has the ball right in front of it, go forward,
                # rotate otherwise
                if direction == 0:
                    left_speed = 7
                    right_speed = 7
                else:
                    left_speed = direction * 4
                    right_speed = direction * -4

                # Set the speed to motors
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)

                # Send message to team robots
                self.send_data_to_team(self.player_id)'''
