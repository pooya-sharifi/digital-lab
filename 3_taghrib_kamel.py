#cj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401
import time
# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
        kp=10
        ki=10
        kd=0
        sum_Error=0
        Error_back=0
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

                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
            
##                x_angle_direction =ball_data['direction'][0]
##                y_angle_direction=ball_data['direction'][1]
##                angle_direction=math.atan2(y_angle_direction,x_angle_direction)
                
                #print(f"x_angle_direction{x_angle_direction}")
              #  print(f"y_angle_direction{y_angle_direction}")
                
                #print(f"angle baad atan2 {math.degrees(angle_direction)}")
##                phi_des=math.degrees(angle_direction)
                heading_deg= math.degrees(heading) 
                #print(f"phi_des is :{phi_des}")
               # print(f"heading_deg is :{heading_deg}")
               # print("bababooyi")
##                Error= phi_des 
##                Error =math.atan2(math.sin(Error*math.pi/180),math.cos(Error*math.pi/180))
                y_co=0
                x_co=-0.75
##                print(f"y_pos is:{robot_pos[0]}")
##                print(f"x_pos is:{robot_pos[1]}")
                dot_deg=math.degrees(math.atan2((y_co-robot_pos[0]),-(x_co-robot_pos[1])))
                
##                print(f"dot_deg is:{dot_deg}")
##                print(f"head_deg is:{heading_deg}")
                Error=dot_deg-heading_deg
##                print(f"Erroe is :{math.fabs(Error)}")
                sum_Error += Error * 0.1
##                print(Error)
                
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
##                print(f"Error abs is:{math.fabs(Error)}")
                if(math.fabs(Error)< 6.0):
                    self.left_motor.setVelocity(0.1)
                    self.right_motor.setVelocity(0.1)
                    kp_d=10
                    ki_d=0
                    kd_d=0
                    Errordis=math.sqrt(((y_co-robot_pos[0])**2)+(((x_co-robot_pos[1]))**2))
                    sum_Errordis=0
                    Errordis_back=0
                    sum_Errordis += Errordis * 0.1

                
                    Errordis_d=( Errordis - Errordis_back)/0.1
                    Errordis_back = Errordis
                    
                    w=0
                    v = kp_d*Errordis + ki_d* sum_Errordis + kd_d*Errordis_d
                    vr= (2*v - L*w)/(2*R)
                    vl= (2*v + L*w)/(2*R)
                    
                    self.left_motor.setVelocity(vl)
                    self.right_motor.setVelocity(vr)
                    if (Errordis<0.01):
                        self.left_motor.setVelocity(0)
                        self.right_motor.setVelocity(0)
                
                    
                
                t2=time.time()
                if t2 - t1 < 0.0001 :
                    time.sleep(0.0001-(t2-t1))
                
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



