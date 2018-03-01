###Modified Skybot because I'm unoriginal and suck at coding still
import math
import time
from quicktracer import trace

class Agent:
	def __init__(self, name, team, index):
		self.index = index

		# Controller inputs
		self.throttle = 0
		self.steer = 0
		self.pitch = 0
		self.yaw = 0
		self.roll = 0
		self.boost = False
		self.jump = False
		self.powerslide = False

		# Game values
		self.bot_loc_x = None
		self.bot_loc_y = None
		self.bot_loc_z = None
		self.bot_speed_x = None
		self.bot_speed_y = None
		self.bot_speed_z = None
		self.bot_rot_yaw = None
		self.bot_rot_roll = None
		self.bot_rot_pitch = None
		self.bot_jumped = False
		self.bot_doublejumped = False
		self.bot_ground = False
		self.bot_sonic = False
		self.bot_dodge = False
		self.bot_boost = None
		self.ball_loc_x = None
		self.ball_loc_y = None
		self.ball_loc_z = None
		self.ball_speed_x = None
		self.ball_speed_y = None
		self.ball_speed_z = None
		self.ball_acc_x = None
		self.ball_lt_x = None
		self.ball_lt_y = None
		self.ball_lt_z = None

		#game values converted
		self.bot_yaw = None
		self.bot_pitch = None
		self.bot_roll = None
		self.angle_front_to_target = None
		self.angle_car_ball = None

		#custom values
		self.angle_car_ball = None
		self.distance_car_ball = None
		self.bot_speed_linear = None

		self.after_dodge = False
		self.next_dodge_time = None

		self.ball_lt_x_prev = None
		self.ball_lt_y_prev = None

		self.counttime= None
		self.ttt=None

	def distance(self, x1, y1, z1, x2, y2, z2):
		distance=math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
		return distance

                goal_pos_blue = [0, -5000]
                goal_pos_oran = [0, 5000]
                ball_pos = self.ball_loc

                goal_to_ball_blue = [goal_pos_blue[0] - ball_pos.X, goal_pos[1] - ball_pos.Y]
                goal_to_ball_oran = [goal_pos_oran[0] - ball_pos.X, goal_pos[1] - ball_pos.Y]
                
                magnitude_blue = math.sqrt(goal_to_ball_blue[0] ** 2 + goal_to_ball_blue[1] ** 2)
                magnitude_oran = math.sqrt(goal_to_ball_oran[0] ** 2 + goal_to_ball_oran[1] ** 2)
                
                normalised_blue = [goal_to_ball_blue[0] / magnitude_blue, goal_to_ball_blue[1] / magnitude_blue]
                normalised_oran = [goal_to_ball_oran[0] / magnitude_oran, goal_to_ball_oran[1] / magnitude_oran]

                final_pos_blue = [ball_pos.X + normalised_blue * 100, ball_pos.Y + normalised_blue * 100]
                final_pos_oran = [ball_pos.X + normalised_oran * 100, ball_pos.Y + normalised_oran * 100]
        
        # math.atan( (4800 - BallY) / (0 - BallX))
        
	def dodge(self):
		if self.bot_doublejumped:
			self.jump = False
		elif not self.bot_jumped:
			self.jump = True
			self.pitch = -1
			self.next_dodge_time = time.time() + 0.15
		elif time.time() > self.next_dodge_time:
			self.jump = True
			if 10 < self.angle_front_to_target < 45:
				self.yaw = 1
				self.pitch = -1
			elif -10 > self.angle_front_to_target > -45:
				self.yaw = -1
				self.pitch = -1
			elif 46 < self.angle_front_to_target < 90:
				self.yaw = 1
				self.pitch = 0
			elif -46 > self.angle_front_to_target > -90:
				self.yaw = -1
				self.pitch = 0
			else:
				self.yaw = 0
				self.pitch = -1
			self.after_dodge = True


	def aim(self, target_x, target_y):
		self.throttle=1
		angle_between_bot_and_target = math.degrees(math.atan2(target_y - self.bot_loc.Y, target_x - self.bot_loc.X))
		self.angle_front_to_target = angle_between_bot_and_target - self.bot_rot_yaw
		# Correct the values
		if self.angle_front_to_target < -180:
			self.angle_front_to_target += 360
		if self.angle_front_to_target > 180:
			self.angle_front_to_target -= 360

		if self.angle_front_to_target < -10:
			# If the target is more than 10 degrees right from the centre, steer left
			self.steer = -1
		elif self.angle_front_to_target > 10:
			# If the target is more than 10 degrees left from the centre, steer right
			self.steer = 1
		else:
			# If the target is less than 10 degrees from the centre, steer straight
			self.steer = self.angle_front_to_target/10

		if self.angle_front_to_target < 15 or self.angle_front_to_target > -15:
			self.boost=True
		else: self.boost=False
		if self.angle_front_to_target >80 or self.angle_front_to_target <-80:
			self.powerslide = 1
		else: self.powerslide = 0

	def get_output_vector(self, values):
		self.boost = False
		self.jump=False

		# Update game data variables
		self.bot_loc= values.gamecars[self.index].Location
		self.bot_rot= values.gamecars[self.index].Rotation
		self.ball_loc= values.gameball.Location

		#get values
		self.bot_loc_x = values.gamecars[self.index].Location.X
		self.bot_loc_y = values.gamecars[self.index].Location.Y
		self.bot_loc_z = values.gamecars[self.index].Location.Z
		self.bot_speed_x = values.gamecars[self.index].Velocity.X
		self.bot_speed_y = values.gamecars[self.index].Velocity.Y
		self.bot_speed_z = values.gamecars[self.index].Velocity.Z
		self.bot_jumped = values.gamecars[self.index].bJumped
		self.bot_doublejumped = values.gamecars[self.index].bDoubleJumped
		self.bot_sonic = values.gamecars[self.index].bSuperSonic
		self.bot_ground = values.gamecars[self.index].bOnGround
		self.bot_boost = values.gamecars[self.index].Boost
		self.ball_loc_x = values.gameball.Location.X
		self.ball_loc_y = values.gameball.Location.Y
		self.ball_loc_z = values.gameball.Location.Z
		self.ball_speed_x = values.gameball.Velocity.X
		self.ball_speed_y = values.gameball.Velocity.Y
		self.ball_speed_z = values.gameball.Velocity.Z
		self.ball_acc_x = values.gameball.Acceleration.Z
		self.ball_lt_x = values.gameball.LatestTouch.sHitLocation.X
		self.ball_lt_y = values.gameball.LatestTouch.sHitLocation.Y
		self.ball_lt_z = values.gameball.LatestTouch.sHitLocation.Z


		# Get car's yaw, pitch and roll and convert from Unreal Rotator units to degrees
		self.bot_rot_yaw = abs(self.bot_rot.Yaw) % 65536 / 65536 * 360
		if self.bot_rot.Yaw < 0:
			self.bot_rot_yaw *= -1
		self.bot_rot_pitch = abs(self.bot_rot.Pitch) % 65536 / 65536 * 360
		if self.bot_rot.Pitch < 0:
			self.bot_rot_pitch *= -1
		self.bot_rot_roll = abs(self.bot_rot.Roll) % 65536 / 65536 * 360
		if self.bot_rot.Roll < 0:
			self.bot_rot_roll *= -1

		#get values
		self.angle_car_ball = math.degrees(math.atan2(self.ball_loc_y - self.bot_loc.Y, self.ball_loc_x - self.bot_loc.X)) - self.bot_rot_yaw
		self.distance_car_ball = self.distance(self.bot_loc_x, self.bot_loc_y, self.bot_loc_z, self.ball_loc_x, self.ball_loc_y, self.ball_loc_z)-93
		self.bot_speed_linear = self.distance(self.bot_speed_x, self.bot_speed_y, self.bot_speed_z, 0, 0, 0)


		# Blue has their goal at -5000 (Y axis) and orange has their goal at 5000 (Y axis). This means that:
		# - Blue is behind the ball if the ball's Y axis is greater than blue's Y axis
		# - Orange is behind the ball if the ball's Y axis is smaller than orange's Y axis

#normal aim
		if (self.index == 0 and self.bot_loc_y < self.ball_loc_y-200) or (self.index == 1 and self.bot_loc_y > self.ball_loc_y+200):
			if self.index == 0:
				self.aim(self.ball_loc_x, self.ball_loc_y-100)
			if self.index == 1:
				self.aim(self.ball_loc_x, self.ball_loc_y+100)
		else:
			if self.index == 0:
				# Blue team's goal is located at (0, -5000)
				self.aim(0, -4800)
			else:
				# Orange team's goal is located at (0, 5000)
				self.aim(0, 4800)

		if self.index == 0 and self.bot_loc_y >5000:
			self.aim(0,5000)
		if self.index == 1 and self.bot_loc_y <5000:
			self.aim(0,-5000)

		#reset if ball is too high
		if self.ball_loc_z > 400 :
			if self.index == 0 and self.ball_loc_y > 0:
				self.aim(self.ball_loc_x,-4900)
			if self.index == 1 and self.ball_loc_y < 0:
				self.aim(self.ball_loc_x,4900)
		#print(self.ball_loc_z)

		#x is latitude
		#y is longitude
		# #reset if ball is in corner
		# if self.index==0 and self.ball_loc_y > 4000 and self.ball_loc_x<-900:
		# 	self.aim(-3500,0)
		# elif self.index==0 and self.ball_loc_y > 4000 and self.ball_loc_x>900:
		# 	self.aim(3500,0)
		# elif self.index==1 and self.ball_loc_y < -4000 and self.ball_loc_x<-900:
		# 	self.aim(3500,0)
		# elif self.index==1 and self.ball_loc_y < -4000 and self.ball_loc_x>900:
		# 	self.aim(-3500,0)
		# # print(self.ball_loc_y)



		#perfect speed
		if self.ball_speed_z < 0 :
			time_ball_ground=(self.ball_loc_z-92.12849426269531)/-self.ball_speed_z
			if time_ball_ground !=0:
				perfect_speed = self.distance_car_ball/time_ball_ground
			#print(perfect_speed/self.bot_speed_linear)
			if perfect_speed < self.bot_speed_linear:
				self.throttle=-1
			elif perfect_speed/self.bot_speed_linear>2:
				self.boost=True
			else: self.boost=False
		# if self.distance(self.bot_loc_x, self.bot_loc_y, 0, self.ball_loc_x, self.ball_loc_y, 0)<self.ball_loc_z:
		# 	self.throttle=0
		#print(self.distance_car_ball/self.distance(self.bot_speed_x,self.bot_speed_y,self.bot_speed_z,self.ball_speed_x,self.ball_speed_y,self.ball_speed_z))


		if self.bot_speed_linear < 2000:
			self.boost=True
			# self.dodge()

		# print(self.bot_speed_linear)



#dodge
		if self.throttle==1:
			if self.distance(self.bot_loc_x, self.bot_loc_y, 0, self.ball_loc_x, self.ball_loc_y, 0) < 400 and self.ball_loc_z <170 and -15<self.angle_car_ball<15:
					self.dodge()
			if self.bot_ground:
				self.pitch=0
				if self.after_dodge:
					self.pitch=0
					self.after_dodge = False
#right side up
		if not self.bot_ground:
			if self.bot_rot_roll >20:
				self.roll = -1
			elif self.bot_rot_roll <-20:
				self.roll = 1
			else: self.roll = -(self.bot_rot_roll/50)












		return [self.throttle, self.steer, self.pitch, self.yaw, self.roll, self.jump, self.boost, self.powerslide]
