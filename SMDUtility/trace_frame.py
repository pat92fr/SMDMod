
## http://www.science.smith.edu/dftwiki/index.php/Color_Charts_for_TKinter


from tkinter import *
#from tkinter.ttk import *
import time
from trace_frame import *
from ik import *

class trace_frame(LabelFrame):

	def __init__(self,window,protocol,id):
		super().__init__(text="TRACE")
		self.protocol = protocol
		self.id = id
		self["width"]=200
		self["height"]=1000
		self.labels = {}
		self.entries = {}
		self.variables = {}
		self.checks = {}
		self.viewports = {}

		self.test_timer = 0
		self.test_square_value = 0

		self.columnconfigure(0, weight=1)

		fa = LabelFrame(self,text="Actions")
		fa.grid(column = 0, row = 0, sticky='nesw')
		fa.columnconfigure(0, weight=1)
		self.variables["torque_enable_local"] = IntVar()
		self.variables["torque_enable_local"].set(0)		
		self.checks['torque_enable'] = Checkbutton(fa,text="Torque Enable", command=self.torque_enable,variable=self.variables["torque_enable_local"])
		self.checks['torque_enable'].grid(column = 0, row = 0, sticky='nw')

		self.variables["current"] = IntVar()
		self.variables["current"].set(200)		
		self.entries["current"] = Entry(fa, width = 5, textvariable = self.variables["current"]) 
		self.entries['current'].grid(column = 5, row = 0, sticky='nw')

		self.variables["square_current"] = IntVar()
		self.variables["square_current"].set(0)		
		self.checks['square_current'] = Checkbutton(fa,text="Square current Test", variable=self.variables["square_current"])
		self.checks['square_current'].grid(column = 4, row = 0, sticky='nw')
		
		self.variables["square_position"] = IntVar()
		self.variables["square_position"].set(0)		
		self.checks['square_position'] = Checkbutton(fa,text="Square position Test", variable=self.variables["square_position"])
		self.checks['square_position'].grid(column = 3, row = 0, sticky='nw')

		self.variables["triangle_position"] = IntVar()
		self.variables["triangle_position"].set(0)		
		self.checks['triangle_position'] = Checkbutton(fa,text="Triangle position Test", variable=self.variables["triangle_position"])
		self.checks['triangle_position'].grid(column = 2, row = 0, sticky='nw')
		
		self.variables["round_position"] = IntVar()
		self.variables["round_position"].set(0)		
		self.checks['round_position'] = Checkbutton(fa,text="Round position Test", variable=self.variables["round_position"])
		self.checks['round_position'].grid(column = 1, row = 0, sticky='nw')
		
		fp = LabelFrame(self,text="Acceleration")
		fp.grid(column = 0, row = 1, sticky='nesw')
		fp.columnconfigure(0, weight=1)
		self.viewports['acceleration'] = Canvas(fp, bg="#FFFFFF") #, width=viewport_max_x, height=viewport_size_y_pos, bg="#FFFFFF")
		self.viewports['acceleration'].grid(column = 0, row = 1, rowspan=20, sticky="we") #, sticky='wens')
		self.variables['goal_acceleration'] = IntVar()
		self.variables['goal_acceleration'].set(1)
		self.checks['goal_acceleration']= Checkbutton(fp,text="Goal Acceleration",variable=self.variables['goal_acceleration'])
		self.checks['goal_acceleration'].grid(column = 4, row = 1, sticky='w')
		self.variables['setpoint_acceleration'] = IntVar()
		self.variables['setpoint_acceleration'].set(1)
		self.checks['setpoint_acceleration']= Checkbutton(fp,text="Setpoint Acceleration",variable=self.variables['setpoint_acceleration'])
		self.checks['setpoint_acceleration'].grid(column = 4, row = 2, sticky='w')
		self.variables['present_acceleration'] = IntVar()
		self.variables['present_acceleration'].set(1)		
		self.checks['present_acceleration']= Checkbutton(fp,text="Present Acceleration",variable=self.variables['present_acceleration'])
		self.checks['present_acceleration'].grid(column = 4, row = 3, sticky='w')


		fv = LabelFrame(self,text="Velocity Control")
		fv.grid(column = 0, row = 2, sticky='nesw')
		fv.columnconfigure(0, weight=1)
		self.viewports['velocity'] = Canvas(fv, bg="#FFFFFF") #, width=viewport_max_x, height=viewport_size_y_pos, bg="#FFFFFF")
		self.viewports['velocity'].grid(column = 0, row = 1, rowspan=20, sticky="we") #, sticky='wens')
		self.variables['goal_velocity'] = IntVar()
		self.variables['goal_velocity'].set(0)
		self.checks['goal_velocity']= Checkbutton(fv,text="Goal Velocity",variable=self.variables['goal_velocity'])
		self.checks['goal_velocity'].grid(column = 4, row = 1, sticky='w')
		self.variables['setpoint_velocity'] = IntVar()
		self.variables['setpoint_velocity'].set(1)
		self.checks['setpoint_velocity']= Checkbutton(fv,text="Setpoint Velocity",variable=self.variables['setpoint_velocity'])
		self.checks['setpoint_velocity'].grid(column = 4, row = 2, sticky='w')
		self.variables['present_velocity'] = IntVar()
		self.variables['present_velocity'].set(1)		
		self.checks['present_velocity']= Checkbutton(fv,text="Present Velocity",variable=self.variables['present_velocity'])
		self.checks['present_velocity'].grid(column = 4, row = 3, sticky='w')


		fc = LabelFrame(self,text="Current Control")
		fc.grid(column = 0, row = 3, sticky='nesw')
		fc.columnconfigure(0, weight=1)
		self.viewports['current'] = Canvas(fc, bg="#FFFFFF") #, width=viewport_max_x, height=viewport_size_y_pos, bg="#FFFFFF")
		self.viewports['current'].grid(column = 0, row = 1, rowspan=20, sticky="we") #, sticky='wens')
		self.variables['goal_current'] = IntVar()
		self.variables['goal_current'].set(0)
		self.checks['goal_current']= Checkbutton(fc,text="Goal Current",variable=self.variables['goal_current'])
		self.checks['goal_current'].grid(column = 4, row = 1, sticky='w')
		self.variables['setpoint_current'] = IntVar()
		self.variables['setpoint_current'].set(1)
		self.checks['setpoint_current']= Checkbutton(fc,text="Setpoint Current",variable=self.variables['setpoint_current'])
		self.checks['setpoint_current'].grid(column = 4, row = 2, sticky='w')
		self.variables['present_current'] = IntVar()
		self.variables['present_current'].set(1)		
		self.checks['present_current']= Checkbutton(fc,text="Present Current",variable=self.variables['present_current'])
		self.checks['present_current'].grid(column = 4, row = 3, sticky='w')

		fpwm = LabelFrame(self,text="PWM Control")
		fpwm.grid(column = 0, row = 4, sticky='nesw')
		fpwm.columnconfigure(0, weight=1)
		self.viewports['pwm'] = Canvas(fpwm, bg="#FFFFFF", height=160) #, width=viewport_max_x, height=viewport_size_y_pos, bg="#FFFFFF")
		self.viewports['pwm'].grid(column = 0, row = 1, rowspan=20, sticky="we") #, sticky='wens')
		self.variables['goal_pwm'] = IntVar()
		self.variables['goal_pwm'].set(1)
		self.checks['goal_pwm']= Checkbutton(fpwm,text="Goal PWM    ",variable=self.variables['goal_pwm'])
		self.checks['goal_pwm'].grid(column = 4, row = 1, sticky='w')
		self.variables['setpoint_pwm'] = IntVar()
		self.variables['setpoint_pwm'].set(1)
		self.checks['setpoint_pwm']= Checkbutton(fpwm,text="Setpoint PWM    ",variable=self.variables['setpoint_pwm'])
		self.checks['setpoint_pwm'].grid(column = 4, row = 2, sticky='w')

		self.viewport_current_x = 0
		self.viewport_start_time = time.time()
		self.viewport_size_y_pos = self.viewports['acceleration'].winfo_height()
		self.viewport_size_x_pos = self.viewports['acceleration'].winfo_width()
		self.viewport_size_y_vel = self.viewports['velocity'].winfo_height()
		self.viewport_size_y_cur = self.viewports['current'].winfo_height()
		self.viewport_size_y_pwm = self.viewports['pwm'].winfo_height()
		self.grid_x()

	def grid_x(self):
			self.viewports['acceleration'].create_line(0,self.viewport_size_y_pos/2.0,self.viewport_size_x_pos,self.viewport_size_y_pos/2.0,width=1,fill="#CCCCCC")
			self.viewports['velocity'].create_line(0,self.viewport_size_y_vel/2.0,self.viewport_size_x_pos,self.viewport_size_y_vel/2.0,width=1,fill="#CCCCCC")
			self.viewports['current'].create_line(0,self.viewport_size_y_cur/2.0,self.viewport_size_x_pos,self.viewport_size_y_cur/2.0,width=1,fill="#CCCCCC")
			self.viewports['pwm'].create_line(0,self.viewport_size_y_pwm/2.0,self.viewport_size_x_pos,self.viewport_size_y_pwm/2.0,width=1,fill="#CCCCCC")

	def clear(self):
		for n,c in self.viewports.items():
			c.delete("all")
			self.grid_x()
		self.viewport_current_x = 0

	def torque_enable(self):

		if self.variables["torque_enable_local"].get()==0:
			print("torque disable")

		elif self.variables["torque_enable_local"].get()==1:
			print("torque enable")

		print("write RAM...")
		error = self.protocol.write_byte_command(
			self.id.current_id, # ID
			0x40, #RAM
			[int(self.variables['torque_enable_local'].get())], # data,
			verbose=1
		)
		print("error:"+str(error))


	def test_square_current(self):
		current = self.variables["current"].get()
		if time.time()*1000.0 >= self.test_timer+2000.0:
			self.test_timer = time.time()*1000.0
			if self.test_square_value == 0:
				self.test_square_value = 1
				return -current
			elif self.test_square_value == 1:
				self.test_square_value = -1
				return 1
			elif self.test_square_value == -1:
				self.test_square_value = 1
				return -current
		else:
			return 0

	def test_square_position(self):
		amplitude = 20
		if time.time()*1000.0 >= self.test_timer+1000.0:
			self.test_timer = time.time()*1000.0
			if self.test_square_value == 0:
				self.test_square_value = 1
				return int((amplitude+90)*10)
			elif self.test_square_value == 1:
				self.test_square_value = -1
				return int((-amplitude+90)*10)
			elif self.test_square_value == -1:
				self.test_square_value = 1
				return int((amplitude+90)*10)
		else:
			return 0

	def test_triangle_position(self):
		amplitude = 30
		if time.time()*1000.0 >= self.test_timer+200.0:
			self.test_timer = time.time()*1000.0
			if self.test_square_value == 0:
				self.test_square_value = 1
				return int((amplitude+90)*10), 
			elif self.test_square_value == 1:
				self.test_square_value = -1
				return int((-amplitude+90)*10)
			elif self.test_square_value == -1:
				self.test_square_value = 1
				return int((amplitude+90)*10)
		else:
			return 0

	def test_round_position(self):
		if time.time()*1000.0 >= self.test_timer+20.0:
			self.test_timer = time.time()*1000.0
			r = 0.025;
			x0 = 0.005
			z0 = -0.095;
			foot_position = np.zeros((3,1)) # x,y,z
			foot_position[0,0] = x0+r*math.cos(self.test_timer/500*2.0*math.pi)
			foot_position[2,0] = z0+r*math.sin(self.test_timer/500*2.0*math.pi)
			servo_angles_rad = ik(foot_position)
			return True, np.degrees(servo_angles_rad)
		else: 
			foot_position = np.zeros((3,1)) # x,y,z
			return False, None


	def update(self,
		goal_acceleration,
		setpoint_acceleration,
		present_acceleration,
		goal_velocity,
		setpoint_velocity,
		present_velocity,
		goal_current,
		setpoint_current,
		present_current,
		goal_pwm,
		setpoint_pwm
		):

			self.viewport_size_y_pos = self.viewports['acceleration'].winfo_height()
			self.viewport_size_x_pos = self.viewports['acceleration'].winfo_width()
			self.viewport_size_y_vel = self.viewports['velocity'].winfo_height()
			self.viewport_size_y_cur = self.viewports['current'].winfo_height()
			self.viewport_size_y_pwm = self.viewports['pwm'].winfo_height()

			#print(str(viewport_size_y_pos)+"x"+str(viewport_size_x_pos))

			# time line trace
			if self.viewport_current_x == 0:
				self.viewport_start_time=time.time()

			if time.time() >= self.viewport_start_time+1.0:
				self.viewport_start_time += 1.0
				self.viewports['acceleration'].create_line(
					self.viewport_current_x,
					0,
					self.viewport_current_x+1,
					self.viewport_size_y_pos,
					width=2,
					fill="#DDDDDD"
				)
				self.viewports['velocity'].create_line(
					self.viewport_current_x,
					0,
					self.viewport_current_x+1,
					self.viewport_size_y_vel,
					width=2,
					fill="#DDDDDD"
				)
				self.viewports['current'].create_line(
					self.viewport_current_x,
					0,
					self.viewport_current_x+1,
					self.viewport_size_y_cur,
					width=2,
					fill="#DDDDDD"
				)
				self.viewports['pwm'].create_line(
					self.viewport_current_x,
					0,
					self.viewport_current_x+1,
					self.viewport_size_y_pwm,
					width=2,
					fill="#DDDDDD"
				)

				# change 180 by the max rotation angle of the servo form ram
				# change 180 by the max rotation angle of the servo form ram
				# change 180 by the max rotation angle of the servo form ram

			if self.variables['goal_acceleration'].get() == 1:
				self.viewports['acceleration'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_pos/2-goal_acceleration/1000.0*self.viewport_size_y_pos/2,
					self.viewport_current_x+1,
					self.viewport_size_y_pos/2-goal_acceleration/1000.0*self.viewport_size_y_pos/2,
					width=2,
					fill="#0000FF"
				)
			if self.variables['setpoint_acceleration'].get() == 1:						
				self.viewports['acceleration'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_pos/2-setpoint_acceleration/1000.0*self.viewport_size_y_pos/2,
					self.viewport_current_x+1,
					self.viewport_size_y_pos/2-setpoint_acceleration/1000.0*self.viewport_size_y_pos/2,
					width=3,
					fill="#FF0000"
				)		
			if self.variables['present_acceleration'].get() == 1:
				self.viewports['acceleration'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_pos/2-present_acceleration/1000.0*self.viewport_size_y_pos/2,
					self.viewport_current_x+1,
					self.viewport_size_y_pos/2-present_acceleration/1000.0*self.viewport_size_y_pos/2,
					width=3,
					fill="#000000"
				)
		
			if self.variables['goal_velocity'].get() == 1:
				self.viewports['velocity'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_vel/2-goal_velocity/1200.0*self.viewport_size_y_vel/2,
					self.viewport_current_x+1,
					self.viewport_size_y_vel/2-goal_velocity/1200.0*self.viewport_size_y_vel/2,
					width=2,
					fill="#0000FF"
				)
			if self.variables['setpoint_velocity'].get() == 1:
				self.viewports['velocity'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_vel/2-setpoint_velocity/1200.0*self.viewport_size_y_vel/2,
					self.viewport_current_x+1,
					self.viewport_size_y_vel/2-setpoint_velocity/1200.0*self.viewport_size_y_vel/2,
					width=2,
					fill="#FF0000"
				)					
			if self.variables['present_velocity'].get() == 1:
				self.viewports['velocity'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_vel/2-present_velocity/1200.0*self.viewport_size_y_vel/2,
					self.viewport_current_x+1,
					self.viewport_size_y_vel/2-present_velocity/1200.0*self.viewport_size_y_vel/2,
					width=2,
					fill="#000000"
				)

			if self.variables['goal_current'].get() == 1:
				self.viewports['current'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_cur/2-goal_current/1000.0*self.viewport_size_y_cur/2,
					self.viewport_current_x+1,
					self.viewport_size_y_cur/2-goal_current/1000.0*self.viewport_size_y_cur/2,
					width=2,
					fill="#0000FF"
				)					
			if self.variables['setpoint_current'].get() == 1:
				self.viewports['current'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_cur/2-setpoint_current/1000.0*self.viewport_size_y_cur/2,
					self.viewport_current_x+1,
					self.viewport_size_y_cur/2-setpoint_current/1000.0*self.viewport_size_y_cur/2,
					width=2,
					fill="#FF0000"
				)					
			if self.variables['present_current'].get() == 1:
				self.viewports['current'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_cur/2-present_current/1000.0*self.viewport_size_y_cur/2,
					self.viewport_current_x+1,
					self.viewport_size_y_cur/2-present_current/1000.0*self.viewport_size_y_cur/2,
					width=2,
					fill="#000000"
				)					


			if self.variables['goal_pwm'].get() == 1:
				self.viewports['pwm'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_pwm/2-goal_pwm/100.0*self.viewport_size_y_pwm/2.0,
					self.viewport_current_x+1,
					self.viewport_size_y_pwm/2-goal_pwm/100.0*self.viewport_size_y_pwm/2.0,
					width=2,
					fill="#0000FF"
				)					
			if self.variables['setpoint_pwm'].get() == 1:
				self.viewports['pwm'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_pwm/2-setpoint_pwm/100.0*self.viewport_size_y_pwm/2.0,
					self.viewport_current_x+1,
					self.viewport_size_y_pwm/2-setpoint_pwm/100.0*self.viewport_size_y_pwm/2.0,
					width=2,
					fill="#000000"
				)					

			# inc x
			self.viewport_current_x = self.viewport_current_x + 1 

			# repeat
			if self.viewport_current_x > self.viewport_size_x_pos:
				self.clear()
