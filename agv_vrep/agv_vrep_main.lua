--LUA Script exploring Vrep's functionalities:
-- // AGV PROJECT //
-- BubbleRob Robot from Vrep Model Library;
-- Qt Based Gui & Mouse Input Listener;
-- Keyboard Input Listener;
-- Task Managing Function;
-- Navigation Modes:
--   Obstacle Avoidance Mode, User Control Mode
-- Orientation & Velocity Command Function
-- Proximity Sensors: Front, Oscillating, Bottom
-- Wheel Encoders Developed
-- Servo Motor Actuation/Control: Velocity / VREP PID Position Control
-- Interface with HOKUYO URG Sensor Model:
--   Mapping to Point Cloud Function
--   Vrep Communication Tunnel
-- Interface with ROBOTIQ 85 End Effector Model:
--   Sense Dynamic Objects & Pick/Place
--   Vrep Communication Tunnel
-- Vision/Camera Sensor Reading
-- Wireless Signal Communication Test
-- Gyro & Accelerometer Sensors Signal Communication Test
-- Print Data:
--   Vrep Shell & Console Logs; Vrep Graph

function abs_b(x)
	if x < 0 then
		x = x*(-1)
	end
	return(x)
end

-- ORIENTATION & VELOCITY FUNCTION
-- Assigns Command ID; Assigns Orientation & Speed for Each Wheel Dependent on Task Manager;
function orn_and_spd(dtr)
	if dtr == 1 then-- UP
		cmnd_id=dtr
		cmnd={1,1}
	end
	if dtr == 2 then-- DOWN
		cmnd={-1,-1}
		cmnd_id=dtr
	end
	if dtr == 3 then-- SPACE
		cmnd={0,0}
		cmnd_id=dtr
	end
	if dtr == 4 then-- RIGHT
		cmnd={-1,1}
		cmnd_id=dtr
	end
	if dtr == 5 then-- LEFT
		cmnd={1,-1}
		cmnd_id=dtr
	end
end

-- KEYBOARD INPUT LISTENER
-- Loops Keyboard Buffer; Assigns Keyboard Input ID; Calls Task Manager Function
function key_cntrl(message,auxiliaryData)
	while message~=-1 do
		if (message==sim.message_keypress) then
			sim.addStatusbarMessage("message : '"..auxiliaryData[1].."'")
			--sim.auxiliaryConsolePrint(out,"message : '"..auxiliaryData[1].."'"..'\n')
			if (auxiliaryData[1]==2007) then-- up key
				p_key=1
			elseif (auxiliaryData[1]==2008) then-- down key
				p_key=2
			elseif (auxiliaryData[1]==32) then-- space key
				p_key=3
			elseif (auxiliaryData[1]==2009) then-- r key
				p_key=4
			elseif (auxiliaryData[1]==2010) then-- l key
				p_key=5
			end
			if p_key>0 then
				task_m(10,p_key)--
			end
		end
		message,auxiliaryData=sim.getSimulatorMessage()
	end
end

-- TASK MANAGING FUNCTION
-- Assigns tasks based on user input
function task_m(tsk_id,tsk_status)
	-- MAPPING
	-- Open/Close Mapping Task (Mouse Call)
	if tsk_id==2 then
		map_task = tsk_status
	end
	-- GRIPPER TASK
	-- Open & Send Signal to Gripper ROBOTIQ_85 End Effector Sript (Mouse Call)
	if tsk_id==4 then
		sim.setIntegerSignal('gripper_signal',tsk_status)
	end
	-- ARM ACTUATION TASK
	-- Actuate Arm 0/90 Degrees (Mouse Call)
	if tsk_id == 5 then
		if tsk_status == 1 then
			sim.setJointTargetPosition(serv3,90*math.pi/180)
		else
			sim.setJointTargetPosition(serv3,0)
		end
	end
	-- KEYBOARD CONTROL TASK
	-- Open/Close Keyboard Task (Mouse Call)
	if tsk_id == 6 then
		key_cntrl_task = tsk_status
	end
	-- ORIENTATION & SPEED TASK
	-- Open/Close Orientation & Speed Task (Keybord Call)
	if tsk_id == 10 then
		orn_spd_task = orn_and_spd(tsk_status)
	end
end

-- UI INPUT LISTENER FUNCTION
-- Assigns caller ID and Clears Previous ID based on UI Mouse Input; Calls Task Managing Funtion
function cll(ui,id)
	if call_id then
		if call_id >0 then
			call_id=0
			st=0
			for i=2,6,1
			do
				simUI.setStyleSheet(ui, i, 'background-color: grey')
			end
		else
			call_id=id
			st=1
			simUI.setStyleSheet(ui, id, 'background-color: yellow')
		end
	else
		call_id=id
		st=1
		simUI.setStyleSheet(ui, id, 'background-color: yellow')
	end
	task_m(id,st)
end

-- UI INPUT SPEED CHANGE FUNCTION
-- Defines Speed Based on UI Scroll Input;
function speedChange_callback(ui,id,newVal)
	speed=minMaxSpeed[1]+(minMaxSpeed[2]-minMaxSpeed[1])*newVal/100
end

-- INIT LOOP - Executed Once
if (sim_call_type==sim.syscb_init) then

	-- ROBOT & PERIPHERALS HANDLES INIT
	bubbleRobBase=sim.getObjectAssociatedWithScript(sim.handle_self) -- this is bubbleRob's handle

	-- ACTUATOR HANDLES INIT
	serv2=sim.getObjectHandle("Revolute_joint0") -- Handle of the servo
	serv=sim.getObjectHandle("Revolute_joint") -- Handle of the servo
	serv3=sim.getObjectHandle("Revolute_joint1") -- Handle of the left motor
	leftMotor=sim.getObjectHandle("bubbleRob_leftMotor") -- Handle of the left motor
	rightMotor=sim.getObjectHandle("bubbleRob_rightMotor") -- Handle of the right motor

	-- ENCODER INIT
	previous_angle=0
	cumulative_angles=0
	previous_angle2=0
	cumulative_angles2=0

	-- PROXIMITY SENSORS INIT
	sens_f1=sim.getObjectHandle("bubbleRob_sensingNose") -- Handle of the proximity sensor
	sens_f2=sim.getObjectHandle("Proximity_sensor") -- Handle of the proximity sensor
	sens_b1=sim.getObjectHandle("s_down_f") -- Handle of the proximity sensor
	sens_b2=sim.getObjectHandle("s_down_b") -- Handle of the proximity sensor

	-- HOKUYO SENSOR INIT
	laserScannerHandle=sim.getObjectHandle("Hokuyo_URG_04LX_UG01")
	laserScannerObjectName=sim.getObjectName(laserScannerHandle)
	communicationTube=sim.tubeOpen(0,laserScannerObjectName..'_HOKUYO',1)
	pt=sim.getObjectHandle('Point_cloud')
	color=nil
	duplicateTolerance=0.01

	-- MISC INIT
	backUntilTime=-1 -- Tells whether bubbleRob is in forward or backward mode
	drc = 1
	drc2 = 1

	-- UI INIT
	xml = '<ui title="'..sim.getObjectName(bubbleRobBase)..' speed" closeable="false" resizeable="false" activate="false">'..[[
              <hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>
          <label text="" style="* {margin-left: 300px;}"/>
		<button enabled="true" text="Control" on-click="cll" id="6" style="* {background-color: grey;}" />
		<button enabled="true" text="Mapping" on-click="cll" id="2" style="* {background-color: grey;}"/>
		<button enabled="true" text="Navigation" on-click="cll" id="3" style="* {background-color: grey;}"/>
		<button enabled="true" text="Gripper" on-click="cll" id="4" style="* {background-color: grey;}"/>
		<button enabled="true" text="Gripper2" on-click="cll" id="5" style="* {background-color: grey;}"/>
      </ui>
      ]]
	ui=simUI.create(xml)
	out=sim.auxiliaryConsoleOpen('Encoders',5,1)

	-- UI SPEED SLIDER INIT
	minMaxSpeed={50*math.pi/180,300*math.pi/180} -- Min and max speeds for each motor
	speed=(minMaxSpeed[1]+minMaxSpeed[2])*0.5
	simUI.setSliderValue(ui,1,100*(speed-minMaxSpeed[1])/(minMaxSpeed[2]-minMaxSpeed[1]))
end

-- MAIN LOOP
if (sim_call_type==sim.syscb_actuation) then
	p_key= 0

	-- READ SENSOR DATA
	sens_f1_d=sim.readProximitySensor(sens_f1) -- Read the proximity sensor
	sens_f2_d=sim.readProximitySensor(sens_f2) -- Read the proximity sensor
	sens_b1_d=sim.readProximitySensor(sens_b1) -- Read the proximity sensor
	sens_b2_d=sim.readProximitySensor(sens_b2) -- Read the proximity sensor

	-- READ SERVO POSITION DATA
	prb=sim.getJointPosition(serv)
	prb2=sim.getJointPosition(serv2)
	prb3=sim.getJointPosition(serv3)

	-- KEYBOARD LISTENER FUNCTION CALL
	message,auxiliaryData=sim.getSimulatorMessage()
	if key_cntrl_task == 1 then
		key_cntrl(message,auxiliaryData)
	end

	-- MAPPING TASK
	if (map_task == 1) then
		-- READ SENSOR DATA FROM COMMUNICATION TUBE
		data=sim.tubeRead(communicationTube)
		if (data) then
			laserDetectedPoints=sim.unpackFloatTable(data)
			-- INSERT DATA TO POINT CLOUD
			sim.insertPointsIntoPointCloud(pt,3,laserDetectedPoints,color,duplicateTolerance)
			--DISABLE VREP PID CONTROLLER
			simSetObjectInt32Parameter(serv,sim_jointintparam_ctrl_enabled,0)
			simSetObjectInt32Parameter(serv2,sim_jointintparam_ctrl_enabled,0)
			-- CONTROL OSCILLATING MOTION
			if (prb>45*math.pi/180) and drc > 0 then
				drc = drc*-1
			elseif (prb<(45*math.pi/180)*-1) and drc < 0 then
				drc = drc*-1
			end
			if (prb2>45*math.pi/180) and drc2 > 0 then
				drc2 = drc2*-1
			elseif (prb2<(45*math.pi/180)*-1) and drc2 < 0 then
				drc2 = drc2*-1
			end
			-- ACTUATE SERVO
			sim.setJointTargetVelocity(serv,drc)
			sim.setJointTargetVelocity(serv2,drc2)
		end
	else
		--ENABLE VREP PID CONTROLLER
		simSetObjectInt32Parameter(serv,sim_jointintparam_ctrl_enabled,1)
		simSetObjectInt32Parameter(serv2,sim_jointintparam_ctrl_enabled,1)
		-- HOME SERVO
		sim.setJointTargetPosition(serv,0)
		sim.setJointTargetPosition(serv2,0)
	end

	-- NAVIGATION TASK
	if call_id == 3 then
		-- SET TIMER IF OBSTACLE
		if (sens_f1_d>0) or (sens_b1_d==0) then backUntilTime=sim.getSimulationTime()+4 end
		if (backUntilTime<sim.getSimulationTime()) then
			-- FORWARD MODE ACTUATION
			sim.setJointTargetVelocity(leftMotor,speed)
			sim.setJointTargetVelocity(rightMotor,speed)
		else
			-- OBSTACLE MODE ACTUATION
			sim.setJointTargetVelocity(leftMotor, -speed/2)
			sim.setJointTargetVelocity(rightMotor, -speed/8)
		end
	elseif call_id == 6 and cmnd then
		-- KEYBOARD CONTROL MODE ACTUATION
		sim.setJointTargetVelocity(leftMotor,speed*cmnd[1])
		sim.setJointTargetVelocity(rightMotor,speed*cmnd[2])
	else
		sim.setJointTargetVelocity(leftMotor,speed*0)
		sim.setJointTargetVelocity(rightMotor,speed*0)
	end

	-- ENCODER A
	current_p = sim.getJointPosition(leftMotor)-- CURRENT POSITION
	current_p2 = sim.getJointPosition(rightMotor)-- CURRENT POSITION
	if previous_angle then
		angle_result=current_p-previous_angle-- GET ANGLE RESULT
	end
	if (abs_b(angle_result)>math.pi)then
		angle_result=angle_result - (2.0 * math.pi * abs_b(angle_result)/angle_result)-- ADD MAGIC
	end
	cumulative_angles = cumulative_angles + angle_result-- ADD ANGLE RESULT
	previous_angle = current_p-- DEFINE PREVIOUS ANGLE
	encoder=(1000*cumulative_angles)/(2.0*math.pi)
	-- ENCODER B
	if previous_angle2 then
		angle_result2=current_p2-previous_angle2-- GET ANGLE RESULT
	end
	if (abs_b(angle_result2)>math.pi)then
		angle_result2=angle_result2 - (2.0 * math.pi * abs_b(angle_result2)/angle_result2)-- ADD MAGIC
	end
	cumulative_angles2 = cumulative_angles2 + angle_result2-- ADD ANGLE RESULT
	previous_angle2 = current_p2-- DEFINE PREVIOUS ANGLE
	encoder2=(1000*cumulative_angles2)/(2.0*math.pi)

	-- PRINT & DEB
	sim.auxiliaryConsolePrint(out,encoder..'\n')
	sim.auxiliaryConsolePrint(out,encoder2..'\n')
	--sim.addStatusbarMessage(sim.getScriptName(sim.handle_self)..": encoder = '"..encoder.."'")
	--sim.addStatusbarMessage(sim.getScriptName(sim.handle_self)..": encoder2 = '"..encoder2.."'")
end

-- CLEANUP
if (sim_call_type==sim.syscb_cleanup) then
    simUI.destroy(ui)
end
