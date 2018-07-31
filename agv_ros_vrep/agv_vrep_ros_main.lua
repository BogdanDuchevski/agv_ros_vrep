function sysCall_init()

    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    xml = '<ui title="'..sim.getObjectName(robotHandle)..' speed" closeable="false" resizeable="false" activate="false">'..[[
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
    -- UI SPEED SLIDER INIT
    minMaxSpeed={50*math.pi/180,300*math.pi/180} -- Min and max speeds for each motor
    speed=(minMaxSpeed[1]+minMaxSpeed[2])*0.5
    simUI.setSliderValue(ui,1,100*(speed-minMaxSpeed[1])/(minMaxSpeed[2]-minMaxSpeed[1]))

    leftMotor=sim.getObjectHandle("bubbleRob_leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("bubbleRob_rightMotor") -- Handle of the right motor
    noseSensor=sim.getObjectHandle("bubbleRob_sensingNose") -- Handle of the proximity sensor
    -- Check if the required ROS plugin is there:
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginNotFound=false
        end
        index=index+1
    end
    -- Ok now launch the ROS client application:
    if (not pluginNotFound) then
        local sysTime=sim.getSystemTimeInMs(-1)
        local speedTopicName='speed'
        local leftEncTopicName='leftMotorEnc'
        local rightEncTopicName='rightMotorEnc'
        local leftMotorTopicName='leftMotorSpeed'--..sysTime -- we add a random component so that we can have several instances of this robot running
        local rightMotorTopicName='rightMotorSpeed'--..sysTime -- we add a random component so that we can have several instances of this robot running
        local sensorTopicName='sensorTrigger'--..sysTime -- we add a random component so that we can have several instances of this robot running
        local simulationTimeTopicName='simTime'--..sysTime -- we add a random component so that we can have several instances of this robot running
        -- Prepare the sensor publisher and the motor speed subscribers:
        speedTopicPub=simROS.advertise('/'..speedTopicName,'std_msgs/Float32')
        leftEncTopicPub=simROS.advertise('/'..leftEncTopicName,'std_msgs/Float32')
        rightEncTopicPub=simROS.advertise('/'..rightEncTopicName,'std_msgs/Float32')
        sensorPub=simROS.advertise('/'..sensorTopicName,'std_msgs/Bool')
        simTimePub=simROS.advertise('/'..simulationTimeTopicName,'std_msgs/Float32')
        leftMotorSub=simROS.subscribe('/'..leftMotorTopicName,'std_msgs/Float32','setLeftMotorVelocity_cb')
        rightMotorSub=simROS.subscribe('/'..rightMotorTopicName,'std_msgs/Float32','setRightMotorVelocity_cb')
        -- Now we start the client application:
        --result2=sim.launchExecutable('/home/bad/vrep_pro/src/ros_bubble_rob2/bin/rosBubbleRob2',leftMotorTopicName.." "..rightMotorTopicName.." "..sensorTopicName.." "..simulationTimeTopicName,0)
    end

end
function speedChange_callback(ui,id,newVal)
	speed=minMaxSpeed[1]+(minMaxSpeed[2]-minMaxSpeed[1])*newVal/100
  simROS.publish(speedTopicPub,{data=speed})
end
function setLeftMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    sim.setJointTargetVelocity(leftMotor,msg.data)
end

function setRightMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(rightMotor,msg.data)
end

function getTransformStamped(objHandle,name,relTo,relToName)
    t=sim.getSystemTime()
    p=sim.getObjectPosition(objHandle,relTo)
    o=sim.getObjectQuaternion(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end


function sysCall_actuation()
    -- Send an updated sensor and simulation time message, and send the transform of the robot:
    if not pluginNotFound then
        local result=sim.readProximitySensor(noseSensor)
        local detectionTrigger={}
        detectionTrigger['data']=result>0
        l_pos=sim.getJointPosition(leftMotor)
        r_pos=sim.getJointPosition(rightMotor)
        --sim.addStatusbarMessage('l_pos = '.. l_pos)
        simROS.publish(leftEncTopicPub,{data=l_pos})
        simROS.publish(rightEncTopicPub,{data=r_pos})
        simROS.publish(sensorPub,detectionTrigger)
        simROS.publish(simTimePub,{data=sim.getSimulationTime()})
        -- Send the robot's transform:
        simROS.sendTransform(getTransformStamped(robotHandle,'rosInterfaceControlledBubbleRob',-1,'world'))
        -- To send several transforms at once, use simROS.sendTransforms instead
    end
end

function sysCall_cleanup()
    if not pluginNotFound then
        -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
        simROS.shutdownPublisher(sensorPub)
        simROS.shutdownSubscriber(leftMotorSub)
        simROS.shutdownSubscriber(rightMotorSub)
    end
end
