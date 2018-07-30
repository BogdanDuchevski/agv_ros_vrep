if (sim_call_type==sim.syscb_init) then
	bbb=sim.getObjectAssociatedWithScript(sim.handle_self)
    j1=sim.getObjectHandle('ROBOTIQ_85_active1')
    j2=sim.getObjectHandle('ROBOTIQ_85_active2')
	connector=sim.getObjectHandle('ROBOTIQ_85_attachPoint')
    objectSensor=sim.getObjectHandle('ROBOTIQ_85_attachProxSensor')
	shape=sim.getObjectHandle('Cylinder12')
	out=sim.auxiliaryConsoleOpen('Debug2',5,1)
	--detectedObjectHandle=NULL
	--detectedSurfaceNormalVector=NULL

end

if (sim_call_type==sim.syscb_cleanup) then

end

if (sim_call_type==sim.syscb_actuation) then
    p1=sim.getJointPosition(j1)
    p2=sim.getJointPosition(j2)
    closing=sim.getIntegerSignal('gripper_signal')-- RECEIVE GRIPPER SIGNAL (UI)
    if (closing==1) then
		while true do
			result,distance,detectedPoint,detectedObjectHandle=sim.handleProximitySensor(objectSensor,0)
			sim.auxiliaryConsolePrint(out,detectedObjectHandle)
			sim.auxiliaryConsolePrint(out,'\n')
			if (detectedObjectHandle==shape) then
			--if (sim.checkProximitySensor(objectSensor,shape)==1) then
				sim.auxiliaryConsolePrint(out,'non-static respondable shape was detected')
				-- connect detected non-static respondable shape
				sim.setObjectParent(shape,connector,true)
				break
			else
				break
			end
		end
        if (p1<p2-0.008) then
            sim.setJointTargetVelocity(j1,-0.01)
            sim.setJointTargetVelocity(j2,-0.04)
        else
            sim.setJointTargetVelocity(j1,-0.04)
            sim.setJointTargetVelocity(j2,-0.04)
        end
    else
		sim.setObjectParent(shape,-1,true)
        if (p1<p2) then
            sim.setJointTargetVelocity(j1,0.04)
            sim.setJointTargetVelocity(j2,0.02)
        else
            sim.setJointTargetVelocity(j1,0.02)
            sim.setJointTargetVelocity(j2,0.04)
        end
    --    sim.setJointTargetVelocity(j1,0.04)
    --    sim.setJointTargetVelocity(j2,0.04)
    end

    -- You have basically 2 alternatives to grasp an object:
    --
    -- 1. You try to grasp it in a realistic way. This is quite delicate and sometimes requires
    --    to carefully adjust several parameters (e.g. motor forces/torques/velocities, friction
    --    coefficients, object masses and inertias)
    --
    -- 2. You fake the grasping by attaching the object to the gripper via a connector. This is
    --    much easier and offers very stable results.
    --
    -- Alternative 2 is explained hereafter:
    --
    --
    -- a) In the initialization phase, retrieve some handles:
    --
    -- connector=sim.getObjectHandle('ROBOTIQ_85_attachPoint')
    -- objectSensor=sim.getObjectHandle('ROBOTIQ_85_attachProxSensor')

    -- b) Before closing the gripper, check which dynamically non-static and respondable object is
    --    in-between the fingers. Then attach the object to the gripper:
    --
    -- index=0
    -- while true do
    --     shape=sim.getObjects(index,sim.object_shape_type)
    --     if (shape==-1) then
    --         break
    --     end
    --     if (sim.getObjectInt32Parameter(shape,sim.shapeintparam_static)==0) and (sim.getObjectInt32Parameter(shape,sim.shapeintparam_respondable)~=0) and (sim.checkProximitySensor(objectSensor,shape)==1) then
    --         -- Ok, we found a non-static respondable shape that was detected
    --         attachedShape=shape
    --         -- Do the connection:
    --         sim.setObjectParent(attachedShape,connector,true)
    --         break
    --     end
    --     index=index+1
    -- end

    -- c) And just before opening the gripper again, detach the previously attached shape:
    --
    -- sim.setObjectParent(attachedShape,-1,true)
end
