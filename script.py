from http import client
import sim
import time
import math

class ref_point:
    def __init__(self,clientID):
        self.clientID = clientID
        _,self.body = sim.simxGetObjectHandle(self.clientID,'/Path/ref_point',sim.simx_opmode_blocking)
        
        ##current ref_point body pos
        sim.simxGetObjectPosition(self.clientID,self.body,-1,sim.simx_opmode_streaming)

    def getbodyposition(self):
        _,body_pos=sim.simxGetObjectPosition(self.clientID,self.body,-1,sim.simx_opmode_buffer)
        return _,body_pos


class Manta:
    def __init__(self,clientID):
        self.clientID = clientID
        _,self.body = sim.simxGetObjectHandle(self.clientID,'/Manta',sim.simx_opmode_blocking)
        _,self.steer_handle = sim.simxGetObjectHandle(self.clientID,'./steer_joint',sim.simx_opmode_blocking)
        _,self.motor_handle = sim.simxGetObjectHandle(self.clientID,'./motor_joint',sim.simx_opmode_blocking)
        _,self.fl_brake_handle = sim.simxGetObjectHandle(self.clientID,'./fl_brake_joint',sim.simx_opmode_blocking)
        _,self.fr_brake_handle = sim.simxGetObjectHandle(self.clientID,'./fr_brake_joint',sim.simx_opmode_blocking)
        _,self.bl_brake_handle = sim.simxGetObjectHandle(self.clientID,'./bl_brake_joint',sim.simx_opmode_blocking)
        _,self.br_brake_handle = sim.simxGetObjectHandle(self.clientID,'./br_brake_joint',sim.simx_opmode_blocking)
        
        self.max_steer_angle=0.5235987
        self.motor_torque=60
        self.dVel=0
        self.dSteer=0.1
        self.steer_angle=0
        self.motor_velocity=self.dVel*10
        self.brake_force=0

        ##current body pos
        sim.simxGetObjectPosition(self.clientID,self.body,-1,sim.simx_opmode_streaming)
        ##current body orientation
        sim.simxGetObjectOrientation(self.clientID,self.body,-1,sim.simx_opmode_streaming)
        ##current steer pos
        sim.simxGetJointPosition(self.clientID,self.steer_handle,sim.simx_opmode_streaming)
        ##current angular velocity of back left wheel
        sim.simxGetObjectFloatParam(self.clientID,self.bl_brake_handle,sim.sim_jointfloatparam_velocity,sim.simx_opmode_streaming)
        # current angular velocity of back right wheel
        sim.simxGetObjectFloatParam(self.clientID,self.br_brake_handle,sim.sim_jointfloatparam_velocity,sim.simx_opmode_streaming)
        

    def getbodyposition(self):
        _,body_pos=sim.simxGetObjectPosition(self.clientID,self.body,-1,sim.simx_opmode_buffer)
        return _,body_pos

    def getbodyorientation(self):
        _,body_orient=sim.simxGetObjectOrientation(self.clientID,self.body,-1,sim.simx_opmode_buffer)
        return _,body_orient

    def move(self, steer_angle=0, motor_velocity=0,brake_force=0):
        ## Get Param
        ##current steer pos
        _,steer_pos=sim.simxGetJointPosition(self.clientID,self.steer_handle,sim.simx_opmode_buffer)
        ##current angular velocity of back left wheel
        _,bl_wheel_velocity=sim.simxGetObjectFloatParam(self.clientID,self.bl_brake_handle,sim.sim_jointfloatparam_velocity,sim.simx_opmode_buffer)
        ##current angular velocity of back right wheel
        _,br_wheel_velocity=sim.simxGetObjectFloatParam(self.clientID,self.br_brake_handle,sim.sim_jointfloatparam_velocity,sim.simx_opmode_buffer)
        

        ## average angular velocity of the back wheels
        rear_wheel_velocity=(bl_wheel_velocity+br_wheel_velocity)/2
        ##linear velocity
        linear_velocity=rear_wheel_velocity*0.09 

        if (abs(motor_velocity)<self.dVel*0.1):
            brake_force=100
        else:
            brake_force=0
        
        ## set maximum steer angle
        if (steer_angle> self.max_steer_angle):
            steer_angle=self.max_steer_angle
        
        if (steer_angle< -self.max_steer_angle):
            steer_angle= -self.max_steer_angle
        
        sim.simxSetJointTargetPosition(self.clientID,self.steer_handle, steer_angle,sim.simx_opmode_oneshot)
        
        ##brake and motor can not be applied at the same time
        if(brake_force>0):
            sim.simxSetJointForce(self.clientID,self.motor_handle, 0,sim.simx_opmode_oneshot)
        else:
            sim.simxSetJointForce(self.clientID,self.motor_handle, self.motor_torque,sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID,self.motor_handle, motor_velocity,sim.simx_opmode_oneshot)
        
        sim.simxSetJointForce(self.clientID,self.fr_brake_handle, brake_force,sim.simx_opmode_oneshot)
        sim.simxSetJointForce(self.clientID,self.fl_brake_handle, brake_force,sim.simx_opmode_oneshot)
        sim.simxSetJointForce(self.clientID,self.bl_brake_handle, brake_force,sim.simx_opmode_oneshot)
        sim.simxSetJointForce(self.clientID,self.br_brake_handle, brake_force,sim.simx_opmode_oneshot)



print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(2)

    target = ref_point(clientID)
    car = Manta(clientID)
    try:
        while True:
            returnCodedummy,target_pos=target.getbodyposition()
            returnCodecar,car_pos=car.getbodyposition()
            returnCodecar_orient,car_orient=car.getbodyorientation()
            if returnCodedummy==sim.simx_return_ok and returnCodecar==sim.simx_return_ok and returnCodecar_orient==sim.simx_return_ok:
                distance = math.sqrt(pow(target_pos[0]-car_pos[0],2)+pow(target_pos[1]-car_pos[1],2))
                global_angle = math.atan2(target_pos[0]-car_pos[0],target_pos[1]-car_pos[1])
                angle = car_orient[2]*57.2958 + global_angle*57.2958
                
                if angle > 180:
                    angle-=360
                elif angle < -180:
                    angle+=360
                
                # print("target_pos",target_pos," | ","car_pos",car_pos," | ",angle)
                # print(car_orient[2]*57.2958)

                set_point = 0.5 # 0.5 meter
                PID_P=10
                errorValue = set_point - distance
                ctrl=errorValue*PID_P

                if angle > 30:
                    angle = 30
                elif angle<-30:
                    angle=-30

                car.move(-angle/57.2958,-ctrl)
                text = "angle: "+str(angle)+" | "+"Speed: "+str(ctrl)
                print(text)
                # sim.simxAddStatusbarMessage(clientID,text,sim.simx_opmode_oneshot)

            
    except KeyboardInterrupt:   #Checks if ctrl+c is pressed 
        car.move(0,0)
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        print("Stopping Simulation")
        pass





    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'CoppeliaSim Died!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
