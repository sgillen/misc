# import os, inspect
# currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# print("current_dir=" + currentdir)
# parentdir = os.path.join(currentdir, "../gym")
#
# os.sys.path.insert(0, parentdir)

import os
import pybullet as p
import pybullet_data

import time

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -10)

useRealTimeSim = 1

#for video recording (works best on Mac and Linux, not well on Windows)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.setRealTimeSimulation(useRealTimeSim)  # either this
p.loadURDF(pybullet_data.getDataPath() + "/plane.urdf")
#car = p.loadURDF("kuka_car.urdf")

#car = p.loadURDF(f"{os.path.dirname(__file__)}/racecar.urdf")
car = p.loadURDF(f"{os.path.dirname(__file__)}/stick_car.urdf")
for i in range(p.getNumJoints(car)):
  print(p.getJointInfo(car, i))

wheels = [2,3,5,7]

# inactive_wheels = []
# for wheel in inactive_wheels:
#   p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

steering = [4, 6]
targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 10, 10)
steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0)

while (True):
  maxForce = p.readUserDebugParameter(maxForceSlider)
  targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
  steeringAngle = p.readUserDebugParameter(steeringSlider)
  #print(targetVelocity)

  for wheel in wheels:
    p.setJointMotorControl2(car,
                            wheel,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetVelocity,
                            force=maxForce)

  for steer in steering:
    p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)

  steering
  if (useRealTimeSim == 0):
    p.stepSimulation()
  time.sleep(0.01)