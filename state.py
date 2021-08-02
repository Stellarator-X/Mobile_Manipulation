import pybullet as p
cin = p.connect(p.SHARED_MEMORY)
if (cin < 0):
    cin = p.connect(p.GUI)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("urdf/apple1/apple.urdf", 0.300000,1.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("urdf/apple1/apple.urdf", 0.500000,0.500000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("urdf/apple1/apple.urdf", 0.500000,-1.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("urdf/apple1/apple.urdf", 0.500000,-2.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("urdf/apple1/apple.urdf", -1.590677,1.005622,0.029990,0.723200,0.579420,0.342211,0.155390)]
objects = [p.loadURDF("ur_description/urdf/arm_with_gripper.urdf", 0.171012,-0.369446,0.235026,-0.005044,0.000348,0.992375,0.123155)]
ob = objects[0]
jointPositions=[ -0.003142, 0.140381, -2.037771, 1.894842, -1.561444, -1.540834, 0.000000, 0.000000, 0.000004, 0.000000, 0.002828, 0.000000, -0.006004, 0.001390, 0.000567, 0.001954 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

objects = [p.loadURDF("ur_description/urdf/mobile_base_without_arm.urdf", 0.175537,-0.370500,0.081760,-0.005049,0.000349,0.992365,0.123229)]
ob = objects[0]
jointPositions=[ -39.950612, 191.892940, -39.943183, 191.901796, 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

cid0 = p.createConstraint(7,-1,6,-1,p.JOINT_FIXED,[0.000000,0.000000,0.000000],[0.000000,0.000000,0.100000],[0.000000,0.000000,0.000000],[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.changeConstraint(cid0,maxForce=500.000000)
p.setGravity(0.000000,0.000000,-9.810000)
p.stepSimulation()
p.disconnect()
