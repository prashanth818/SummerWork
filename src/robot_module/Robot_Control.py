from py4j.java_gateway import JavaGateway
gateway = JavaGateway()
arm_controller = gateway.entry_point
success = arm_controller.connectArm()
if success == True:
    arm_controller.moveArmToHome()

arm_controller.putArmToSleep()
