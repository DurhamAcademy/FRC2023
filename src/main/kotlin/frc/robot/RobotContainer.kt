package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.controls.ControlScheme
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.Drivetrain

class RobotContainer {
    val xbox = CommandXboxController(0)
    val controlScheme: ControlScheme = DefaultControlScheme(xbox)

    @Suppress("unused")
    val drivetrain = Drivetrain(controlScheme)
    init {
        controlScheme.run {
//            driveTest.whileTrue(
//                DriveMotorTest(
//                    drivetrain,
//                    controlScheme
//                ).repeatedly()
//            )
//            turnTest.whileTrue(
//                TurnMotorTest(
//                    drivetrain,
//                    if (testFrontLeft.asBoolean) drivetrain.frontLeft
//                    else if (testFrontRight.asBoolean) drivetrain.frontRight
//                    else if (testBackLeft.asBoolean) drivetrain.backLeft
//                    else if (testBackRight.asBoolean) drivetrain.backRight
//                    else null,
//                    testPercent
//                )
//            )
        }
//        SmartDashboard.putData("Drive Motor Test", DriveMotorTest(drivetrain, null, 0.0))
//        SmartDashboard.putData("Turn Motor Test", TurnMotorTest(drivetrain, null, 0.0))
    }
}