package frc.robot6502

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot6502.commands.DriveMotorTest
import frc.robot6502.commands.TurnMotorTest
import frc.robot6502.controls.ControlScheme
import frc.robot6502.controls.DefaultControlScheme
import frc.robot6502.subsystems.Drivetrain

class RobotContainer {
    val xbox = CommandXboxController(0)
    val controlScheme: ControlScheme = DefaultControlScheme(xbox)

    @Suppress("unused")
    val drivetrain = Drivetrain(controlScheme)
    init {
        controlScheme.run {
            driveTest.whileTrue(
                DriveMotorTest(
                    drivetrain,
                    if (testFrontLeft.asBoolean) drivetrain.frontLeft
                    else if (testFrontRight.asBoolean) drivetrain.frontRight
                    else if (testBackLeft.asBoolean) drivetrain.backLeft
                    else if (testBackRight.asBoolean) drivetrain.backRight
                    else null,
                    testPercent
                )
            )
            turnTest.whileTrue(
                TurnMotorTest(
                    drivetrain,
                    if (testFrontLeft.asBoolean) drivetrain.frontLeft
                    else if (testFrontRight.asBoolean) drivetrain.frontRight
                    else if (testBackLeft.asBoolean) drivetrain.backLeft
                    else if (testBackRight.asBoolean) drivetrain.backRight
                    else null,
                    testPercent
                )
            )
        }
        SmartDashboard.putData("Drive Motor Test", DriveMotorTest(drivetrain, null, 0.0))
        SmartDashboard.putData("Turn Motor Test", TurnMotorTest(drivetrain, null, 0.0))
    }
}