package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.MoveToPosition
import frc.robot.commands.arm.SetArmTo90
import frc.robot.controls.ControlScheme
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Wrist

class RobotContainer {
    val xbox = CommandXboxController(0)
    val controlScheme: ControlScheme = DefaultControlScheme(xbox)

//    var cameraWrapper: PhotonCameraWrapper = TODO("camera not working")//PhotonCameraWrapper()

    @Suppress("unused")
    val drivetrain = Drivetrain(controlScheme, cameraWrappers = listOf(/*cameraWrapper*/))
    val arm = Arm()
    val wrist = Wrist()

    init {
        controlScheme.run {
            // assign the go to april tag 1 trigger to the command that
            // moves the robot to the april tag
            testGoToAprilTag1
                .whileTrue(
                    MoveToPosition(drivetrain, 14.5, 1.0, 0.0)
                )

            // assign the go-to zero zero trigger to the command that
            // moves the robot to (0, 0)
            testGoToZeroZero
                .whileTrue(
                    MoveToPosition(drivetrain, 0.0, 0.0, 0.0)
                )

            // assign the arm 90 trigger to the command that
            // moves the arm to 90 degrees
            testArm90
                .whileTrue(
                    SetArmTo90(arm)
                )
        }

    }
}
