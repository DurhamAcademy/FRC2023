package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.MoveToPosition
import frc.robot.controls.ControlScheme
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.Drivetrain

class RobotContainer {
    val xbox = CommandXboxController(0)
    val controlScheme: ControlScheme = DefaultControlScheme(xbox)

    var cameraWrapper = PhotonCameraWrapper()

    @Suppress("unused")
    val drivetrain = Drivetrain(controlScheme, cameraWrappers = listOf(cameraWrapper))

    init {
        controlScheme.run {
            // assign the go to april tag 1 trigger to the command that
            // moves the robot to the april tag
            testGoToAprilTag1
                .whileTrue(
                    MoveToPosition(drivetrain, 14.5, 1.0, 180.0)
                        .andThen(
                            MoveToPosition(
                                drivetrain,
                                Pose2d(
                                    Translation2d(
                                        14.0,
                                        .50
                                    ), Rotation2d.fromDegrees(90.0)
                                ),
                                velocity = Transform2d(
                                    Translation2d(0.0, 0.0),
                                    Rotation2d.fromDegrees(0.0)
                                )
                            )
                        )
//                        .andThen(MoveToPosition(drivetrain, 5.0, 5.0, 270.0))
//                        .andThen(MoveToPosition(drivetrain))
                )

            // assign the go-to zero zero trigger to the command that
            // moves the robot to (0, 0)
            testGoToZeroZero
                .whileTrue(
                    MoveToPosition(drivetrain, 0.0, 0.0, 0.0)
                )
        }
    }
}
