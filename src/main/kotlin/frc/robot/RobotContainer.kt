package frc.robot

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
            xbox!!.y()
                .whileTrue(
                    MoveToPosition(drivetrain, 14.5, 1.0, 0.0)
                )

            xbox!!.x()
                .whileTrue(
                    MoveToPosition(drivetrain, 0.0, 0.0, 0.0)
                )

            xbox!!.b()
                .whileTrue(
                    MoveToPosition(drivetrain, 14.5, 1.25, 90.0)
                )

            xbox!!.a()
                .whileTrue(
                    MoveToPosition(drivetrain, 1.0, 0.0, 180.0)
                )
        }
    }
}