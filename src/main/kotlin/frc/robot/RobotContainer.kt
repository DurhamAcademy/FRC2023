package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.ElevatorTestDown
import frc.robot.commands.ElevatorTestUp
import frc.robot.commands.MoveToPosition
import frc.robot.controls.ControlScheme
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator

class RobotContainer {
    val xbox = CommandXboxController(0)
    val controlScheme: ControlScheme = DefaultControlScheme(xbox)

    var cameraWrapper = PhotonCameraWrapper()

    @Suppress("unused")
    
    val drivetrain = Drivetrain(controlScheme, cameraWrappers = listOf(cameraWrapper))
    val elevator = Elevator(controlScheme)
    
    init {
        controlScheme.run {
            xbox!!.a().onTrue(ElevatorTestUp(elevator))
            xbox!!.b().onTrue(ElevatorTestDown(elevator))
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
        }
    }
}
