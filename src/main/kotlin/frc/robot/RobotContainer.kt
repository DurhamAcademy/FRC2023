package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.ElevatorTestDown
import frc.robot.commands.ElevatorTestUp
import frc.robot.commands.MoveToPosition
import frc.robot.commands.arm.SetArmToAngle
import frc.robot.commands.wrist.SetWristAngle
import frc.robot.commands.manipulator.CloseManipulator
import frc.robot.commands.manipulator.GrabConeCommand
import frc.robot.commands.manipulator.HoldConeCommand
import frc.robot.commands.manipulator.OpenManipulator
import frc.robot.controls.ControlScheme
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Wrist
import kotlin.math.PI
import frc.robot.subsystems.Manipulator

class RobotContainer {
    val xbox = CommandXboxController(0)
    val controlScheme: ControlScheme = DefaultControlScheme(xbox)

//    var cameraWrapper: PhotonCameraWrapper = TODO("camera not working")//PhotonCameraWrapper()
    val manipulator = Manipulator()

    val drivetrain = Drivetrain(controlScheme, cameraWrappers = listOf(/*cameraWrapper*/))
    val elevator = Elevator(controlScheme)
    val arm = Arm()
    val wrist = Wrist()

    init {
        controlScheme.run {
            xbox!!.a().whileTrue(ElevatorTestUp(elevator))
            xbox!!.b().whileTrue(ElevatorTestDown(elevator))
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
                    SetArmToAngle(arm, PI / 2)
                )
            testArm0
                .whileTrue(
                    SetArmToAngle(arm, 0.0)
                )
            testArmNeg90
                .whileTrue(
                    SetArmToAngle(arm, -PI / 2)
                )

            // assign the wrist 90 trigger to the command that
            // moves the wrist to 90 degrees
            testWrist90
                .whileTrue(
                    SetWristAngle(wrist, PI / 2)
                )
            testWrist0
                .whileTrue(
                    SetWristAngle(wrist, 0.0)
                )
            testWristNeg90
                .whileTrue(
                    SetWristAngle(wrist, -PI / 2)
                )

            // assign the open manipulator trigger to the command that
            // opens the manipulator
            openManipulator
                .onTrue(
                    OpenManipulator(manipulator)
                )

            // assign the close manipulator trigger to the command that
            // closes the manipulator
            closeManipulator
                .onTrue(
                    CloseManipulator(manipulator)
                )

            // assign the grab cone trigger to the command that
            // grabs a cone
            grabCone
                .whileTrue(
                    GrabConeCommand(manipulator)
                        .until { manipulator.objectType == GamePiece.cone }
                        .andThen(HoldConeCommand(manipulator))
                        .repeatedly()
                )

            // assign the hold cone trigger to the command that
            // holds a cone
            holdCone
                .whileTrue(
                    HoldConeCommand(manipulator)
                )
        }

    }
}
