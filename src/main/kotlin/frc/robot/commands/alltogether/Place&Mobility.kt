package frc.robot.commands.alltogether
/*

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmConstants
import frc.robot.subsystems.DrivetrainConstants
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Manipulator

class `Place&Mobility`(
    drivetrain: DrivetrainConstants,
    arm: ArmConstants,
    elevator: Elevator,
    manipulator: Manipulator,
):
    CommandBase()
{

            SetManipulatorSpeed(manipulator, 0.1).withTimeout(0.5).andThen(
                LeaveStartConfig(this@RobotContainer, arm).andThen(
                    SetSubsystemPosition(this@RobotContainer, {IOLevel.High}, {cone}).withTimeout(2.0).andThen(
                        Throw(manipulator, {cone}, {PlacementLevel.Level3}).withTimeout(1.0).andThen(
                            SetSubsystemPosition(this@RobotContainer, {IOLevel.Idle}, {cone}).withTimeout(2.0)
                        ).andThen(
                            MoveToPosition(
                                drivetrain,
                                {_,_,_->
                                    Pose2d(
                                        when (Game.alliance) {
                                            DriverStation.Alliance.Blue -> 5.81
                                            DriverStation.Alliance.Red -> 10.75
                                            else -> drivetrain.estimatedPose2d.x
                                        },
                                        4.62,
                                        drivetrain.estimatedPose2d.rotation
                                    )
                                }
                            ).withTimeout(5.0)
                        )
                    )
                )
            )
        )
}
*/
