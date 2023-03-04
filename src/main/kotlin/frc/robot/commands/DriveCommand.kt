package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand
import frc.kyberlib.command.Game
import frc.robot.Constants
import frc.robot.commands.pathing.MoveToPosition
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain
import kotlin.math.PI

class DriveCommand(
    var drivetrain: Drivetrain,
    var controlScheme: ControlScheme,
) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }
    var heldRotation2d = Rotation2d.fromDegrees(drivetrain.gyro.yaw)
    val rPIDController = ProfiledPIDController(
        MoveToPosition.rP, 0.0, 0.0, TrapezoidProfile.Constraints(
            PI / 2, PI / 1.5
        )
    ).also {
        it.enableContinuousInput(-PI, PI)
        it.reset(drivetrain.estimatedPose2d.rotation.radians, drivetrain.estimatedVelocity.rotation.radians)
    }
    override fun execute() {
        val alianceMulitplier = when (Game.alliance) {
            Alliance.Invalid -> 1.0
            Alliance.Blue -> Constants.Field2dLayout.Axes.Blue.fieldOffsetMultiplier
            Alliance.Red -> Constants.Field2dLayout.Axes.Red.fieldOffsetMultiplier
        }
        val vec = Translation2d(-controlScheme.forward, controlScheme.strafe)
            .times(2.0)
        val rotCorrection =
            if (controlScheme.rotation == 0.0)
                rPIDController.calculate(
                    Rotation2d.fromDegrees(drivetrain.gyro.yaw).radians,
                    heldRotation2d.radians
                )
            else 0.0
                .also { heldRotation2d = Rotation2d.fromDegrees(drivetrain.gyro.yaw) }

        drivetrain.drive(
            ChassisSpeeds(
                vec.x * Constants.powerPercent * alianceMulitplier,
                vec.y * Constants.powerPercent,
                -controlScheme.rotation *2 * Math.PI *
                        Constants.powerPercent *.25
                +rotCorrection
            ),
            true,
            Translation2d() // chris wants in the middle
        )
    }
}