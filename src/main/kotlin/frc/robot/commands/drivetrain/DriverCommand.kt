package frc.robot.commands.drivetrain

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Game
import frc.robot.commands.pathing.MoveToPosition
import frc.robot.constants.Constants
import frc.robot.constants.drivetrain.maxVelocity
import frc.robot.constants.drivetrain.maxAutonomousAngularAcceleration
import frc.robot.constants.drivetrain.maxAutonomousAngularVelocity
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.slewLimited
import frc.robot.utils.xMul
import kotlin.math.PI

class DriverCommand(
    var drivetrain: Drivetrain,
    var controlScheme: ControlScheme,
    var nearestStation: () -> Boolean,
) : CommandBase() {
    private val rotationPid = ProfiledPIDController(
        MoveToPosition.rP, 0.0, 0.0, TrapezoidProfile.Constraints(
            maxAutonomousAngularVelocity, maxAutonomousAngularAcceleration
        )
    ).apply {
        enableContinuousInput(-PI, PI)

    }
    var lastNearestStation = nearestStation()

    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        if (nearestStation() != lastNearestStation) {
            lastNearestStation = nearestStation()
            rotationPid.reset(drivetrain.estimatedPose2d.rotation.radians)
        }
        val allianceMulitplier = when (Game.alliance) {
            DriverStation.Alliance.Invalid -> 1.0
            else -> Game.alliance.xMul
        }
        val vec = Translation2d(-controlScheme.forward, -controlScheme.strafe)
            .times(maxVelocity)
        drivetrain.drive(
            ChassisSpeeds(
                vec.x * Constants.powerPercent *
                        allianceMulitplier * (if (drivetrain.invertx.getBoolean(false)) -1 else 1)
                        * controlScheme.speedMutiplier,
                vec.y * Constants.powerPercent *
                        allianceMulitplier * (if (drivetrain.inverty.getBoolean(false)) -1 else 1)
                        * controlScheme.speedMutiplier,
                if(nearestStation()) {
                    rotationPid.calculate(drivetrain.estimatedPose2d.rotation.radians,
                        degreesToRadians(findNearest180(drivetrain.estimatedPose2d.rotation.degrees))
                    ) // desired radians
                }
                else{
                    -controlScheme.rotation * 2 * Math.PI *
                            Constants.powerPercent * .5 * (if (drivetrain.invertrot.getBoolean(false)) -1 else 1) * controlScheme.speedMutiplier
                }
            ),//.slewLimited(drivetrain.xSlewRateLimiter, drivetrain.ySlewRateLimiter, drivetrain.rotSlewRateLimiter),
            true,
            Translation2d() // chris wants in the middle
        )
    }
}