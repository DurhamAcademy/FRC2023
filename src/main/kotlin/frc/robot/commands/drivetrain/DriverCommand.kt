package frc.robot.commands.drivetrain

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Game
import frc.robot.constants.Constants
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.slewLimited
import frc.robot.utils.xMul
import kotlin.math.PI

class DriverCommand(
    var drivetrain: Drivetrain,
    var controlScheme: ControlScheme,
    var nearestStation: Boolean,
) : CommandBase() {
    private val rotationPid = ProfiledPIDController(1.0, 0.0, 0.0, TrapezoidProfile.Constraints(2*PI, 4*PI)).apply {
        enableContinuousInput(-PI, PI)
    }

    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        val alianceMulitplier = when (Game.alliance) {
            DriverStation.Alliance.Invalid -> 1.0
            else -> Game.alliance.xMul
        }
        val vec = Translation2d(-controlScheme.forward, -controlScheme.strafe)
            .times(3.5)
        drivetrain.drive(
            ChassisSpeeds(
                vec.x * Constants.powerPercent *
                        alianceMulitplier * (if (drivetrain.invertx.getBoolean(false)) -1 else 1)
                        * controlScheme.speedMutiplier,
                vec.y * Constants.powerPercent *
                        alianceMulitplier * (if (drivetrain.inverty.getBoolean(false)) -1 else 1)
                        * controlScheme.speedMutiplier,
                if(nearestStation) {
                    rotationPid.calculate(drivetrain.estimatedPose2d.rotation.radians,
                         Nearest180.findNearest180(drivetrain)
                    ) // desired radians
                }
                else{
                    -controlScheme.rotation * 2 * Math.PI *
                            Constants.powerPercent * .5 * (if (drivetrain.invertrot.getBoolean(false)) -1 else 1) * controlScheme.speedMutiplier
                }
            ).slewLimited(drivetrain.xSlewRateLimiter, drivetrain.ySlewRateLimiter, drivetrain.rotSlewRateLimiter),
            true,
            Translation2d() // chris wants in the middle
        )
    }
}