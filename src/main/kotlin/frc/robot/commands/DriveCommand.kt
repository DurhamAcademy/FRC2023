package frc.robot.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain

class DriveCommand(
    var drivetrain: Drivetrain,
    var controlScheme: ControlScheme,
) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }
//    val rotPID
    override fun execute() {
        val vec = Translation2d(controlScheme.forward, controlScheme.strafe).times(2.0)
//        val normalized = if (vec.norm == 0.0) Translation2d() else vec.div(vec.norm).times(2.0)
        drivetrain.drive(
            ChassisSpeeds(
                vec.x * Constants.powerPercent,
                vec.y * Constants.powerPercent,
                -controlScheme.rotation *2 * Math.PI *
                        Constants.powerPercent *.25
            ),
            true,
            Translation2d(-Constants.MODULE_DISTANCE_X *0.4, 0.0)

        )

    }
}