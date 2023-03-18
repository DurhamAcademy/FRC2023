package frc.robot.commands.drivetrain

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain


class AutoBalance(
    var drivetrain: Drivetrain
): CommandBase() {

    var speed: Double = 0.3
    val half: Int = 7 //may need to fine tune this
    init{
        addRequirements(drivetrain)
    }
    override fun execute(){
        if (drivetrain.gyro.pitch >= half) {
            drivetrain.drive(
                ChassisSpeeds(0.0,1.0,0.0),
                true,
                Translation2d(0.0,0.0)
                )
        } else if (drivetrain.gyro.pitch <= -half) {
            drivetrain.drive(
                ChassisSpeeds(0.0, 1.0, 0.0),
                true,
                Translation2d(0.0,0.0)
                )

        } else {
            drivetrain.drive(
                ChassisSpeeds(0.0, 0.0, 0.0),
                true,
                Translation2d(0.0, 0.0)
            )
        }
        speed = drivetrain.gyro.pitch / 130;

    }

    fun end(){
        drivetrain.drive(
            ChassisSpeeds(0.0, 0.0, 0.0),
            true,
            Translation2d(0.0, 0.0)
        )
    }
//
//    override fun isFinished() {
//
//    }
}