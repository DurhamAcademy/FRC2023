package frc.robot.commands.drivetrain

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain


class AutoBalance(
    var drivetrain: Drivetrain
): CommandBase() {

    var speed: Double = 0.25
    val half = 11.0 //may need to fine tune this

    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        if ((-half..half).contains(drivetrain.gyro.pitch)) {
            println("stay ${drivetrain.gyro.pitch}")
            drivetrain.drive(
                ChassisSpeeds(0.0, 0.0, 0.0),
                true,
                Translation2d(0.0, 0.0)
            )
        } else if (drivetrain.gyro.pitch <= -half) {
            println("up ${drivetrain.gyro.pitch}")
            drivetrain.drive(
                ChassisSpeeds(speed, 0.0, 0.0),
                true,
                Translation2d(0.0, 0.0)
            )

        } else {
            println("down ${drivetrain.gyro.pitch}")
            drivetrain.drive(
                ChassisSpeeds(-speed, 0.0, 0.0),
                true,
                Translation2d(0.0, 0.0)
            )
        }
    }

    fun end() {
        drivetrain.drive(
            ChassisSpeeds(0.0, 0.0, 0.01),
            true,
            Translation2d(0.0, 0.0)
        )
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