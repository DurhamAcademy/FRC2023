package frc.robot.commands.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain
import kotlin.math.PI
import kotlin.math.floor

class TurnMotorTest(
    private val drivetrain: Drivetrain,
) : CommandBase() {
    init {
        addRequirements(drivetrain)
        SmartDashboard.putData("huh?", this)

    }
    var time = Timer.getFPGATimestamp()
    override fun execute() {
        SmartDashboard.putData("huh?", this)
        time = Timer.getFPGATimestamp()
        drivetrain.swerveModuleStates = drivetrain.modules.map {
            SwerveModuleState(0.0, Rotation2d(floor((time / (PI * 2)) % .9999999999999999) * 4.0))
        }
    }

    override fun end(interrupted: Boolean) {
        SmartDashboard.putData("huh?", this)

    }

    override fun isFinished() = false
}