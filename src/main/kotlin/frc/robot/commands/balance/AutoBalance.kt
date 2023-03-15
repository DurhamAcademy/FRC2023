package frc.robot.commands.balance

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.Div
import frc.kyberlib.math.units.KUnit
import frc.kyberlib.math.units.Radian
import frc.kyberlib.math.units.Second
import frc.kyberlib.math.units.extensions.*
import frc.robot.subsystems.Drivetrain
import kotlin.math.*

class AutoBalance(
    val drivetrain: Drivetrain
) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    val currentPitch: Angle
        get() = drivetrain.gyro.pitch.degrees
    val currentRoll: Angle
        get() = drivetrain.gyro.roll.degrees
    val currentYaw: Angle
        get() = drivetrain.gyro.yaw.degrees

    // inclination is the angle between the robot's x,y plane and the ground.
    // given the pitch and roll, we can calculate the inclination of the robot
    // using the following formula:
    // inclination = arctan(pitch / sqrt(1 - pitch^2) * cos(roll) + roll * sin(roll))
    val currentInclination: Angle
        get() = atan2(
            currentPitch.radians,
            sqrt(1 - currentPitch.radians.pow(2)) * cos(currentRoll.radians) + currentRoll.radians * sin(currentRoll.radians)
        ).radians
    val lastInclination = currentInclination
    val lastTime = Timer.getFPGATimestamp().seconds
    val globalPitchVelocity: KUnit<Div<Radian, Second>>
        get() = (currentInclination - lastInclination) /
                (Timer.getFPGATimestamp().seconds - lastTime)


    val debouncer = Debouncer(0.05)
    override fun execute() {
        drivetrain.drive(
            ChassisSpeeds(
                -1.0,
                0.0,
                0.0
            ),
            true
        )

        SmartDashboard.putNumber("currentPitchHeading", currentPitch.degrees)
        SmartDashboard.putNumber("globalPitchVelocity", globalPitchVelocity.degreesPerSecond)
        SmartDashboard.putNumber("drivetrainGlobalPitch", lastInclination.degrees)
        SmartDashboard.putNumber("drivetrainGlobalPitch", currentInclination.degrees)
    }

    override fun isFinished(): Boolean =
        debouncer.calculate(globalPitchVelocity.absoluteValue.degreesPerSecond > 10)
}