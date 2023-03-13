package frc.robot.commands.balance

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain
import kotlin.math.absoluteValue
import kotlin.math.atan
import kotlin.math.sqrt
import kotlin.math.tan

class AutoBalance(
    val drivetrain: Drivetrain
) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    // inclination = atan(sqrt(tan^2(roll)+tan^2(pitch)))
    val drivetrainGlobalPitch: Double
        get() = atan(sqrt(Math.pow(tan(drivetrain.gyro.pitch), 2.0) + Math.pow(tan(drivetrain.gyro.roll), 2.0)))
    val lastDrivetrainGlobalPitch: Double = drivetrainGlobalPitch
    val lastTime: Double = Timer.getFPGATimestamp()
    var globalPitchVelocity = 0.0

    // the formula for getting the direction of inclination given the pitch and
    // roll is:
    // heading = atan(tan(pitch)/cos(roll))
//    val autoTab = Shuffleboard.getTab("Autonomous")
//    val currentPitchHeadingEntry = autoTab.add("Current Pitch Heading", 0.0)
//        .withWidget(BuiltInWidgets.kGraph)
//        .withProperties(mapOf("min" to -1.0, "max" to 1.0))
//        .entry
//    val pitchHeadingEntry = autoTab.add("Pitch Heading", 0.0)
//        .withWidget(BuiltInWidgets.kGraph)
//        .withProperties(mapOf("min" to -1.0, "max" to 1.0))
//        .entry
//    val pitchVelocityEntry = autoTab.add("Pitch Velocity", 0.0)
//        .withWidget(BuiltInWidgets.kGraph)
//        .withProperties(mapOf("min" to -1.0, "max" to 1.0))
//        .entry
    val currentPitchHeading: Double
        get() = atan(tan(drivetrain.gyro.pitch) / Math.cos(drivetrain.gyro.roll))
    val debouncer = Debouncer(0.05)
    override fun execute() {
        val currentDrivetrainGlobalPitch = drivetrainGlobalPitch
        globalPitchVelocity =
            (currentDrivetrainGlobalPitch - lastDrivetrainGlobalPitch) /
                    (Timer.getFPGATimestamp() - lastTime)

        drivetrain.drive(
            ChassisSpeeds(
                if (currentPitchHeading > 0) -0.2 else .2,
                0.0,
                0.0
            ),
            true
        )

//        currentPitchHeadingEntry.setDouble(currentPitchHeading)
//        pitchHeadingEntry.setDouble(currentPitchHeading)
//        pitchVelocityEntry.setDouble(globalPitchVelocity)
    }

    override fun isFinished(): Boolean =
        debouncer.calculate(globalPitchVelocity.absoluteValue > 0.05)
}