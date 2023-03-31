package frc.robot.commands.balance

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

class BalanceDrive(
    val drivetrain: Drivetrain
) : CommandBase() {


    enum class Phase {
        /** Drive foreward quickly to get the wheels on the ramp, even if not
         * totally on the ramp yet */
        Getup,
        /**
        MoveUp,
        STOP
         */
    }

    init {
        addRequirements(drivetrain)
    }

    val pitch: Double
        get() = drivetrain.gyro.pitch
    val roll: Double
        get() = drivetrain.gyro.roll

    val heading: Rotation2d
        get() = drivetrain.estimatedPose2d.rotation

    private var phaseStart = Timer.getFPGATimestamp()
    val phaseTime: Double
        get() = Timer.getFPGATimestamp() - phaseStart

    private var phase = Phase.Getup

    override fun initialize() {
        phaseStart = Timer.getFPGATimestamp()
        phase = Phase.Getup
    }

//    override fun execute() {
//        when (phase) {
//            Phase.Getup -> {
//                if (pitch.absoluteValue < 12 || roll.absoluteValue < 12) {
//                    phase = Phase.MoveUp
//                    phaseStart = Timer.getFPGATimestamp()
//                }
}