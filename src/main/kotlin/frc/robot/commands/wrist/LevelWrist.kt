package frc.robot.commands.wrist

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Wrist
import java.lang.Math.toRadians

class LevelWrist(
    private val wrist: Wrist,
    private val arm: Arm,
    val angleOfFlip: Double = toRadians(20.0),
) : CommandBase() {
    init {
        addRequirements(wrist)
    }
    override fun execute() {
        wrist.setPosition(wrist.levelAngle(angleOfFlip))
    }

    override fun isFinished() = false
}