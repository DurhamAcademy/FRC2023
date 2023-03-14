package frc.robot.commands.alltogether

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.commands.SetManipulatorSpeed
import frc.robot.subsystems.Manipulator

class CollectObject(
    val manipulator: Manipulator
) : CommandBase() {

    private val debounce = Debouncer(0.5, Debouncer.DebounceType.kFalling)
    constructor(robotContainer: RobotContainer) : this(robotContainer.manipulator)
    init {
        addRequirements(manipulator)
    }

    override fun initialize() {

    }

    override fun execute() {
        SetManipulatorSpeed(manipulator, 1.0)
    }

    override fun end(interrupted: Boolean) {
        SetManipulatorSpeed(manipulator, .1)
    }

    override fun isFinished(): Boolean = false
}