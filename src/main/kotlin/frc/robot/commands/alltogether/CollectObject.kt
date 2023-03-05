package frc.robot.commands.alltogether

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
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
        if (debounce.calculate(manipulator.inColorRange == true)) {//
            manipulator.motorPercentage = 0.1
            manipulator.isOpen = false
        } else if (manipulator.sensorConnected) {
            manipulator.motorPercentage = 1.0
            // close manipulator if distance is lower than 0.05
            manipulator.isOpen = true
        } else {
            manipulator.isOpen = true
            manipulator.motorPercentage = 1.0
        }
    }

    override fun end(interrupted: Boolean) {
        if (!manipulator.sensorConnected)
            manipulator.isOpen =false
        manipulator.motorPercentage = 0.25
        0
    }

    override fun isFinished(): Boolean = false
}