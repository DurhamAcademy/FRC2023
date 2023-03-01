package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Manipulator

class CollectObject(
    val manipulator: Manipulator
) : CommandBase() {
    constructor(robotContainer: RobotContainer) : this(robotContainer.manipulator)
    init {
        addRequirements(manipulator)
    }

    override fun initialize() {

    }

    override fun execute() {
        if (manipulator.inColorRange == true) {//
            manipulator.motorPercentage = 0.005
            manipulator.isOpen = false
        } else if (manipulator.sensorConnected) {
            manipulator.motorPercentage = 0.5
            // close manipulator if distance is lower than 0.05
            manipulator.isOpen = true
        } else {
            manipulator.motorPercentage = 0.05
        }
    }

    override fun end(interrupted: Boolean) {
        manipulator.motorPercentage = 0.0
    }

    override fun isFinished(): Boolean = false
}