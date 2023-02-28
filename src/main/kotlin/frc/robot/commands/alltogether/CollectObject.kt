package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Manipulator

class CollectObject(
    private val manipulator: Manipulator,
) : CommandBase() {
    init {
        addRequirements(manipulator)
    }

    override fun execute() {
        if (manipulator.inColorRange == true) {
            manipulator.motorPercentage = 0.005
        } else if (manipulator.sensorConnected) {
            manipulator.motorPercentage = 0.1 + 0.05 * manipulator.distance!! * 100
            // close manipulator if distance is lower than 0.05
            if (manipulator.distance!! < 0.05) {
                manipulator.motorPercentage = 0.5
            }
        } else {
            manipulator.motorPercentage = 0.1
        }
    }

    override fun end(interrupted: Boolean) {
        manipulator.motorPercentage = 0.0
    }

    override fun isFinished(): Boolean = false
}