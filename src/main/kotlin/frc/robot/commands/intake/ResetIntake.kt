import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake

class SetConePosition(
    val intake: Intake
) : CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun execute() {
        //@TODO Set the right position using shuffleboard details
        //intake.setIntakePosition(idle intake position)
    }

    override fun end(interrupted: Boolean) {
        intake.systemMotorPercentage = 0.0
    }

    override fun isFinished(): Boolean {
        return intake.limitSwitchPressed
    }
}