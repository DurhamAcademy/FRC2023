package frc.robot.commands.alltogether

import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Manipulator
import frc.robot.subsystems.Wrist
import frc.robot.utils.Solver
import kotlin.math.PI

class IntakePositionForeward(
    private val elevator: Elevator,
    private val arm: Arm,
    private val wrist: Wrist,
) : CommandBase() {
    fun withManipulator(manipulator: Manipulator) =
        this
            .raceWith(CollectObject(manipulator))

    init {
        addRequirements(elevator, arm, wrist)
    }

    override fun execute() {
        val armAngle = degreesToRadians(125.0)
        wrist.setpoint
        arm.setArmPosition(armAngle)
        Solver.getWristPose(0.0, armAngle, armAngle - PI / 2)
        elevator.setpoint = -Solver.armCoords[2] + 0.075
        wrist.setPosition(wrist.levelAngle(30.0))
    }
}