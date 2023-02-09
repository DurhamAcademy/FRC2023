package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.ElevatorTestDown
import frc.robot.commands.ElevatorTestUp
import frc.robot.controls.ControlScheme
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator

class RobotContainer {
    val xbox = CommandXboxController(0)
    val controlScheme: ControlScheme = DefaultControlScheme(xbox)

    @Suppress("unused")
    val drivetrain = Drivetrain(controlScheme)
    val elevator = Elevator(controlScheme)
    init {
        controlScheme.run {
            xbox!!.a().onTrue(ElevatorTestUp(elevator, controlScheme))
            xbox!!.b().onTrue(ElevatorTestDown(elevator, controlScheme))
        }
    }
}