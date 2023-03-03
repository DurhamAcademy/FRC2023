package frc.robot.commands.manipulator

import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Constants
import frc.robot.controls.BryanControlScheme
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Manipulator

class OpenManipulator(
    private val manipulator: Manipulator,
) :InstantCommand() {
    init {
        addRequirements(manipulator)
    }

    override fun execute() {
        manipulator.isOpen = true
    }
}