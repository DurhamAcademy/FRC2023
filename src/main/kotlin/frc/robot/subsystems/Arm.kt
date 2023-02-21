package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.controls.ControlScheme

class Arm(
    val controlScheme: ControlScheme,
) : SubsystemBase() {
    val armMotor = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless)
}