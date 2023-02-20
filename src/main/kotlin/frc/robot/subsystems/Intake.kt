package frc.robot.subsystems
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake(

): SubsystemBase() {
    private val deployMotor = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val spinningMotor = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)
}
