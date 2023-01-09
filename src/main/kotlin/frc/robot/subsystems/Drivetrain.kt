package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.CANCoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints

class Drivetrain : SubsystemBase() {
    class SwerveModule(
        driveMotorId: Int,
        turnMotorId: Int,
        encoderId: Int,
    ) {
        private val driveMotor = WPI_TalonFX(driveMotorId)
        private val turnMotor = WPI_TalonFX(turnMotorId)
        private val turnEncoder = CANCoder(encoderId)

        //should adjust these gains or characterize since they are a little slow
        val DRIVE_P = 0.1
        val DRIVE_I = 0.0
        val DRIVE_D = 0.0
        val drivePid = ProfiledPIDController(
            DRIVE_P,
            DRIVE_I,
            DRIVE_D,
            Constraints(2.0,2.0 )// TODO: Fix these
        )
        //should adjust these gains or characterize since they are a little slow
        val ANGLE_P = 0.01
        val ANGLE_I = 0.0
        val ANGLE_D = 0.0
        val anglePid = ProfiledPIDController(
            ANGLE_P,
            ANGLE_I,
            ANGLE_D,
            Constraints(
                3.0,
                3.0
            )// TODO: Fix these
        ).apply {
            enableContinuousInput(-Math.PI, Math.PI)
        }

        var setpoint = SwerveModuleState()

        fun move() {

        }


        fun periodic() {

        }
    }
}