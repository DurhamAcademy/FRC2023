//package frc.robot.subsystems
//
//import edu.wpi.first.math.controller.PIDController
//import edu.wpi.first.math.controller.ProfiledPIDController
//import edu.wpi.first.math.controller.SimpleMotorFeedforward
//import edu.wpi.first.math.trajectory.TrapezoidProfile
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
//import edu.wpi.first.wpilibj.motorcontrol.Spark
//import edu.wpi.first.wpilibj2.command.SubsystemBase
//import frc.robot.Constants
//import frc.robot.controls.ControlScheme
//
//class Arm(
//    var controlScheme: ControlScheme,
//) : SubsystemBase() {
//    var armMotor = PWMSparkMax(0)
//    var armPID = ProfiledPIDController(
//        Constants.Arm.armMotor.PID.kP,
//        Constants.Arm.armMotor.PID.kI,
//        Constants.Arm.armMotor.PID.kD,
//        TrapezoidProfile.Constraints(
//            Constants.Arm.armMotor.PID.TrapezoidProfile.maxVelocity,
//            Constants.Arm.armMotor.PID.TrapezoidProfile.maxAcceleration
//        )
//    )
//    var armFeedforward = SimpleMotorFeedforward(
//        Constants.Arm.armMotor.FeedForeward.kS,
//        Constants.Arm.armMotor.FeedForeward.kV,
//        Constants.Arm.armMotor.FeedForeward.kA
//    )
////    var armPosition: Double
////        get() = armMotor.encoder.position
////        set(value) {
////            armPID.goal = TrapezoidProfile.State(value, 0.0)
////        }
////    var armVelocity: Double
//}