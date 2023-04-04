//package frc.robot.commands.intake
//
//import edu.wpi.first.wpilibj.Timer
//import edu.wpi.first.wpilibj2.command.CommandBase
//import frc.robot.RobotContainer
//import frc.robot.constants.intake
//import frc.robot.subsystems.Intake
//
//class ManualZeroIntake(private val robotContainer: RobotContainer): CommandBase() {
//
////    val timer = Timer()
//    init {
//        addRequirements(robotContainer.intake)
//    }
//
////    override fun initialize() {
////        timer.restart()
////        robotContainer.intake.modeZeroed = false
////        robotContainer.intake.setDeployAngle(robotContainer.intake.deployPosition)
////    }
//    override fun execute() {
//        robotContainer.intake.modeVoltage = -3.0
//        robotContainer.intake.setModeAngle(-1.7)
//    }
//
//    override fun end(interrupted: Boolean) {
//        robotContainer.intake.zeroModeMotor()
//        robotContainer.intake.modeVoltage = 0.0
//    }
//
//    override fun isFinished() = intake.an
//
//}