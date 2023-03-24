package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.commands.balance.AutoBalance
import kotlin.math.PI

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
    @Suppress("unused")
    lateinit var robotContainer: RobotContainer

    override fun robotInit() {
        robotContainer = RobotContainer()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()

        robotContainer.update()
    }

    var auto: Command? = null

    override fun autonomousInit() {
        auto = robotContainer.auto
//        auto = Commands.runOnce({ robotContainer.arm.setArmPosition(-PI /2) }) // move the arm to horizontal
//                .andThen(Commands.waitUntil { robotContainer.arm.armPID.atGoal() })
//                .andThen(AutoBalance(robotContainer.drivetrain))
        auto?.schedule()
    }

    override fun autonomousExit() {
        auto?.cancel()
    }
}
