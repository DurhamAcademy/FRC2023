package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.intake.ZeroModeMotor

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
    lateinit var robotContainer: RobotContainer

    override fun robotInit() {
        robotContainer = RobotContainer()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()

        robotContainer.update()

//        if (DriverStation.getMatchTime() <= .25 && Game.OPERATED) {
//            DriveCommand(robotContainer.drivetrain, rotation = { 0.0001 })
//                .until {
//                    DriverStation.getMatchTime() > 0.25
//                }
//        }
    }

    override fun teleopInit() {
        ZeroModeMotor(robotContainer).schedule()
    }

    var auto: Command? = null
    var defaultCommandHolder: Command? = null

    override fun autonomousInit() {
        defaultCommandHolder = robotContainer.intake.defaultCommand
        robotContainer.intake.defaultCommand = null
        auto = robotContainer.auto
//        auto = Commands.runOnce({ robotContainer.arm.setArmPosition(-PI /2) }) // move the arm to horizontal
//                .andThen(Commands.waitUntil { robotContainer.arm.armPID.atGoal() })
//                .andThen(AutoBalance(robotContainer.drivetrain))
        auto?.schedule()
    }

    override fun autonomousExit() {
        auto?.cancel()

        robotContainer.intake.defaultCommand = defaultCommandHolder
        defaultCommandHolder = robotContainer.intake.defaultCommand
    }

    override fun disabledExit() {
//        ZeroModeMotor(robotContainer.intake).schedule()
    }
}
