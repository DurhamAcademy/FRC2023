package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.pathing.selectAuto

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
        selectAuto()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
//        robotContainer.cameraWrapper.getEstimatedGlobalPose(Pose2d()).ifPresent {
//            SmartDashboard.putNumber("tagX", it.estimatedPose.x)
//        }
//        SmartDashboard.
        robotContainer.drivetrain.poseEstimator.estimatedPosition.run {
            SmartDashboard.putNumber("poseRotation", rotation.rotations)
            SmartDashboard.putNumber("poseX", x)
            SmartDashboard.putNumber("tagX", y)
        }

        robotContainer.update()
    }

    // schedule test commands during test mode
    override fun testInit() {

    }

    override fun testPeriodic() {

    }
    var auto: Command? = null

    override fun autonomousInit() {
        if(SmartDashboard.getBoolean("auto1", false))
            auto = robotContainer.auto1
        else if(SmartDashboard.getBoolean("auto2", false) and SmartDashboard.getBoolean("high", false))
            auto = robotContainer.auto2high
        else if(SmartDashboard.getBoolean("auto2", false) and SmartDashboard.getBoolean("low", false))
            auto = robotContainer.auto2low
        else if(SmartDashboard.getBoolean("auto3", false) and SmartDashboard.getBoolean("high", false))
            auto = robotContainer.auto3high
        else if(SmartDashboard.getBoolean("auto3", false) and SmartDashboard.getBoolean("low", false))
            auto = robotContainer.auto3low
        else if(SmartDashboard.getBoolean("auto4", false) and SmartDashboard.getBoolean("high", false))
            auto = robotContainer.auto4high
        else if(SmartDashboard.getBoolean("auto4", false) and SmartDashboard.getBoolean("low", false))
            auto = robotContainer.auto4low
        else if(SmartDashboard.getBoolean("auto5", false) and SmartDashboard.getBoolean("mid", false))
            auto = robotContainer.auto5
        else if(SmartDashboard.getBoolean("auto6", false) and SmartDashboard.getBoolean("high", false))
            auto = robotContainer.auto6high
        else if(SmartDashboard.getBoolean("auto6", false) and SmartDashboard.getBoolean("low", false))
            auto = robotContainer.auto6low
        else if(SmartDashboard.getBoolean("auto7", false) and SmartDashboard.getBoolean("high", false))
            auto = robotContainer.auto7
        else
            auto = robotContainer.auto1
        auto?.schedule()
    }

    override fun autonomousExit() {
        auto?.cancel()
    }
}
