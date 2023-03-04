package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

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
        auto = robotContainer.auto
//        auto?.schedule()
    }

    override fun autonomousExit() {
//        auto?.cancel()
    }
}
