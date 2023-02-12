package frc.robot.commands

// import junit library

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test


class MoveToPositionTest {
    lateinit var drivetrain: TestableDrivetrain
    var controlScheme = TestableControlScheme()

    @BeforeEach
    fun setup() {
        // clear network tables
        NetworkTableInstance.getDefault().flush()
        NetworkTableInstance.getDefault().flushLocal()
        NetworkTableInstance.getDefault().stopServer()
        NetworkTableInstance.getDefault().stopDSClient()
        NetworkTableInstance.getDefault().stopClient()
        NetworkTableInstance.getDefault().stopLocal()

        // clear/shutdown shuffleboard
        Shuffleboard.disableActuatorWidgets()
        // create a drivetrain and control scheme
        controlScheme = TestableControlScheme()
        drivetrain = TestableDrivetrain(controlScheme)
    }

    @Test
    fun testMoveToPosition() {
        val command = MoveToPosition(drivetrain, 0.0, 0.0, 0.0)
        command.initialize()
        // set the drivetrain's pose to something other than the origin
        drivetrain.kPose = Pose2d(1.0, 1.0, Rotation2d())
        // run the command for 1 second
        for (i in 0..100) {
            command.execute()
            drivetrain.updatePose(0.01)
        }
        // check that the drivetrain's pose is close to the origin
        assertEquals(0.0, drivetrain.estimatedPose2d.x, 0.1)
        assertEquals(0.0, drivetrain.estimatedPose2d.y, 0.1)
        assertEquals(0.0, drivetrain.estimatedPose2d.rotation.radians, 0.1)
    }

    @Test
    fun testMoveToPosition2() {
        val command = MoveToPosition(drivetrain, 12.0, 1.0, 0.0)
        command.initialize()
        // set the drivetrain's pose to something other than the origin
        drivetrain.kPose = Pose2d(0.0, 0.0, Rotation2d(170.0))
        // run the command for 1 second
        for (i in 0..100) {
            command.execute()
            drivetrain.updatePose(0.01)
            drivetrain.lastChassisSpeeds?.let {
                println("vx: ${it.vxMetersPerSecond}, vy: ${it.vyMetersPerSecond}, omega: ${it.omegaRadiansPerSecond}")
            }
            drivetrain.lastFieldOriented?.let {
                println("field oriented: $it")
            }
        }
        // check that the drivetrain's pose is close to the origin
        assertEquals(12.0, drivetrain.estimatedPose2d.x, 0.1)
        assertEquals(1.0, drivetrain.estimatedPose2d.y, 0.1)
        assertEquals(0.0, drivetrain.estimatedPose2d.rotation.radians, 0.1)
    }
}
