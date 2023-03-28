package frc.robot.commands.balance

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Game
import frc.robot.subsystems.Drivetrain

class AutoBalance(private val drivetrain: Drivetrain) : CommandBase() {

    init {
        addRequirements(drivetrain)
    }

    private var gravityVector = DoubleArray(3)

    private val balancePid = PIDController(1.5, 0.00, 0.002)

    private val xTilt: Double // tilt along the x-axis of the field, in Gs
        get() {
            drivetrain.gyro.getGravityVector(gravityVector) // get the gravity vector of the pigeon
            // flatten the gravity vector to x/y, this is robot-relative tilt on each axis
            val tiltVec = Translation2d(gravityVector[0], gravityVector[1])
            // rotate the tilt vector into field space
            return tiltVec.rotateBy(drivetrain.estimatedPose2d.rotation).x
        }

    private val timer = Timer()

    override fun initialize() {
        timer.start()
    }

    override fun execute() {

        val speedMul = when (Game.alliance) {
            DriverStation.Alliance.Red -> -1.0
            DriverStation.Alliance.Blue -> 1.0
            DriverStation.Alliance.Invalid -> 0.0
        }

        if (timer.hasElapsed(0.3)) {
            drivetrain.drive(
                ChassisSpeeds(-balancePid.calculate(MathUtil.applyDeadband(xTilt, .03), 0.0), 0.0, 0.0),
                true
            )
        } else {
            drivetrain.drive(ChassisSpeeds(2.0 * speedMul, 0.0, 0.0), true)
        }


//        val correctiveFactor = MathUtil.applyDeadband(xTilt, 0.05)
        SmartDashboard.putNumber("xTilt", xTilt)
//        SmartDashboard.putNumber("dTilt", dTilt)       
//        SmartDashboard.putBoolean("level", isLevel)

    }

    override fun isFinished() = false

}