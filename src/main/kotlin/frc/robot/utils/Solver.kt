package frc.robot.utils

import frc.robot.Constants
import frc.robot.RobotContainer
import kotlin.math.cos
import kotlin.math.sin

object Solver {
    var armLength = Constants.arm.length
    var wristLength = Constants.manipulator.wristToObj
    var robotContainer: RobotContainer? = null
    var wristCoords = ArrayList<Double>(4).apply { for (i in 0..3) this.add(0.0) }
    var armCoords = ArrayList<Double>(4).apply { for (i in 0..3) this.add(0.0) }

    /**
     * Returns the wrist pose given the elevator height, arm angle, and wrist
     * angle.
     * @param elevatorHeight The height of the elevator in meters.
     * @param armAngle The angle of the arm in radians along the y normal plane,
     * where 0 is vertical, pi/2 is horizontal, and -pi/2 is horizontal.
     * @param wristAngle The angle of the wrist in radians along the y normal
     * plane, where 0 is parallel to the arm, pi/2 is perpendicular to the arm,
     * and -pi/2 is perpendicular to the arm.
     * @return The wrist pose in a list of x, y, z, and pitch.
     */
    fun getWristPose(
        elevatorHeight: Double,
        armAngle: Double,
    ) {
        val armX = armLength * cos(armAngle)
        val armY = 0.0
        val armZ = elevatorHeight + armLength * sin(armAngle)
        val wristPitch = armAngle
        wristCoords[0] = armX + wristLength * cos(wristPitch)
        wristCoords[1] = armY
        wristCoords[2] = armZ + wristLength * sin(wristPitch)
        wristCoords[3] = wristPitch

        armCoords[0] = armX
        armCoords[1] = armY
        armCoords[2] = armZ
        armCoords[3] = armAngle
    }

    fun getWristPose() {
        if (robotContainer == null) {
            throw Exception("RobotContainer is null!")
        }
        getWristPose(
            robotContainer!!.elevator.height,
            robotContainer!!.arm.armPosition,
        )
    }
}