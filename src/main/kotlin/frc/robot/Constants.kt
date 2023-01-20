package frc.robot

import kotlin.math.PI

object Constants {
    const val gyroReversed = false
    const val FLDriveMotorId = 10
    const val FRDriveMotorId = 11
    const val BLDriveMotorId = 12
    const val BRDriveMotorId = 13

    const val FLTurnMotorId = 14
    const val FRTurnMotorId = 15
    const val BLTurnMotorId = 16
    const val BRTurnMotorId = 17

    const val FLTurnEncoderId = 6
    const val FRTurnEncoderId = 7
    const val BLTurnEncoderId = 8
    const val BRTurnEncoderId = 9

    const val WHEEL_RADIUS = .0508
    val WHEEL_CIRCUMFRENCE = WHEEL_RADIUS * 2 * PI
    const val DRIVE_GEAR_RATIO = 6.75
}