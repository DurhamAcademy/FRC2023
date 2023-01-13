package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.BLDriveMotorId
import frc.robot.Constants.BLTurnEncoderId
import frc.robot.Constants.BLTurnMotorId
import frc.robot.Constants.BRDriveMotorId
import frc.robot.Constants.BRTurnEncoderId
import frc.robot.Constants.BRTurnMotorId
import frc.robot.Constants.FLDriveMotorId
import frc.robot.Constants.FLTurnEncoderId
import frc.robot.Constants.FLTurnMotorId
import frc.robot.Constants.FRDriveMotorId
import frc.robot.Constants.FRTurnEncoderId
import frc.robot.Constants.FRTurnMotorId
import frc.robot.commands.DriveCommand
import frc.robot.controls.ControlScheme

class Drivetrain(
    controlScheme: ControlScheme,
) : SubsystemBase() {
    init {
        defaultCommand = DriveCommand(this, controlScheme)
    }

    val FL = SwerveModule(FLDriveMotorId, FLTurnMotorId, FLTurnEncoderId, Translation2d()) // FIXME: add actual positon
    val FR = SwerveModule(FRDriveMotorId, FRTurnMotorId, FRTurnEncoderId, Translation2d()) // FIXME: add actual positon
    val BL = SwerveModule(BLDriveMotorId, BLTurnMotorId, BLTurnEncoderId, Translation2d()) // FIXME: add actual positon
    val BR = SwerveModule(BRDriveMotorId, BRTurnMotorId, BRTurnEncoderId, Translation2d()) // FIXME: add actual positon
    val modules = listOf(FL, FR, BL, BR)
    val kinematics = SwerveDriveKinematics(
        *modules.map { it.position }.toTypedArray()
    )

    fun drive(chassisSpeeds: ChassisSpeeds) {
        val goals: Array<SwerveModuleState> = kinematics.toSwerveModuleStates(chassisSpeeds)
        modules[0].setpoint = goals[0]
        modules[1].setpoint = goals[1]
        for (i in goals.indices) {
            modules[i].setpoint = goals[i]
        }
        goals.forEachIndexed { index, goal ->
            modules[index].setpoint = goal
        }
    }
}

