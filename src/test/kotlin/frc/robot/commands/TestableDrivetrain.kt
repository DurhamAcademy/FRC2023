package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Drivetrain

class TestableDrivetrain(controlScheme: ControlScheme) : Drivetrain(controlScheme, listOf()) {
    var lastChassisSpeeds: ChassisSpeeds? = null
    var lastFieldOriented: Boolean? = null

    var kPose = Pose2d()
    override val estimatedPose2d: Pose2d
        get() = super.estimatedPose2d

    override fun drive(chassisSpeeds: ChassisSpeeds, fieldOriented: Boolean) {
        lastChassisSpeeds = chassisSpeeds
        lastFieldOriented = fieldOriented
    }

    fun updatePose(dt: Double) {
        lastChassisSpeeds?.let {
            val simSpeed = Translation2d(it.vxMetersPerSecond, it.vyMetersPerSecond).times(dt)
            val simRotation = Rotation2d(it.omegaRadiansPerSecond * dt)
            val T = Transform2d(simSpeed, simRotation)
            kPose = kPose.plus(T)
        }
    }
}
