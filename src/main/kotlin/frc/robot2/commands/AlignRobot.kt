// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot2.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot2.subsystems.SwerveSubsystem
import org.photonvision.PhotonCamera

class AlignRobot(
    private val swerve: SwerveSubsystem,
    private val tracker: PhotonCamera,
    private val fieldRelative: Boolean,
) : CommandBase() {
    private val turn = PIDController(0.03, 0.0, 12.0)
    private var rotationSpeed = 0.0
    private val translation = Translation2d(0.0, 0.0)
    private val rotationLimit = SlewRateLimiter(0.5)

    /** Creates a new AlignRobot.  */
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val result = tracker.latestResult
        if (result.hasTargets()) {
            //rotationSpeed = turn.calculate(result.getBestTarget().getYaw(), 0);
            rotationSpeed = rotationLimit.calculate(turn.calculate(result.bestTarget.yaw, 0.0))
            println("Rotation Speed: $rotationSpeed")
        }
        // else{
        //   rotationSpeed = 0;
        // }
        swerve.drive(translation, rotationSpeed, fieldRelative)
        if (result.bestTarget.yaw == 0.0) {
            end(true)
        }
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}