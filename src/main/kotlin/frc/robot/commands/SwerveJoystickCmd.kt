// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants.chassisConstants
import frc.robot.subsystems.SwerveSubsystem

/** An example command that uses an example subsystem.  */
class SwerveJoystickCmd(
    private val swerve: SwerveSubsystem,
    private val driver: XboxController,
    private val fieldRelative: Boolean,
) :
    CommandBase() {
    private var rotation = 0.0
    private var translation: Translation2d? = null
    private val overrideJS = false
    private val yLim = SlewRateLimiter(3.0)
    private val xLim = SlewRateLimiter(1.0)
    private val rotLim = SlewRateLimiter(1.0)

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        var yAxis = -driver.leftY
        var xAxis = -driver.leftX
        var rotAxis = -driver.rightX
        yAxis = if (Math.abs(yAxis) < chassisConstants.deadband) 0.0 else yLim.calculate(yAxis * 0.3)
        xAxis = if (Math.abs(xAxis) < chassisConstants.deadband) 0.0 else xLim.calculate(xAxis * 0.3)
        rotAxis = if (Math.abs(rotAxis) < chassisConstants.deadband) 0.0 else rotLim.calculate(rotAxis * 0.3)
        translation = Translation2d(yAxis, xAxis).times(chassisConstants.maxSpeedMPS)
        rotation = rotAxis * chassisConstants.maxTurnSpeed
        swerve.drive(translation, rotAxis, fieldRelative)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}