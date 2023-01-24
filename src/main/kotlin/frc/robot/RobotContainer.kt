// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.AlignRobot
import frc.robot.commands.SwerveJoystickCmd
import frc.robot.subsystems.SwerveSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private val driver = XboxController(0)
    private val swerveSubsystem = SwerveSubsystem()
    //private final SwerveJoystickCmd m_autoCommand = new SwerveJoystickCmd(swerveSubsystem);
    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        val fieldRelative = false
        swerveSubsystem.defaultCommand = SwerveJoystickCmd(swerveSubsystem, driver, fieldRelative)

        // Configure the button bindings
        configureButtonBindings()
        kA!!.onTrue(AlignRobot(swerveSubsystem, SwerveSubsystem.Companion.tracker, fieldRelative))
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
        kA = JoystickButton(driver, 1)
    }

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = SequentialCommandGroup(InstantCommand({}, swerveSubsystem))

    companion object {
        var kA: JoystickButton? = null
    }
}