package frc.robot.commands.pathing

import edu.wpi.first.wpilibj2.command.Command

interface Auto {
    fun getCommand(): Command
}