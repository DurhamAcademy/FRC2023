package frc.robot.commands.pathing

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.kyberlib.command.Game
import frc.robot.RobotContainer
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.SetSubsystemPosition
import frc.robot.commands.drivetrain.AutoBalance
import frc.robot.commands.manipulator.Throw
import frc.robot.constants.manipulator
import frc.robot.subsystems.Arm
import frc.robot.utils.GamePiece
import frc.robot.utils.grid.PlacementLevel
import kotlin.math.PI

class AutoPlaceAndBalance(
    private val robotContainer: RobotContainer,
): Auto {

        override fun getCommand(): Command {
            return when(Game.alliance){
                DriverStation.Alliance.Red -> pathRed()
                DriverStation.Alliance.Blue -> pathBlue()
                DriverStation.Alliance.Invalid -> Commands.print("No alliance")
            }
        }
        private fun pathRed(): Command {
            return MoveToPosition(robotContainer.drivetrain, 14.67, 2.72, 180.0).withTimeout(1.0)
                .andThen(SetSubsystemPosition(robotContainer, { IOLevel.High }, { GamePiece.cube },false)) // get in position to place cube on L3 middle
                .andThen(Throw(robotContainer.manipulator, { GamePiece.cube }, {PlacementLevel.Level3})) // shoot cube
                .andThen(MoveToPosition(robotContainer.drivetrain, 14.34, 2.72, 180.0)) // back up and turn 45
                .andThen(MoveToPosition(robotContainer.drivetrain, 14.34, 2.72, 45.0)) // back up and turn 45
                .andThen(Commands.runOnce({ robotContainer.arm.setArmPosition(-PI /2) })) // move the arm to horizontal
                .andThen(Commands.waitUntil { robotContainer.arm.armPID.atGoal() })
                .andThen(AutoBalance(robotContainer.drivetrain))
        }
        private fun pathBlue(): Command {
            return MoveToPosition(robotContainer.drivetrain, 16.52-14.67, 2.72, 0.0).withTimeout(1.0)
                .andThen(SetSubsystemPosition(robotContainer, { IOLevel.High }, { GamePiece.cube },false)) // get in position to place cube on L3 middle
                .andThen(Throw(robotContainer.manipulator, { GamePiece.cube }, {PlacementLevel.Level3})) // shoot cube
                .andThen(MoveToPosition(robotContainer.drivetrain, 16.52-14.34, 2.72, 0.0)) // back up and turn 45
                .andThen(MoveToPosition(robotContainer.drivetrain, 16.52-14.34, 2.72, 45.0)) // back up and turn 45
                .andThen(AutoBalance(robotContainer.drivetrain))
                .andThen(Commands.runOnce({ robotContainer.arm.setArmPosition(-PI /2) })) // move the arm to horizontal
                .andThen(Commands.waitUntil { robotContainer.arm.armPID.atGoal() })
                .andThen(AutoBalance(robotContainer.drivetrain))
        }
}