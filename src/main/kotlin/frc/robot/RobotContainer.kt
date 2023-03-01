package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.kyberlib.command.Game
import frc.kyberlib.lighting.KLEDRegion
import frc.kyberlib.lighting.KLEDStrip
import frc.kyberlib.lighting.animations.*
import frc.kyberlib.math.kEpsilon
import frc.kyberlib.math.units.extensions.feetPerSecond
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.seconds
import frc.robot.commands.ElevatorTestDown
import frc.robot.commands.ElevatorTestUp
import frc.robot.commands.MoveToPosition
import frc.robot.commands.alltogether.*
import frc.robot.commands.arm.SetArmToAngle
import frc.robot.commands.manipulator.CloseManipulator
import frc.robot.commands.manipulator.OpenManipulator
import frc.robot.commands.manipulator.SetManipulatorSpeed
import frc.robot.commands.wrist.LevelWrist
import frc.robot.commands.wrist.SetWristAngle
import frc.robot.controls.BryanControlScheme
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.*
import frc.robot.utils.Solver
import java.awt.Color
import java.lang.Math.toRadians
import kotlin.math.PI

class RobotContainer {
    val xbox = CommandXboxController(0)
    val controlScheme: ControlScheme = BryanControlScheme()//xbox)

//    var cameraWrapper: PhotonCameraWrapper = TODO("camera not working")//PhotonCameraWrapper()
    val manipulator = Manipulator()

    init {
        Solver.robotContainer = this
    }

    val drivetrain = Drivetrain(controlScheme, cameraWrappers = listOf(/*cameraWrapper*/))
    val elevator = Elevator(controlScheme, this)
    val arm = Arm()
    val wrist = Wrist(arm).apply {
        defaultCommand = LevelWrist(
            this, arm, toRadians(60.0)
        )
    }

    init {
        controlScheme.run {
            xbox!!.a().whileTrue(ElevatorTestUp(elevator))
            xbox!!.b().whileTrue(ElevatorTestDown(elevator))
            // assign the go to april tag 1 trigger to the command that
            // moves the robot to the april tag
            testGoToAprilTag1
                .whileTrue(
                    MoveToPosition(drivetrain, 14.5, 1.0, 0.0)
                )

            // assign the go-to zero zero trigger to the command that
            // moves the robot to (0, 0)
            testGoToZeroZero
                .whileTrue(
                    MoveToPosition(drivetrain, 0.0, 0.0, 0.0)
                )

            // assign the arm 90 trigger to the command that
            // moves the arm to 90 degrees
            testArm90
                .whileTrue(
                    SetArmToAngle(arm, PI / 2)
                )
            testArm0
                .whileTrue(
                    SetArmToAngle(arm, 0.0)
                )
            testArmNeg90
                .whileTrue(
                    SetArmToAngle(arm, -PI / 2)
                )

            // assign the wrist 90 trigger to the command that
            // moves the wrist to 90 degrees
            testWrist90
                .whileTrue(
                    SetWristAngle(wrist, PI / 2)
                )
            testWrist0
                .whileTrue(
//                    OpenManipulator(manipulator)
//                        .andThen(
                    SetManipulatorSpeed(manipulator, -0.1)//)
                )
            testWristNeg90
                .whileTrue(
                    SetWristAngle(wrist, -PI / 2)
                )

            // assign the open manipulator trigger to the command that
            // opens the manipulator
            openManipulator
                .onTrue(
                    OpenManipulator(manipulator)
                )

            // assign the close manipulator trigger to the command that
            // closes the manipulator
            closeManipulator
                .onTrue(CloseManipulator(manipulator))

            toggleManipulator
                .toggleOnFalse(
                    CloseManipulator(manipulator)
                ).toggleOnTrue(
                    OpenManipulator(manipulator)
                )

            // assign the grab cone trigger to the command that
            // grabs a cone

            // assign the hold cone trigger to the command that
            // holds a cone

            // idle
            idleConfiguration
                .whileTrue(SetPosition.idle(this@RobotContainer))
                .onFalse(HoldPosition(elevator, arm, wrist))

            // assign l1
            placeLvl1
                .whileTrue(
                    SetPosition.setpoint(PlacePoint.Level1, this@RobotContainer)
                )

            // assign l2
            placeLvl2
                .whileTrue(
                    SetPosition.setpoint(PlacePoint.Level2, this@RobotContainer)
                ).onFalse(HoldPosition(elevator, arm, wrist))

            // assign l3
            placeLvl3
                .whileTrue(
                    SetPosition.setpoint(PlacePoint.Level3, this@RobotContainer)
                ).onFalse(HoldPosition(elevator, arm, wrist))

            // assign intake
            intake
                .whileTrue(
                    IntakePositionForeward(elevator, arm, wrist)
                        .withManipulator(manipulator)
                ).onFalse(HoldPosition(elevator, arm, wrist))

            // assign outtake to set manipulator speed to -0.5
            outtake
                .whileTrue(SetManipulatorSpeed(manipulator, -0.5)).onFalse(SetManipulatorSpeed(manipulator, 0.0))
        }

    }
    val leds = KLEDStrip(0, Constants.leds.count).apply {
        val coral = Color(255, 93, 115)
        val allianceColor = if (Game.alliance == DriverStation.Alliance.Red) coral else Color.CYAN

        // idle alliance animations
        val prematchArms = AnimationRGBFade(7.seconds)//AnimationRGBWave(1.0, .1.seconds)
        val idleArms = AnimationSolid(Color.BLACK) { Game.enabled }
        val idleCylon = AnimationSparkle(allianceColor, false) { Game.enabled }

        val chain = KLEDRegion(0, Constants.leds.count,
            prematchArms, idleArms, idleCylon
        )
        this += (chain)
    }
    fun update() {
        leds.update()
    }
}
