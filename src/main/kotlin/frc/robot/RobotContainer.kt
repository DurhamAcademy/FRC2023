package frc.robot

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.kyberlib.command.Game
import frc.kyberlib.lighting.KLEDRegion
import frc.kyberlib.lighting.KLEDStrip
import frc.kyberlib.lighting.animations.*
import frc.kyberlib.math.units.extensions.seconds
import frc.robot.commands.ElevatorTestDown
import frc.robot.commands.ElevatorTestUp
import frc.robot.commands.pathing.MoveToPosition
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
    val controlSchemeA: ControlScheme = BryanControlScheme(0)//xbox)
    val controlSchemeB: ControlScheme = BryanControlScheme(1)//xbox)

    var cameraWrapper: PhotonCameraWrapper = PhotonCameraWrapper()

    init {
        Solver.robotContainer = this
    }

    val drivetrain = Drivetrain(
        controlSchemeA,
        cameraWrappers = listOf(cameraWrapper),
        this
    )
    val manipulator = Manipulator()
    val elevator = Elevator(this)
    val arm = Arm()
    val wrist = Wrist(arm).apply {
        defaultCommand = LevelWrist(
            this, arm, toRadians(60.0)
        )
    }

//    val powerDistributionHub = PowerDistribution(0, kRev)

    init {
        arrayOf(controlSchemeA, controlSchemeB).forEachIndexed { i, it ->
            it.run {
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
                    .whileTrue(
                        SetManipulatorSpeed(manipulator, 0.0, true)
                    )

                // assign the close manipulator trigger to the command that
                // closes the manipulator
                closeManipulator
                    .whileTrue(SetManipulatorSpeed(manipulator,1.0, false))

                toggleManipulator
                    .toggleOnFalse(
                        SetManipulatorSpeed(manipulator,0.1)
                    ).toggleOnTrue(
                        SetManipulatorSpeed(manipulator, 0.0)
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
                lowIntake
                    .whileTrue(
                        IntakePositionForward(elevator, arm, wrist)
                            .withManipulator(manipulator)
                    ).onFalse(HoldPosition(elevator, arm, wrist))

                // assign outtake to set manipulator speed to -0.5
                outtake
                    .whileTrue(SetManipulatorSpeed(manipulator, -0.4)).onFalse(SetManipulatorSpeed(manipulator, 0.0))

                highIntake
                    .whileTrue(
                        SetPosition.humanPlayer(elevator, arm, wrist).alongWith(CollectObject(manipulator))
                            // previous setposition command was finishing before the race would actually work
                    )

                moveToClosest
                    .whileTrue(
                        MoveToPosition.snapToYValue(drivetrain,
                            {Constants.Field2dLayout.Axes.YInt.platforms[0]},
                            r = {
                                if (Game.alliance == Alliance.red) Rotation2d()
                                else Rotation2d.fromDegrees(180.0)
                                },
                        )
                    )

                xbox!!.povDown().onTrue(
                    InstantCommand ({
                        if (!Game.COMPETITION)
                            drivetrain.zeroHeading()
                    }, drivetrain)
                )
                if (i == 0) {
                    //fixme dont do this
                    xbox!!.povLeft().onTrue(
                        InstantCommand({
                            arm.armOffset += 5.0
                        }, arm).ignoringDisable(true)
                    )
                    xbox!!.povRight().onTrue(
                        InstantCommand({
                            arm.armOffset -= 5.0
                        }, arm).ignoringDisable(true)
                    )
                } else {
                    xbox!!.povLeft().onTrue(
                        InstantCommand({
                            this@RobotContainer.wantingObject =
                                if (this@RobotContainer.wantingObject == GamePiece.cube)
                                    GamePiece.cone
                                else GamePiece.cube
                        }, NoSubsystem).ignoringDisable(true)
                    )
                }
            }
        }
    }
    var wantingObject: GamePiece = GamePiece.cone
    val leds = KLEDStrip(9, Constants.leds.count).apply {
        val coral = Color(255, 93, 115)
        val coneColor = Color(255, 255, 0)
        val cubeColor = Color(123, 0, 255)

        val allianceColor = if (Game.alliance == DriverStation.Alliance.Red) coral else Color.CYAN

        // idle alliance animations
        val prematchArms = AnimationRGBFade(7.seconds)//AnimationRGBWave(1.0, .1.seconds)
        val percentage = AnimationCustom({t, l ->
            val percent = ((Game.batteryVoltage-7.0)/6.0).coerceIn(0.0, 1.0)
            return@AnimationCustom List(l) { i ->
                return@List if (i < (Constants.leds.count * percent)) {
                    Color.green
                } else {
                    allianceColor
                }
            }
        }, {Game.disabled}, false)
        val wantsCone = AnimationSolid(coneColor, condition =
        {!Game.disabled && (wantingObject == GamePiece.cone)})
        val wantsCube = AnimationSolid(cubeColor, condition =
        {!Game.disabled && (wantingObject == GamePiece.cube)})
        val limpMode = AnimationBlink(Color.RED, 0.5.seconds, condition = {isLimpHeld})
        val chain = KLEDRegion(0, Constants.leds.count,
            prematchArms, percentage, wantsCone, wantsCube, limpMode
        )
        this += (chain)
    }
    var isLimpHeld = false
    var isLimp = false
    val limpCommand = RunCommand({
        arm.armMotor.idleMode = CANSparkMax.IdleMode.kCoast
        wrist.wristMotor.idleMode = CANSparkMax.IdleMode.kCoast
        elevator.elevatorMotor.setNeutralMode(NeutralMode.Coast)
        isLimp = true
    }, arm, wrist, elevator)
        .ignoringDisable(true)
        .handleInterrupt {
            arm.armMotor.idleMode = CANSparkMax.IdleMode.kBrake
            wrist.wristMotor.idleMode = CANSparkMax.IdleMode.kBrake
            elevator.elevatorMotor.setNeutralMode(NeutralMode.Brake)
        }
    val limpTrigger: Trigger =
        Trigger {
            Game.TEST && RobotController.getUserButton()
        }.apply {
            this
                .onTrue(InstantCommand({ isLimpHeld = true },NoSubsystem))
                .debounce(0.25).whileTrue(
                    limpCommand
                )
        }
    val limpLongButton = Trigger {RobotController.getUserButton() && !Game.COMPETITION}
        .run {
            this.onTrue(InstantCommand({ isLimpHeld = true },NoSubsystem))
                .debounce(5.0)
                .whileTrue(
                    limpCommand
                )        }
    val auto
    get() =
        ConditionalCommand(
            MoveToPosition.pathRed(drivetrain, elevator, arm, wrist, manipulator),
            ConditionalCommand(
                MoveToPosition.pathBlue(drivetrain, elevator, arm, wrist, manipulator),
                PrintCommand("UNKOWN ALLIANCE ${Game.alliance}"),
                { Game.alliance == DriverStation.Alliance.Blue}
            ),
            { Game.alliance == DriverStation.Alliance.Red }
        )

    fun update() {
        leds.update()
        SmartDashboard.putData("Drivetrain/sendable", drivetrain)
    }
}

object NoSubsystem: SubsystemBase()