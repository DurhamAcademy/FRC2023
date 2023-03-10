package frc.robot

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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
import frc.kyberlib.math.units.Pose2d
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.extensions.radians
import frc.kyberlib.math.units.extensions.seconds
import frc.robot.commands.ElevatorTestDown
import frc.robot.commands.ElevatorTestUp
import frc.robot.commands.pathing.MoveToPosition
import frc.robot.commands.alltogether.*
import frc.robot.commands.arm.SetArmToAngle
import frc.robot.commands.manipulator.SetManipulatorSpeed

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
//                testWrist90
//                    .whileTrue(
//                        SetWristAngle(wrist, PI / 2)
//                    )
//                testWrist0
//                    .whileTrue(
//                        //                    OpenManipulator(manipulator)
//                        //                        .andThen(
//                        SetManipulatorSpeed(manipulator, -0.1)//)
//                    )
//                testWristNeg90
//                    .whileTrue(
//                        SetWristAngle(wrist, -PI / 2)
//                    )

                // assign the open manipulator trigger to the command that
                // opens the manipulator
                openManipulator
                    .whileTrue(
                        SetManipulatorSpeed(manipulator, 0.0)
                    )

                // assign the close manipulator trigger to the command that
                // closes the manipulator
                closeManipulator
                    .whileTrue(SetManipulatorSpeed(manipulator,1.0))

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
                    .onFalse(HoldPosition(elevator, arm))

                // assign l1
                placeLvl1
                    .whileTrue(
                        SetPosition.setpoint(PlacePoint.Level1, this@RobotContainer)
                    )

                // assign l2
                placeLvl2
                    .whileTrue(
                        SetPosition.setpoint(PlacePoint.Level2, this@RobotContainer)
                    ).onFalse(HoldPosition(elevator, arm))

                // assign l3
                placeLvl3
                    .whileTrue(
                        SetPosition.setpoint(PlacePoint.Level3, this@RobotContainer)
                    ).onFalse(HoldPosition(elevator, arm))

                // assign intake
                lowIntake
                    .whileTrue(
                        IntakePositionForward(elevator, arm)
                            .withManipulator(manipulator)
                    ).onFalse(HoldPosition(elevator, arm))

                // assign outtake to set manipulator speed to -0.5
                outtake
                    .whileTrue(SetManipulatorSpeed(manipulator, -0.4)).onFalse(SetManipulatorSpeed(manipulator, 0.0))

                highIntake
                    .whileTrue(
                        SetPosition.humanPlayer(elevator, arm).alongWith(CollectObject(manipulator))
                            // previous setposition command was finishing before the race would actually work
                    )

                moveToClosestHPS
                    .whileTrue(
                        MoveToPosition.snapToScoring(
                            drivetrain,
                            {
                                return@snapToScoring Constants.Field2dLayout.Axes.YInt.platforms.toList()
                            },
                            {return@snapToScoring listOf(-2*PI, -PI, 0.0, PI, 2*PI)}
                        )
                    )
                moveToClosestScoreStation
                    .whileTrue(
                        MoveToPosition.snapToScoring(
                            drivetrain,
                            {
                                return@snapToScoring Constants.Field2dLayout.Axes.YInt.score.toList()
                            },
                            {return@snapToScoring listOf(-2*PI, -PI, 0.0, PI, 2*PI)}
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
        val prematchArms = AnimationRGBWave(1.0, 1.0.seconds) {Game.disabled}
//        val percentage = AnimationCustom({t, l ->
//            val percent = ((Game.batteryVoltage-7.0)/6.0).coerceIn(0.0, 1.0)
//            return@AnimationCustom List(l) { i ->
//                return@List if (i < (Constants.leds.count * percent)) {
//                    Color.green
//                } else {
//                    allianceColor
//                }
//            }
//        }, {Game.disabled}, false)
        val wantsCone = AnimationSolid(coneColor, condition =
        {!Game.disabled && (wantingObject == GamePiece.cone)})
        val wantsCube = AnimationSolid(cubeColor, condition =
        {!Game.disabled && (wantingObject == GamePiece.cube)})
        val limpMode = AnimationBlink(Color.RED, 0.5.seconds, condition = {isLimpHeld})
        val chain = KLEDRegion(0, Constants.leds.count,
            prematchArms,/* percentage,*/ wantsCone, wantsCube, limpMode
        )
        this += (chain)
    }
    var isLimpHeld = false
    var isLimp = false
    val limpCommand = RunCommand({
        arm.armMotor.idleMode = CANSparkMax.IdleMode.kCoast
        //wrist.wristMotor.idleMode = CANSparkMax.IdleMode.kCoast
        elevator.elevatorMotor.setNeutralMode(NeutralMode.Coast)
        isLimp = true
    }, arm, elevator)
        .ignoringDisable(true)
        .handleInterrupt {
            arm.armMotor.idleMode = CANSparkMax.IdleMode.kBrake
            //wrist.wristMotor.idleMode = CANSparkMax.IdleMode.kBrake
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
            MoveToPosition.blueauto2low(drivetrain, elevator, arm, manipulator),
            ConditionalCommand(
                MoveToPosition.blueauto2low(drivetrain, elevator, arm, manipulator),
                PrintCommand("UNKOWN ALLIANCE ${Game.alliance}"),
                { Game.alliance == DriverStation.Alliance.Blue}
            ),
            { Game.alliance == DriverStation.Alliance.Red }
        )

    val armVisualizer = drivetrain.field2d.getObject("arm")
    fun update() {
        leds.update()
        SmartDashboard.putData("Drivetrain/sendable", drivetrain)
    }
}

object NoSubsystem: SubsystemBase()