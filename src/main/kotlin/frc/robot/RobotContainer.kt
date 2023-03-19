package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType.kRev
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.kyberlib.command.Game
import frc.kyberlib.lighting.KLEDRegion
import frc.kyberlib.lighting.KLEDStrip
import frc.kyberlib.lighting.animations.*
import frc.kyberlib.math.units.extensions.seconds
import frc.robot.RobotContainer.LightStatus.*
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.SetSubsystemPosition
import frc.robot.commands.balance.AutoBalance
import frc.robot.commands.drivetrain.SpinCommand
import frc.robot.commands.elevator.ZeroElevatorAndIdle
import frc.robot.commands.manipulator.SetManipulatorSpeed
import frc.robot.commands.manipulator.Throw
import frc.robot.commands.pathing.MoveToPosition
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.goToHumanPlayerStation
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.goToPlacementPoint
import frc.robot.constants.Field2dLayout
import frc.robot.constants.PDH
import frc.robot.constants.leds.count
import frc.robot.controls.BryanControlScheme
import frc.robot.controls.ChrisControlScheme
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.*
import frc.robot.utils.GamePiece
import frc.robot.utils.GamePiece.*
import frc.robot.utils.Slider
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementLevel
import frc.robot.utils.grid.PlacementSide
import java.awt.Color
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class RobotContainer {
    val controlScheme0: ControlScheme = ChrisControlScheme(0)
    val controlScheme1: ControlScheme = BryanControlScheme(1)

    val smartDashboardSelector = DashboardSelector()

    var cameraWrapper: PhotonCameraWrapper = PhotonCameraWrapper()

    val drivetrain = Drivetrain(
        controlScheme0,
        cameraWrappers = listOf(cameraWrapper),
        this
    )
    val manipulator = Manipulator()
    val arm = Arm()
    val elevator = Elevator(this, arm)

    val pdh = PowerDistribution(PDH.id, kRev)

    init {
        arrayOf(controlScheme0, controlScheme1).forEachIndexed { i, it ->
            it.run {
                toggleManipulator
                    .onTrue(
                        SetManipulatorSpeed(manipulator, 0.0)
                    )

                idleConfiguration
                    .whileTrue(
                        SetSubsystemPosition(elevator, arm, { IOLevel.Idle }, { wantedObject }, true)
                            .andThen(ZeroElevatorAndIdle(elevator, arm))
                            .andThen(SetSubsystemPosition(elevator, arm, { IOLevel.Idle }, { wantedObject }, false))
                    )

                // assign l1
                placeLvl1
                    .whileTrue(
                        SetSubsystemPosition(this@RobotContainer, { IOLevel.Low }, { wantedObject })
                    )

                // assign l2
                placeLvl2
                    .whileTrue(
                        SetSubsystemPosition(this@RobotContainer, { IOLevel.Mid }, { wantedObject })
                    )

                // assign l3
                placeLvl3
                    .whileTrue(
                        SetSubsystemPosition(this@RobotContainer, { IOLevel.High }, { wantedObject })
                    )

                // assign intake
                lowIntake
                    .whileTrue(
                        SetSubsystemPosition(this@RobotContainer, { IOLevel.FloorIntake }, { wantedObject })
                            .alongWith(SetManipulatorSpeed(manipulator, 1.0))
                    )

                // assign outtake to set manipulator speed to -0.5
//                spinIntakeOut
//                    .whileTrue(SetManipulatorSpeed(manipulator, -0.2))
//                    .onFalse(SetManipulatorSpeed(manipulator, 0.1))

                throwObject
                    .whileTrue(Throw(manipulator, { wantedObject }) { smartDashboardSelector.placementLevel })
                    .onFalse(SetManipulatorSpeed(manipulator, 0.0))

                spinIntakeIn
                    .whileTrue(SetManipulatorSpeed(manipulator, 1.0))
                    .onFalse(SetManipulatorSpeed(manipulator, 0.1))

                intakeHPS
                    .whileTrue(
                        SetSubsystemPosition(this@RobotContainer, { IOLevel.HumanPlayerSlider }, { wantedObject })
                            .alongWith(
                                SetManipulatorSpeed(manipulator, 1.0)
                            )
                    )
                    .onFalse(
                        SetManipulatorSpeed(manipulator, 0.1)
                    )

                moveToClosestHPSAxis
                    .whileTrue(
                        MoveToPosition.snapToScoring(
                            drivetrain,
                            {
                                return@snapToScoring Field2dLayout.Axes.YInt.platforms.toList()
                            },
                            {
                                return@snapToScoring when (Game.alliance) {
                                    DriverStation.Alliance.Blue -> listOf(PI, -PI)
                                    DriverStation.Alliance.Red -> listOf(0.0, 2 * PI, -2 * PI)
                                    else -> listOf(0.0, 2 * PI, -2 * PI, PI, -PI)
                                }
                            }
                        )
                    )

                moveToClosestScoreStationAxis
                    .whileTrue(
                        MoveToPosition.snapToScoring(
                            drivetrain,
                            {
                                return@snapToScoring Field2dLayout.Axes.YInt.score.toList()
                            },
                            {
                                return@snapToScoring when (Game.alliance) {
                                    DriverStation.Alliance.Blue -> listOf(0.0, 2 * PI, -2 * PI, PI, -PI)
                                    DriverStation.Alliance.Red -> listOf(PI, -PI)
                                    else -> listOf(0.0, 2 * PI, -2 * PI, PI, -PI)
                                }
                            }
                        )
                    )

                autoBalance
                    .whileTrue(AutoBalance(drivetrain, -1.0))

                selectGridUp
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(0, 1))
                selectGridDown
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(0, -1))
                selectGridLeft
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(-1, 0))
                selectGridRight
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(1, 0))

                confirmGridSelection
                    .whileTrue(
                        goToPlacementPoint(
                            drivetrain,
                            arm,
                            { smartDashboardSelector.placementLevel.ioLevel },
                            { smartDashboardSelector.placementPosition },
                            { smartDashboardSelector.placementSide },
                        )
                            .deadlineWith(
                                SetSubsystemPosition(
                                    elevator, arm,
                                    { IOLevel.Idle },
                                    { smartDashboardSelector.placementSide.asObject },
                                )
                            )
                            .andThen(
                                SetSubsystemPosition(
                                    elevator, arm,
                                    { smartDashboardSelector.placementLevel.ioLevel },
                                    { smartDashboardSelector.placementSide.asObject },
                                )
                            )
                    )
                alignClosestHPS
                    .whileTrue(
                        goToHumanPlayerStation(
                            drivetrain,
                            arm,
                            { Slider.far },
                            endAtAlignment = true
                        ).deadlineWith(
                            SetSubsystemPosition(
                                elevator, arm,
                                { IOLevel.Idle },
                                { smartDashboardSelector.placementSide.asObject },
                            )
                        )
                            .andThen(
                                SetSubsystemPosition(
                                    elevator, arm,
                                    { IOLevel.HumanPlayerSlider },
                                    { smartDashboardSelector.placementSide.asObject },
                                )
                                    .alongWith(
                                        WaitCommand(0.1)
                                            .andThen(
                                                WaitUntilCommand {
                                                    elevator.motorPid.atGoal() && arm.armPID.atGoal()
                                                }.andThen(
                                                    goToHumanPlayerStation(
                                                        drivetrain,
                                                        arm,
                                                        { Slider.far },
                                                        endAtAlignment = false
                                                    ).deadlineWith(
                                                        SetManipulatorSpeed(manipulator, 1.0)
                                                    )
                                                )
                                            )
                                    )
                        )
                    )
            }
        }
    }

    private enum class LightStatus {
        DisabledNoFMS,
        DisabledFMSRed,
        DisabledFMSBlue,
        EStopped,
        AutoNoFMS,
        AutoFMSRed,
        AutoFMSBlue,
        TeleopNoFMS,
        TeleopFMSRed,
        TeleopFMSBlue,
        NoDriverStation,
        Unknown
    }

    private val lightStatus: LightStatus
        get() = when {
            !DriverStation.isDSAttached() -> NoDriverStation
            Game.disabled -> {
                if (Game.COMPETITION) when (Game.alliance) {
                    Red -> DisabledFMSRed
                    Blue -> DisabledFMSBlue
                    else -> DisabledNoFMS
                } else DisabledNoFMS
            }

            Game.STOPPED -> EStopped
            Game.AUTO -> if (Game.COMPETITION) when (Game.alliance) {
                Red -> AutoFMSRed
                Blue -> AutoFMSBlue
                else -> AutoNoFMS
            } else AutoNoFMS

            Game.OPERATED -> if (Game.COMPETITION) {
                when (Game.alliance) {
                    Red -> TeleopFMSRed
                    Blue -> TeleopFMSBlue
                    else -> TeleopNoFMS
                }
            } else {
                TeleopNoFMS
            }

            else -> Unknown
        }

    val wantedObject: GamePiece
        get() = smartDashboardSelector.placementSide.asObject

    val leds = KLEDStrip(9, count).apply {
        val coral = Color(255, 93, 115)
        val coneColor = Color(255, 255, 0)
        val cubeColor = Color(123, 0, 255)
        val allianceRed = coral
        val allianceBlue = Color.cyan

        Game.startTime
        // idle alliance animations
        val noFMSDisabled =
            AnimationRGBWave(1.0, 0.05.seconds)
            { lightStatus == DisabledNoFMS }
        val fmsRedDisabled =
            AnimationPulse(allianceRed, 1.0.seconds)
            { lightStatus == DisabledFMSRed }
        val fmsBlueDisabled =
            AnimationPulse(allianceBlue, 1.0.seconds)
            { lightStatus == DisabledFMSBlue }
        val eStopped =
            AnimationBlink(Color.red, 0.5.seconds)
            { lightStatus == EStopped }
        val autoNoFMS =
            AnimationBlink(Color.white, 0.5.seconds)
            { lightStatus == AutoNoFMS }
        val autoFMSRed =
            AnimationLightsaber(allianceRed)
            { lightStatus == AutoFMSRed }
        val autoFMSBlue =
            AnimationLightsaber(allianceBlue)
            { lightStatus == AutoFMSBlue }
        val teleopNoFMSRed =
            AnimationCylon(
                allianceRed,
                10,
                2.0.seconds,
                true
            )
            { lightStatus == TeleopNoFMS && Game.alliance == Red }
        val teleopNoFMSBlue =
            AnimationCylon(
                allianceBlue,
                10,
                2.0.seconds,
                true
            )
            { lightStatus == TeleopNoFMS && Game.alliance == Blue }
        val teleopFMSRed =
            AnimationSparkle(allianceRed)
            { lightStatus == TeleopFMSRed && wantedObject == none }
        val teleopFMSBlue =
            AnimationSparkle(allianceBlue)
            { lightStatus == TeleopFMSBlue && wantedObject == none }
        val teleopCone = AnimationSolid(coneColor, true) {
            when (lightStatus) {
                TeleopFMSRed -> true
                TeleopFMSBlue -> true
                TeleopNoFMS -> true
                else -> false
            } && (wantedObject == cone)
        }
        val teleopCube = AnimationSolid(cubeColor, true) {
            when (lightStatus) {
                TeleopFMSRed -> true
                TeleopFMSBlue -> true
                TeleopNoFMS -> true
                else -> false
            } && (wantedObject == cube)
        }
        val noDriverStation =
            AnimationSparkle(Color.orange)
            { lightStatus == NoDriverStation }

        val nothing =
            AnimationPulse(Color.white.withAlpha(20) * 0.2, 1.0.seconds, true)
            { lightStatus == Unknown || lightStatus == TeleopNoFMS }

        val cameraReady = AnimationCustom({ _, len ->
            val percent = drivetrain.cameraWrappers.maxOf { it.percentage }
            val color = Color(256 - (percent * 256).toInt(), (percent * 256).toInt(), 0)
            val index = (percent * len).toInt()
            return@AnimationCustom List<Color>(len) { i ->
                if (i >= index) color else Color.black
            }
        }, { !drivetrain.canTrustPose })


        val body = KLEDRegion(
            0,
            count,
            cameraReady, noFMSDisabled, fmsRedDisabled, fmsBlueDisabled, eStopped, autoNoFMS,
            autoFMSRed, autoFMSBlue, noDriverStation, teleopCone, teleopCube,
            teleopFMSRed, teleopFMSBlue, teleopNoFMSRed, teleopNoFMSBlue,
            nothing,
        )
        this += body
    }

    val auto: Command
        get() = autoChooser.selected

    // auto chooser
    val autoChooser = SendableChooser<Command>().apply {
        setDefaultOption("Spin", SpinCommand(drivetrain))
        addOption(
            "1",
            SetSubsystemPosition(elevator, arm, { IOLevel.Idle }, { cone }, true)
                .alongWith(SetManipulatorSpeed(manipulator, 0.5).withTimeout(0.25))
                .andThen(ZeroElevatorAndIdle(elevator, arm))
                .andThen(
                    goToPlacementPoint(
                        drivetrain,
                        arm,
                        { PlacementLevel.Level3.ioLevel },
                        { PlacementGroup.Farthest },
                        { PlacementSide.FarCone }
                    )
                )
                .andThen(
                    Commands.runOnce({
                        drivetrain.drive(ChassisSpeeds(0.0, 0.0, 0.0), false)
                    }, drivetrain)
                )
                .andThen(SetSubsystemPosition(elevator, arm, { IOLevel.High }, { cone }, true))
                .andThen(Throw(manipulator, { cone }, { PlacementLevel.Level3 }).withTimeout(0.5))
                .andThen(SetSubsystemPosition(elevator, arm, { IOLevel.Idle }, { cone }, true))
        )
        addOption(
            "2",
            goToPlacementPoint(
                drivetrain,
                arm,
                PlacementLevel.Level2.ioLevel,
                PlacementGroup.Farthest,
                PlacementSide.Cube
            )
        )
        addOption(
            "3",
            goToPlacementPoint(
                drivetrain,
                arm,
                PlacementLevel.Level3.ioLevel,
                PlacementGroup.Farthest,
                PlacementSide.Cube
            )
        )
    }

    // shuffleboard auto chooser
    val autoChooserTab = Shuffleboard.getTab("Autonomous")
    val autoChooserWidget = autoChooserTab.add("Autonomous", autoChooser)

    val armVisual = Field2d()
    val armLine = armVisual.getObject("arm")
    val elevatorLine = armVisual.getObject("elevator")

    val armFieldPosition = drivetrain.field2d.getObject("arm")

    fun update() {
        leds.update()
        smartDashboardSelector.update()

        // send subsystems to SmartDashboard
        SmartDashboard.putData("Drivetrain/sendable", drivetrain)
        SmartDashboard.putData("elevator/sendable", elevator)
        SmartDashboard.putData("Arm/sendable", arm)
        SmartDashboard.putData("Manipulator/sendable", manipulator)

        // put arm angle onto the simulation field by using trig to get the x
        // coordinate and then using the transform function to offset it from
        // the center of the robot
        val armAngle = arm.armPosition // 0 is straight up, pi/2 is straight out
        val armLength = 1.047 // length of the arm in meters
        // (+1 is 1 meter forward, -1 is 1 meter back)
        // armLen * cos(armAngle) = x
        // armLen * sin(armAngle) = z
        val armX = armLength * sin(armAngle)
        val armZ = armLength * cos(armAngle)
        if(armZ + elevator.height > inchesToMeters(76.0)){ elevator.setpoint = inchesToMeters(76.0) - armZ }

        // transform the arm position to the robot's position
        val armPos = drivetrain.estimatedPose2d + Transform2d(
            Translation2d(-armX - 0.176, 0.0),
            Rotation2d(if (armAngle > 0) PI else 0.0)
        )

        // put the arm position on the field
        armFieldPosition.pose = armPos
    }
}

