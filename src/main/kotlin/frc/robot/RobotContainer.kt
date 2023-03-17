package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
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
import frc.kyberlib.command.Game
import frc.kyberlib.lighting.KLEDRegion
import frc.kyberlib.lighting.KLEDStrip
import frc.kyberlib.lighting.animations.*
import frc.kyberlib.math.units.extensions.seconds
import frc.robot.RobotContainer.LightStatus.*
import frc.robot.commands.alltogether.HoldPosition
import frc.robot.commands.alltogether.IntakePositionForward
import frc.robot.commands.alltogether.SetPosition
import frc.robot.commands.balance.AutoBalance
import frc.robot.commands.elevator.ZeroElevatorAndIdle
import frc.robot.commands.manipulator.SetManipulatorSpeed
import frc.robot.commands.manipulator.Throw
import frc.robot.commands.pathing.MoveToPosition
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.goToCommunityZone
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.goToPlacementPoint
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.leaveCommunityZone
import frc.robot.constants.Field2dLayout
import frc.robot.constants.PDH
import frc.robot.controls.BryanControlScheme
import frc.robot.controls.ChrisControlScheme
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.*
import frc.robot.utils.GamePiece.*
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementSide
import frc.robot.utils.grid.PlacmentLevel
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
                        SetPosition.idle(elevator, arm, true)
                            .andThen(ZeroElevatorAndIdle(elevator, arm))
                            .andThen(SetPosition.idle(elevator, arm, false))
                    )

                // assign l1
                placeLvl1
                    .whileTrue(
                        SetPosition.setpoint(PlacmentLevel.Level1, this@RobotContainer)
                    )

                // assign l2
                placeLvl2
                    .whileTrue(
                        SetPosition.setpoint(PlacmentLevel.Level2, this@RobotContainer)
                    )

                // assign l3
                placeLvl3
                    .whileTrue(
                        SetPosition.setpoint(PlacmentLevel.Level3, this@RobotContainer)
                    )

                // assign intake
                lowIntake
                    .whileTrue(
                        IntakePositionForward(elevator, arm)
                            .alongWith(SetManipulatorSpeed(manipulator, 1.0))
                    )

                // assign outtake to set manipulator speed to -0.5
//                spinIntakeOut
//                    .whileTrue(SetManipulatorSpeed(manipulator, -0.2))
//                    .onFalse(SetManipulatorSpeed(manipulator, 0.1))

                throwObject
                    .whileTrue(Throw(manipulator) { wantedObject })
                    .onFalse(SetManipulatorSpeed(manipulator, 0.0))

                spinIntakeIn
                    .whileTrue(SetManipulatorSpeed(manipulator, 1.0))
                    .onFalse(SetManipulatorSpeed(manipulator, 0.1))

                intakeHPS
                    .whileTrue(
                        SetPosition.humanPlayer(elevator, arm)
                            .alongWith(
                                SetManipulatorSpeed(manipulator, 1.0)
                            )
                    )
                    .onFalse(
                        HoldPosition(elevator, arm)
                            .alongWith(
                                SetManipulatorSpeed(manipulator, 0.1)
                            )
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
                    .whileTrue(AutoBalance(drivetrain))

                selectGridUp
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(0, 1))
                selectGridDown
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(0, -1))
                selectGridLeft
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(-1, 0))
                selectGridRight
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(1, 0))
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

    var wantedObject = none

    val leds = KLEDStrip(9, frc.robot.constants.leds.count).apply {
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


        val body = KLEDRegion(
            0,
            frc.robot.constants.leds.count,
            noFMSDisabled, fmsRedDisabled, fmsBlueDisabled, eStopped, autoNoFMS,
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
        addOption(
            "1",
            goToPlacementPoint(drivetrain, PlacmentLevel.Level2, PlacementGroup.Farthest, PlacementSide.CloseCone)
        )
        addOption(
            "2",
            goToPlacementPoint(drivetrain, PlacmentLevel.Level3, PlacementGroup.Farthest, PlacementSide.Cube)
                .andThen(leaveCommunityZone(drivetrain, arm))
                .andThen(goToCommunityZone(drivetrain))
        )
        addOption("3", leaveCommunityZone(drivetrain, arm))
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

        smartDashboardSelector.selected = 3 to 3

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

