package frc.robot
/*
Wrap all auto commands (in robotcontainer) with timeout for 14.5 sec, lock wheels after that
Lock wheels should require the drivetrain and stop movement so we don't try to move with wheels locked
Make sure auto command gets canceled going into teleop and that wheels can unlock properly
 */
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.kyberlib.command.Game
import frc.kyberlib.lighting.KLEDRegion
import frc.kyberlib.lighting.KLEDStrip
import frc.kyberlib.lighting.animations.*
import frc.kyberlib.math.units.extensions.seconds
import frc.robot.RobotContainer.LightStatus.*
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.SetSubsystemPosition
import frc.robot.commands.drivetrain.DriveCommand
import frc.robot.commands.elevator.ZeroElevatorAndIdle
import frc.robot.commands.intake.DeployIntake
import frc.robot.commands.intake.IntakeEject
import frc.robot.commands.manipulator.ManipulatorIO
import frc.robot.commands.manipulator.SetManipulatorSpeed
import frc.robot.commands.manipulator.Throw
import frc.robot.commands.pathing.*
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.goToHumanPlayerStation
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.goToPlacementPoint
import frc.robot.constants.Field2dLayout
import frc.robot.constants.leds.count
import frc.robot.controls.BryanControlScheme
import frc.robot.controls.ChrisControlScheme
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.*
import frc.robot.utils.GamePiece
import frc.robot.utils.GamePiece.*
import frc.robot.utils.Slider
import java.awt.Color
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance as commandSchedulerInstance

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
    val intake = Intake(arm)
    val elevator = Elevator(this@RobotContainer, arm, this.intake)


    init {
        arrayOf(controlScheme0, controlScheme1).forEachIndexed { i, it ->
            it.run {
                toggleManipulator
                    .onTrue(
                        SetManipulatorSpeed(manipulator, 0.0)
                    )

                idleConfiguration
                    .whileTrue(
                        SetSubsystemPosition(elevator, arm, drivetrain, { IOLevel.Idle }, { wantedObject }, true)
                            .andThen(ZeroElevatorAndIdle(elevator, arm))
                            .andThen(
                                SetSubsystemPosition(
                                    elevator,
                                    arm,
                                    drivetrain,
                                    { IOLevel.Idle },
                                    { wantedObject },
                                    false
                                )
                            )
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
                            .alongWith(SetManipulatorSpeed(manipulator, 0.6502))
                    )

                // assign outtake to set manipulator speed to -0.5
//                spinIntakeOut
//                    .whileTrue(SetManipulatorSpeed(manipulator, -0.2))
//                    .onFalse(SetManipulatorSpeed(manipulator, 0.1))

                throwObject
                    .whileTrue(Throw(manipulator, { wantedObject }) { smartDashboardSelector.placementLevel })
                    .onFalse(SetManipulatorSpeed(manipulator, 0.0))

                spinIntakeIn
                    .whileTrue(SetManipulatorSpeed(manipulator, 0.6502))
                    .onFalse(SetManipulatorSpeed(manipulator, 0.1))

                intakeHPS
                    .whileTrue(
                        SetSubsystemPosition(this@RobotContainer, { IOLevel.HumanPlayerSlider }, { wantedObject })
                            .alongWith(
                                SetManipulatorSpeed(manipulator, 0.6502)
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

                selectGridUp
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(0, 1))
                selectGridDown
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(0, -1))
                selectGridLeft
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(1, 0))
                selectGridRight
                    .onTrue(this@RobotContainer.smartDashboardSelector.moveCommand(-1, 0))

                confirmGridSelection
                    .whileTrue(
                        goToPlacementPoint(
                            drivetrain,
                            arm,
                            { smartDashboardSelector.placementLevel.ioLevel },
                            { smartDashboardSelector.placementPosition },
                            { smartDashboardSelector.placementSide },
                        )
//                            .deadlineWith(
//                                SetSubsystemPosition(
//                                    elevator, arm,
//                                    drivetrain,
//                                    { IOLevel.Idle },
//                                    { smartDashboardSelector.placementSide.asObject },
//                                )
//                            )
//                            .andThen(
//                                SetSubsystemPosition(
//                                    elevator, arm,
//                                    drivetrain,
//                                    { smartDashboardSelector.placementLevel.ioLevel },
//                                    { smartDashboardSelector.placementSide.asObject },
//                                )
//                            )
                    )

                ledColor
                    .onTrue(InstantCommand({
                        wantedObject = when (wantedObject) {
                            cone -> cube
                            cube -> cone
                            else -> cone
                        }
                    }))

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
                                drivetrain,
                                { IOLevel.Idle },
                                { smartDashboardSelector.placementSide.asObject },
                            )
                        )
                            .andThen(
                                SetSubsystemPosition(
                                    elevator, arm,
                                    drivetrain,
                                    { IOLevel.HumanPlayerSlider },
                                    { smartDashboardSelector.placementSide.asObject },
                                    stopAtEnd = true
                                )
                                    .andThen(
                                        SetSubsystemPosition(
                                            elevator, arm,
                                            drivetrain,
                                            { IOLevel.HumanPlayerSlider },
                                            { smartDashboardSelector.placementSide.asObject },
                                            stopAtEnd = false
                                        ).withTimeout(1.0)
                                    )
                                    .andThen(
                                        goToHumanPlayerStation(
                                            drivetrain,
                                            arm,
                                            { Slider.far },
                                            endAtAlignment = false
                                        )
                                    )
                                    .deadlineWith(
                                        ManipulatorIO(
                                            manipulator,
                                            { smartDashboardSelector.placementSide.asObject },
                                            { IOLevel.HumanPlayerSlider }
                                        )
                                    )
                                    .andThen(
                                        ManipulatorIO(
                                            manipulator,
                                            { smartDashboardSelector.placementSide.asObject },
                                            { IOLevel.HumanPlayerSlider }
                                        )
                                    )
                            ).andThen(
                                SetSubsystemPosition(
                                    elevator, arm,
                                    drivetrain,
                                    { IOLevel.Idle },
                                    { smartDashboardSelector.placementSide.asObject },
                                    stopAtEnd = true
                                ).deadlineWith(
                                    goToHumanPlayerStation(
                                        drivetrain,
                                        arm,
                                        { Slider.far },
                                        endAtAlignment = true
                                    )
                                )
                            )
                    )

                lockSwerveModulesCircle
                    .whileTrue(DriveCommand(drivetrain, rotation = { 0.01 }))

                intakeGroundIntake
                    .whileTrue(DeployIntake(intake, this@RobotContainer))

                intakeEject
                    .whileTrue(IntakeEject(intake, this@RobotContainer))
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
    var fullAuto = false
    var fullAutoObject: GamePiece = none

    var wantedObject: GamePiece = none
        get() =
            if (fullAuto) fullAutoObject
            else field

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
            { lightStatus == AutoNoFMS && !fullAuto }
        val autoFMSRed =
            AnimationLightsaber(allianceRed)
            { lightStatus == AutoFMSRed && !fullAuto }
        val autoFMSBlue =
            AnimationLightsaber(allianceBlue)
            { lightStatus == AutoFMSBlue && !fullAuto }
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
        val autoCone = AnimationPulse(coneColor, 1.0.seconds) {
            when (lightStatus) {
                AutoFMSRed -> true
                AutoFMSBlue -> true
                AutoNoFMS -> true
                else -> false
            } && (fullAutoObject == cone) && fullAuto
        }
        val autoCube = AnimationPulse(cubeColor, 1.0.seconds) {
            when (lightStatus) {
                AutoFMSRed -> true
                AutoFMSBlue -> true
                AutoNoFMS -> true
                else -> false
            } && (fullAutoObject == cube) && fullAuto
        }
        val noDriverStation =
            AnimationSparkle(Color.orange)
            { lightStatus == NoDriverStation }

        val nothing =
            AnimationPulse(Color.white.withAlpha(20) * 0.2, 1.0.seconds, true)
            { lightStatus == Unknown || lightStatus == TeleopNoFMS }

        val cameraReady = AnimationCustom({ _, len ->
            val percent = drivetrain.cameraWrappers.maxOf { it.percentage }
            val color = Color(255 - (percent * 255).toInt(), (percent * 255).toInt(), 0)
            val index = (percent * len).toInt()
            return@AnimationCustom List<Color>(len) { i ->
                if (i >= index) color else Color.black
            }
        }, { false && !drivetrain.canTrustPose && (lightStatus != TeleopFMSRed || lightStatus != TeleopFMSBlue) })


        val body = KLEDRegion(
            0,
            count,
            cameraReady, noFMSDisabled, fmsRedDisabled, fmsBlueDisabled,
            eStopped, autoCone, autoCube, autoNoFMS, autoFMSRed, autoFMSBlue,
            noDriverStation, teleopCone, teleopCube, teleopFMSRed, teleopFMSBlue,
            teleopNoFMSRed, teleopNoFMSBlue, nothing, cameraReady
        )
        this += body
    }

    val auto: Command
        get() {
            var c = (Commands.runOnce({ // assume the elevator is starting from the top.
                if (!elevator.hasLimitBeenPressed) {
                    println("RESET ELEVATOR")
                    elevator.height = frc.robot.constants.elevator.limits.topLimit
                }
                elevator.setpoint = frc.robot.constants.elevator.limits.topLimit
                elevator.motorPid.reset(elevator.height)
            })
                .andThen(Commands.runOnce({ arm.setArmPosition(-PI / 2) }))
                .andThen(Commands.waitUntil { arm.armPosition > -3 * PI / 4 }) // move the arm to horizontal
                .andThen(SetSubsystemPosition(elevator, arm, drivetrain, { IOLevel.Idle }, { wantedObject }, true)))


            if (autoChooser.selected != null) {
                c =
                    c.andThen(autoChooser.selected!!.getCommand()) // this needs to be like this because of command composition rules. this gets a fresh one each time instead of keeping one instance in the chooser
            }

            return c.withTimeout(14.5) // go to idle
                .andThen(DriveCommand(drivetrain, rotation = { 0.0001 }))
        }


    // auto chooser
    val autoChooser = SendableChooser<Auto?>().apply {
        setDefaultOption("None", null)
        addOption("Place And Balance", AutoPlaceAndBalance(this@RobotContainer))
        addOption("Place and Taxi farthest from judges", TaxiAndSomethingOrOther(this@RobotContainer))
        addOption("Place Cone Only (Backup)", OnlyPlaceConeAuto(this@RobotContainer))
    }

    // shuffleboard auto chooser
    val autoChooserTab: ShuffleboardTab = Shuffleboard.getTab("Autonomous")
    val autoChooserWidget = autoChooserTab.add("Autonomous", autoChooser)

    val armFieldPosition = drivetrain.field2d.getObject("arm")

//    val DriveTab: ShuffleboardTab = Shuffleboard.getTab("DriveTab")
//    val autoChoice = DriveTab.add("Autonomous", autoChooser)
//    val fieldWidget = DriveTab.add("Field", field2dwidget)

    //val CameraWidget = DriveTab.add("Camera", )
//    val cameraWidget = DriveTab.addCamera(
//        "Photon",
//        "photonvision_Port_1182_MJPEG_Server",
//        "http://photonvision.local:1182/stream.mjpg"
//    )

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
        if (armZ + elevator.height > inchesToMeters(76.0))
            elevator.setpoint = inchesToMeters(76.0) - armZ

        // transform the arm position to the robot's position
        val armPos = drivetrain.estimatedPose2d + Transform2d(
            Translation2d(-armX - 0.176, 0.0),
            Rotation2d(if (armAngle > 0) PI else 0.0)
        )

        // put the arm position on the field
        armFieldPosition.pose = armPos

        if (Game.COMPETITION && Game.disabled &&
            listOf(controlScheme0, controlScheme1).any {
                if (it.xbox == null) false
                else (0 until ((it.xbox?.hid?.buttonCount) ?: 0)).any { i ->
                    it.xbox?.hid?.button(
                        i, commandSchedulerInstance().defaultButtonLoop
                    )?.asBoolean == true
                }
            }
        ) controlScheme0.xbox?.hid?.setRumble(GenericHID.RumbleType.kBothRumble, .3)
        else controlScheme0.xbox?.hid?.setRumble(GenericHID.RumbleType.kBothRumble, .0)
    }
}

