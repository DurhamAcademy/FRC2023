package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.kyberlib.command.Game
import frc.kyberlib.lighting.KLEDRegion
import frc.kyberlib.lighting.KLEDStrip
import frc.kyberlib.lighting.animations.*
import frc.kyberlib.math.units.extensions.seconds
import frc.robot.commands.SetManipulatorSpeed
import frc.robot.commands.alltogether.CollectObject
import frc.robot.commands.alltogether.HoldPosition
import frc.robot.commands.alltogether.IntakePositionForward
import frc.robot.commands.alltogether.SetPosition
import frc.robot.commands.arm.SetArmToAngle
import frc.robot.commands.elevator.ElevatorTestDown
import frc.robot.commands.elevator.ElevatorTestUp
import frc.robot.commands.pathing.MoveToPosition
import frc.robot.constants.Field2dLayout
import frc.robot.constants.PDH
import frc.robot.controls.BryanControlScheme
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Manipulator
import frc.robot.utils.PlacePoint
import java.awt.Color
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class RobotContainer {
    val controlSchemeA: ControlScheme = BryanControlScheme(0)//xbox)
    val controlSchemeB: ControlScheme = BryanControlScheme(1)//xbox)

    var cameraWrapper: PhotonCameraWrapper = PhotonCameraWrapper()

    val drivetrain = Drivetrain(
        controlSchemeA,
        cameraWrappers = listOf(cameraWrapper),
        this
    )
    val manipulator = Manipulator()
    val elevator = Elevator(this)
    val arm = Arm()

    val pdh = PowerDistribution(PDH.id, kRev)

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


                toggleManipulator
                    .onTrue(
                        SetManipulatorSpeed(manipulator, 0.0)
                    )

                // assign the grab cone trigger to the command that
                // grabs a cone

                // assign the hold cone trigger to the command that
                // holds a cone

                // idle
                idleConfiguration
                    .whileTrue(SetPosition.idle(elevator, arm))
                    .onFalse(HoldPosition(elevator, arm))

                // assign l1
                placeLvl1
                    .whileTrue(
                        SetPosition.setpoint(PlacePoint.Level1, this@RobotContainer)
                    )
                    println("ran manipulaot rhehe")

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
                    .whileTrue(SetManipulatorSpeed(manipulator, -1.0))
                    .onFalse(SetManipulatorSpeed(manipulator, 0.0))

                intake
                    .whileTrue(SetManipulatorSpeed(manipulator, 1.0))
                    .onFalse(SetManipulatorSpeed(manipulator, 0.0))

                highIntake
                    .whileTrue(
                        SetPosition.humanPlayer(elevator, arm)
                            .alongWith(CollectObject(manipulator))
                            // previous setposition command was finishing before the race would actually work
                    )

                moveToClosestHPS
                    .whileTrue(
                        MoveToPosition.snapToScoring(
                            drivetrain,
                            {
                                return@snapToScoring Field2dLayout.Axes.YInt.platforms.toList()
                            },
                            {
                                return@snapToScoring if (Game.alliance == DriverStation.Alliance.Blue)
                                    listOf(PI, -PI)
                                else if (Game.alliance == DriverStation.Alliance.Red)
                                    listOf(0.0, 2 * PI, -2 * PI)
                                else
                                    listOf(0.0, 2 * PI, -2 * PI, PI, -PI)
                            }
                        )
                    )
                moveToClosestScoreStation
                    .whileTrue(
                        MoveToPosition.snapToScoring(
                            drivetrain,
                            {
                                return@snapToScoring Field2dLayout.Axes.YInt.score.toList()
                            },
                            {
                                return@snapToScoring if (Game.alliance == DriverStation.Alliance.Blue)
                                    listOf(0.0, 2 * PI, -2 * PI, PI, -PI)
                                else if (Game.alliance == DriverStation.Alliance.Red)
                                    listOf(PI, -PI)
                                else
                                    listOf(0.0, 2 * PI, -2 * PI, PI, -PI)
                            }
                        )
                    )

//                if (i == 0) {//warn: this is a hack VERY VERY BAD
//                    //fixme dont do this
//                    xbox!!.povLeft().onTrue(
//                        InstantCommand({
//                            arm.armOffset += 5.0
//                        }, arm).ignoringDisable(true)
//                    )
//                    xbox!!.povRight().onTrue(
//                        InstantCommand({
//                            arm.armOffset -= 5.0
//                        }, arm).ignoringDisable(true)
//                    )
//                } else {
//                    xbox!!.povLeft().onTrue(
//                        InstantCommand({
//                            this@RobotContainer.wantingObject =
//                                if (this@RobotContainer.wantingObject == GamePiece.cube)
//                                    GamePiece.cone
//                                else GamePiece.cube
//                        }, NoSubsystem).ignoringDisable(true)
//                    )
//                }
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
        unknown
    }

    private val lightStatus: LightStatus
        get() = when {
            !DriverStation.isDSAttached() -> LightStatus.NoDriverStation
            Game.disabled -> {
                if (Game.COMPETITION) when (Game.alliance) {
                    Red -> LightStatus.DisabledFMSRed
                    Blue -> LightStatus.DisabledFMSBlue
                    else -> LightStatus.DisabledNoFMS
                } else LightStatus.DisabledNoFMS
            }
            Game.STOPPED -> LightStatus.EStopped
            Game.AUTO -> if (Game.COMPETITION) when (Game.alliance) {
                Red -> LightStatus.AutoFMSRed
                Blue -> LightStatus.AutoFMSBlue
                else -> LightStatus.AutoNoFMS
            } else LightStatus.AutoNoFMS
            Game.OPERATED -> if (Game.COMPETITION) {
                when (Game.alliance) {
                    Red -> LightStatus.TeleopFMSRed
                    Blue -> LightStatus.TeleopFMSBlue
                    else -> LightStatus.TeleopNoFMS
                }
            } else {
                LightStatus.TeleopNoFMS
            }

            else -> LightStatus.unknown
        }
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
            {lightStatus == LightStatus.DisabledNoFMS}
        val fmsRedDisabled =
            AnimationPulse(allianceRed, 1.0.seconds)
            {lightStatus == LightStatus.DisabledFMSRed}
        val fmsBlueDisabled =
            AnimationPulse(allianceBlue, 1.0.seconds)
            {lightStatus == LightStatus.DisabledFMSBlue}
        val eStopped =
            AnimationBlink(Color.red, 0.5.seconds)
            {lightStatus == LightStatus.EStopped}
        val autoNoFMS =
            AnimationBlink(Color.white, 0.5.seconds)
            {lightStatus == LightStatus.AutoNoFMS}
        val autoFMSRed =
            AnimationLightsaber(allianceRed)
            {lightStatus == LightStatus.AutoFMSRed}
        val autoFMSBlue =
            AnimationLightsaber(allianceBlue)
            {lightStatus == LightStatus.AutoFMSBlue}
        val teleopNoFMS =
            AnimationCylon(
                (
                        if (Game.alliance == Red)
                            allianceRed
                        else
                            allianceBlue
                        ),
                5, 2.0.seconds, true
            )
            {lightStatus == LightStatus.TeleopNoFMS}
        val teleopFMSRed =
            AnimationSparkle(allianceRed)
            {lightStatus == LightStatus.TeleopFMSRed}
        val teleopFMSBlue =
            AnimationSparkle(allianceBlue)
            {lightStatus == LightStatus.TeleopFMSBlue}

        val noDriverStation =
            AnimationSparkle(Color.orange)
            {lightStatus == LightStatus.NoDriverStation}

        val nothing =
            AnimationPulse(Color.white.withAlpha(20) *0.2, 1.0.seconds, true)
            { lightStatus == LightStatus.unknown || lightStatus == LightStatus.TeleopNoFMS }


        val chain = KLEDRegion(
            0, frc.robot.constants.leds.count,
            noFMSDisabled, fmsRedDisabled, fmsBlueDisabled, eStopped,
            autoNoFMS, autoFMSRed, autoFMSBlue, noDriverStation, teleopNoFMS,
            teleopFMSRed, teleopFMSBlue, nothing
        )
        this += (chain)
    }

    val auto: Command
    get() = autoChooser.selected

    // auto chooser
    val autoChooser = SendableChooser<Command>().apply {
        setDefaultOption("1", ConditionalCommand(
            MoveToPosition.swerveBrokenAuto(drivetrain, elevator, arm, manipulator),
            ConditionalCommand(
                MoveToPosition.swerveBrokenAuto(drivetrain, elevator, arm, manipulator),
                PrintCommand("UKNOWN ALLIANCE ${Game.alliance}"),
                {Game.alliance == Blue }
            ),
            {Game.alliance == Red }
        ))
        addOption("1", MoveToPosition.swerveBrokenAuto(drivetrain, elevator, arm, manipulator))
        addOption("2", MoveToPosition.blueauto2(drivetrain, elevator, arm, manipulator))
        addOption("3", MoveToPosition.blueauto3(drivetrain, elevator, arm, manipulator))
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

        // transform the arm position to the robot's position
        val armPos = drivetrain.estimatedPose2d + Transform2d(
            Translation2d(-armX - 0.176, 0.0),
            Rotation2d(if (armAngle > 0) PI else 0.0)
        )

        // put the arm position on the field
        armFieldPosition.pose = armPos
    }
}

