package frc.robot.commands.pathing

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.kyberlib.math.units.extensions.radians
import frc.robot.Constants
import frc.robot.commands.alltogether.Idle
import frc.robot.commands.alltogether.IntakePositionForward
import frc.robot.commands.alltogether.SetPosition
import frc.robot.commands.manipulator.SetManipulatorSpeed
import frc.robot.subsystems.*
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.random.Random

class MoveToPosition(
    private val drivetrain: Drivetrain,
    /**
     * The desired position of the robot (in meters)
     */
    private var pose: () -> Pose2d,
    /**
     * The desired velocity of the robot (in meters per second)
     */
    private val velocity: Transform2d = Transform2d(),
    private val toleranceppos: Double = 0.1,
    private val tolerancepvel: Double = 0.1,
    private val tolerancerpos: Double = 0.025,
    private val tolerancervel: Double = 0.1,
    private val snapMode: Boolean = false
) : CommandBase() {
    constructor(drivetrain: Drivetrain, x: Double = 0.0, y: Double = 0.0, angle: Double = 0.0) : this(
        drivetrain,
        Pose2d(x, y, Rotation2d.fromDegrees(angle)),
        Transform2d(Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)),
    )

    constructor(drivetrain: Drivetrain,
                pose: Pose2d,
                velocity: Transform2d = Transform2d(),
                toleranceppos: Double = 0.1,
                tolerancepvel: Double = 0.1,
                tolerancerpos: Double = 0.025,
                tolerancervel: Double = 0.1,
                snapMode: Boolean = false
    ) : this(
        drivetrain,
        { pose },
        velocity,
        toleranceppos,
        tolerancepvel,
        tolerancerpos,
        tolerancervel,
        snapMode
    )

    init {
        addRequirements(drivetrain)
    }

    // these entries are used to debug how fast the robot wants to move to get
    // to the desired position
//    val speedx = drivetrain.Idrc.add("speedx1${Random.nextDouble()}", 0.0)
//        .entry
//    val speedy = drivetrain.Idrc.add("speedy1${Random.nextDouble()}", 0.0)
//        .entry
//    val speedr = drivetrain.Idrc.add("speedr1${Random.nextDouble()}", 0.0)
//        .entry

    val xPIDController = ProfiledPIDController(
        Companion.xP, 0.0, 0.0, TrapezoidProfile.Constraints(
            4.0,
            3.0
        )
    ).also {
        it.reset(drivetrain.poseEstimator.estimatedPosition.translation.x, 0.0)
        it.setTolerance(toleranceppos, tolerancepvel)
    }
    val yPIDController = ProfiledPIDController(
        Companion.yP, 0.0, 0.0, TrapezoidProfile.Constraints(
            4.0,
            3.0
        )
    ).also {
        it.reset(drivetrain.poseEstimator.estimatedPosition.translation.y, 0.0)
        it.setTolerance(toleranceppos, tolerancepvel)
    }
    val rPIDController = ProfiledPIDController(
        Companion.rP, 0.0, 0.0, TrapezoidProfile.Constraints(
            PI / 2, PI / 1.5
        )
    ).also {
        it.enableContinuousInput(-PI, PI)
        it.reset(drivetrain.poseEstimator.estimatedPosition.rotation.radians, 0.0)
        it.setTolerance(tolerancerpos, tolerancervel)
    }

    val start = drivetrain.poseEstimator.estimatedPosition

    override fun initialize() {
        if (snapMode) pose = {
            (SnapToPostion.closestPose(drivetrain) ?: pose) as Pose2d
        }
        xPIDController.reset(drivetrain.poseEstimator.estimatedPosition.translation.x,0.0)
        yPIDController.reset(drivetrain.poseEstimator.estimatedPosition.translation.y, 0.0)
        rPIDController.reset(drivetrain.poseEstimator.estimatedPosition.rotation.radians, 0.0)
    }

    // on command start and every time the command is executed, calculate the

    override fun execute() {
        val current = drivetrain.poseEstimator.estimatedPosition
        val desired = pose()

        // log the current position and the desired position
        SmartDashboard.putNumber("curr-x", current.translation.x)
        SmartDashboard.putNumber("curr-y", current.translation.y)
        SmartDashboard.putNumber("curr-r", current.rotation.radians)
        SmartDashboard.putNumber("des-x", desired.translation.x)
        SmartDashboard.putNumber("des-y", desired.translation.y)
        SmartDashboard.putNumber("des-r", desired.rotation.radians)


        // calculate the speeds needed to get to the desired position
        val speeds = ChassisSpeeds(
            xPIDController.calculate(
                current.translation.x,
                TrapezoidProfile.State(
                    desired.translation.x,
                    velocity.translation.x
                )
            ),
            yPIDController.calculate(
                current.translation.y,
                TrapezoidProfile.State(
                    desired.translation.y,
                    velocity.translation.y
                )
            ),
            rPIDController.calculate(
                current.rotation.radians,
                TrapezoidProfile.State(
                    desired.rotation.radians,
                    velocity.rotation.radians
                )
            )
        )

        SmartDashboard.putNumber("xpiderr", xPIDController.positionError)
        SmartDashboard.putNumber("ypiderr", yPIDController.positionError)
        SmartDashboard.putNumber("rpiderr", rPIDController.positionError)

        SmartDashboard.putNumber("xpidvel", xPIDController.velocityError)
        SmartDashboard.putNumber("ypidvel", yPIDController.velocityError)
        SmartDashboard.putNumber("rpidvel", rPIDController.velocityError)

        SmartDashboard.putData("xpid", xPIDController)
        SmartDashboard.putData("ypid", yPIDController)
        SmartDashboard.putData("rpid", rPIDController)

        SmartDashboard.putNumber("xpidPosTolerance", xPIDController.positionTolerance)
        SmartDashboard.putNumber("ypidPosTolerance", yPIDController.positionTolerance)
        SmartDashboard.putNumber("rpidPosTolerance", rPIDController.positionTolerance)

        SmartDashboard.putNumber("xpidVelTolerance", xPIDController.velocityTolerance)
        SmartDashboard.putNumber("ypidVelTolerance", yPIDController.velocityTolerance)
        SmartDashboard.putNumber("rpidVelTolerance", rPIDController.velocityTolerance)

        // at goal
        SmartDashboard.putBoolean("xpidAtGoal", xPIDController.atGoal())
        SmartDashboard.putBoolean("ypidAtGoal", yPIDController.atGoal())
        SmartDashboard.putBoolean("rpidAtGoal", rPIDController.atGoal())

        SmartDashboard.putNumber("SPEED", hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
//        val speeds = ChassisSpeeds(
//          1.0, 0.0, 0.0
//        )

        // set the debug entries to the speeds so we can see values in the
        // smartdashboard
//        speedx.setDouble(speeds.vxMetersPerSecond)
//        speedy.setDouble(speeds.vyMetersPerSecond)
//        speedr.setDouble(speeds.omegaRadiansPerSecond)


        // tell the drivetrain to drive at the calculated speeds
        drivetrain.drive(speeds, true)
    }

    override fun isFinished(): Boolean {
        // stop when the robot is within 0.1 meters of the desired position
        return drivetrain.poseEstimator.estimatedPosition.minus(Pose2d(pose().translation, Rotation2d())).translation.norm < toleranceppos
                && drivetrain.poseEstimator.estimatedPosition.rotation.minus(Rotation2d(pose().rotation.radians)).radians < tolerancerpos
    }

    override fun end(interrupted: Boolean) {
        drivetrain.drive(ChassisSpeeds(), true)
        // reset the PID controllers
        xPIDController.reset(0.0, 0.0)
        yPIDController.reset(0.0, 0.0)
        rPIDController.reset(0.0, 0.0)
    }

    companion object {
        const val rP = 4.0
        const val yP = 2.25
        const val xP = 2.25

        fun pathBlue(drivetrain: Drivetrain, elevator: Elevator, arm: Arm, manipulator: Manipulator) =
            run {
                (drivetrain.poseEstimator.estimatedPosition)
                    SetPosition.high(elevator, arm)
                        .withTimeout(3.0)
                    .andThen(SetManipulatorSpeed(manipulator, -1.0, true).withTimeout(1.0))
                    .andThen(Idle(elevator, arm).alongWith(SetManipulatorSpeed(manipulator, 0.0, true)))
            }

        fun humanPlayerPathBlue(drivetrain: Drivetrain, elevator: Elevator, arm: Arm, manipulator: Manipulator) =
            run {
                (drivetrain.poseEstimator.estimatedPosition)
                MoveToPosition(drivetrain, 1.87, 4.42, 0.0).withTimeout(1.0)
                    .andThen(SetPosition.high(elevator, arm).withTimeout(3.0))
                    .andThen(SetManipulatorSpeed(manipulator, 1.0, true)).withTimeout(0.5)
                    .andThen(
                        MoveToPosition(drivetrain, 6.58, 4.59, 180.0).withTimeout(3.0)
                        .alongWith(
                            Idle(elevator, arm).alongWith(SetManipulatorSpeed(manipulator, -1.0, true))
                    ))
                    .andThen(
                        MoveToPosition(drivetrain, 1.89, 4.97, 0.0).withTimeout(3.0)
                        .alongWith(
                            SetPosition.high(elevator, arm)
                            .alongWith(
                                SetManipulatorSpeed(manipulator, 0.0, true)
                    )))
                    .andThen(SetManipulatorSpeed(manipulator, 1.0, true).withTimeout(0.5))
                    .andThen(
                        MoveToPosition(drivetrain, 6.48, 5.00, 180.0).withTimeout(1.2)
                        .alongWith(
                            SetManipulatorSpeed(manipulator, 0.0, true)
                    ))
                    .andThen(MoveToPosition(drivetrain, 7.59, 6.45, 180.0).withTimeout(0.5))

            }

        fun pathRed(drivetrain: Drivetrain, elevator: Elevator, arm: Arm, manipulator: Manipulator) =
            run {
                (drivetrain.poseEstimator.estimatedPosition)
                MoveToPosition(drivetrain, 14.66, 1.05, 180.0).withTimeout(1.0)
                    .andThen(SetManipulatorSpeed(manipulator, 1.0, true).withTimeout(1.0))
                    .andThen(Idle(elevator, arm).withTimeout(0.5).alongWith(SetManipulatorSpeed(manipulator, 0.0, true).withTimeout(0.5)))
                    .andThen(MoveToPosition(drivetrain, 14.0,1.0, 180.0).withTimeout(1.0))
                    .andThen(MoveToPosition(drivetrain, 10.5, 1.0, 180.0).withTimeout(6.0))
            }

        fun pathBlueAdvanced(drivetrain: Drivetrain, elevator: Elevator, arm: Arm, manipulator: Manipulator) =
            run {
                (drivetrain.poseEstimator.estimatedPosition)
                // start
                    // move arm into position
                    SetPosition.high(elevator, arm)
                        .withTimeout(3.0)
                    //eject cube
                    .andThen(SetManipulatorSpeed(manipulator, -1.0, true).withTimeout(1.0))
                    // stop manipulator and move to idle
                    .andThen(
                        Idle(elevator, arm).alongWith(SetManipulatorSpeed(manipulator, 0.0, true))
                            // while this is happening, wait and then begin movement to fit position 1
                            .alongWith(WaitCommand(0.5)
                                .andThen(MoveToPosition(drivetrain, 3.22, 0.73,45.0).withTimeout(1.5))
                                .andThen(MoveToPosition(drivetrain, 4.8,.66, 180.0).withTimeout(1.5))
                            ).withTimeout(3.6)
                    )
                    // move into deploy position and deploy
                    .andThen(
                        MoveToPosition(drivetrain, 5.72, 0.94, 180.0).withTimeout(1.0)
                            // while moving, deploy
                            .alongWith(
                                IntakePositionForward(elevator, arm)
                                    .alongWith(SetManipulatorSpeed(manipulator, 1.0, true)
                                        .withTimeout(1.5)
                                    )
                            )
                        // withTimeout of total deploy time
                            .withTimeout(1.75)
                        //and then move to the should have intaked position
                            .andThen(MoveToPosition(drivetrain,6.66,0.93).withTimeout(2.0))
                    )
                    .andThen(
                        Idle(elevator, arm).alongWith(SetManipulatorSpeed(manipulator, 0.1, false).withTimeout(5.0))
                        // start moving back
                            .alongWith(WaitCommand(0.5)
                                .andThen(MoveToPosition(drivetrain, 4.8, .66,0.0).withTimeout(1.75))
                                .andThen(MoveToPosition(drivetrain, 1.95, 1.05, 0.0).withTimeout(3.5))
                                .andThen(MoveToPosition(drivetrain, 1.87,1.05, 0.0).withTimeout(1.0))
                            ).withTimeout(6.0)
                    )
                    //place
                    .andThen(SetPosition.high(elevator, arm)
                        .withTimeout(3.0))
                    .andThen(SetManipulatorSpeed(manipulator, -1.0, true).withTimeout(1.0))
                    .withTimeout(15.0)

            }

        fun pathRedAdvanced(drivetrain: Drivetrain, elevator: Elevator, arm: Arm, manipulator: Manipulator) =
            run {
                (drivetrain.poseEstimator.estimatedPosition)
                // start
                    // move arm into position
                    SetPosition.high(elevator, arm)
                        .withTimeout(3.0)
                    //eject cube
                    .andThen(SetManipulatorSpeed(manipulator, -1.0, true).withTimeout(1.0))
                    // stop manipulator and move to idle
                    .andThen(
                        Idle(elevator, arm).alongWith(SetManipulatorSpeed(manipulator, 0.0, true))
                            // while this is happening, wait and then begin movement to fit position 1
                            .alongWith(WaitCommand(0.5)
                                .andThen(MoveToPosition(drivetrain, flipped(3.22), 0.73,45.0).withTimeout(1.5))
                                .andThen(MoveToPosition(drivetrain, flipped(4.8),.66, 180.0).withTimeout(1.5))
                            ).withTimeout(3.6)
                    )
                    // move into deploy position and deploy
                    .andThen(
                        MoveToPosition(drivetrain, flipped(5.72), 0.94, 180.0).withTimeout(1.0)
                            // while moving, deploy
                            .alongWith(
                                IntakePositionForward(elevator, arm)
                                    .alongWith(SetManipulatorSpeed(manipulator, 1.0, true)
                                        .withTimeout(1.5)
                                    )
                            )
                            // withtimeout of total depoy time
                            .withTimeout(1.75)
                            //and then move to the should have intaked position
                            .andThen(MoveToPosition(drivetrain, flipped(6.66),0.93).withTimeout(2.0))
                    )
                    .andThen(
                        Idle(elevator, arm).alongWith(SetManipulatorSpeed(manipulator, 0.1, false).withTimeout(5.0))
                            // start moving back
                            .alongWith(WaitCommand(0.5)
                                .andThen(MoveToPosition(drivetrain, flipped(4.8), .66,0.0).withTimeout(1.75))
                                .andThen(MoveToPosition(drivetrain, flipped(1.95), 1.05, 0.0).withTimeout(3.5))
                                .andThen(MoveToPosition(drivetrain, flipped(1.87),1.05, 0.0).withTimeout(1.0))
                            ).withTimeout(6.0)
                    )
                    //place
                    .andThen(SetPosition.high(elevator, arm)
                        .withTimeout(3.0))
                    .andThen(SetManipulatorSpeed(manipulator, -1.0, true).withTimeout(1.0))
                    .withTimeout(15.0)

            }

        fun snapToYValue(
            drivetrain: Drivetrain,
            y: () -> Double,
            yTolerance: Double = 0.1,
            r: () -> Rotation2d = { drivetrain.poseEstimator.estimatedPosition.rotation },
            rTolerance: Double = 0.1,
        ) =
            run {
                (drivetrain.poseEstimator.estimatedPosition)
                MoveToPosition(
                    drivetrain,
                    {
                        Pose2d(
                            drivetrain.poseEstimator.estimatedPosition.translation.x,
                            y(),
                            r()
                        )
                    },
                    toleranceppos = yTolerance,
                    tolerancerpos = rTolerance
                ).withTimeout(3.5)
            }

        /**
         * @param rotValues The angle in radians
         */
        fun snapToScoring(drivetrain: Drivetrain, yValues: () -> Iterable<Double>, rotValues: () -> Iterable<Double>): Command =
            snapToYValue(
                drivetrain,
                {yValues().minByOrNull { value ->
                    (value - drivetrain.poseEstimator.estimatedPosition.y).absoluteValue
                }?: drivetrain.poseEstimator.estimatedPosition.y},
                yTolerance = 0.05,
                {
                    Rotation2d.fromRadians(
                        rotValues().minByOrNull { value ->
                            (value.mod(PI*2) - drivetrain.poseEstimator.estimatedPosition.rotation.radians.mod(PI*2)).absoluteValue
                        }?: drivetrain.poseEstimator.estimatedPosition.rotation.radians
                    )
                },
                rTolerance = 0.15
            )

    }
}

//this function should be used when copying and pasting an auto function, and you need to flip the x-coordinates.
fun flipped(x: Double): Double {
    val newX: Double
    if(x<8.3)
        newX=8.3+(8.3-x)
    else
        newX=8.3-(x-8.3)

    return newX
}