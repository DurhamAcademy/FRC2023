package frc.robot

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import kotlin.math.PI

object Constants {
    object leds {
        val count = 74// 74 is what chris said
    }

    init {
        SmartDashboard.setDefaultBoolean("Full DS Control", false)
    }

    var fullDSControl: Boolean
        get() = SmartDashboard.getBoolean("Full DS Control", false)
        set(value: Boolean) {
            SmartDashboard.putBoolean("Full DS Control", value)
        }

    object encoder {
        const val id = 19
        const val offset = 14.0
        const val inverted = true
    }

    val simArmLength = inchesToMeters(10.0)
    const val minAngle = -PI / 2
    const val maxAngle = PI / 2
    const val armMass = 1.0
    const val momentOfInertia = 1.0

    object Elevator {
        val carriageMass = lbsToKilograms(27.5) // rough estimate (25-30 lbs)
        val encoderDistancePerPulse: Double = 1.0 / 2_048.0
        val sproketRadius = inchesToMeters(1.273 / 2.0)

        object elevatorMotor {
            object Feedforward {
                val kG: Double = .32348
                val kS = .053817
                val kV = 10.248
                val kA = .28817
            }

            object tolerance {
                const val positionTolerance: Double = 0.05
                const val velocityTolerance: Double = 0.05
            }
            //todo add encoder offset to smartdashboard

            val inverted = true
            val gearRatio: Double = 1 / 10.51

            object PID {
                val kP = 32.5 / 2
                val kI = 0.0
                val kD: Double = 3.429

                object TrapezoidProfile {
                    val maxVelocity: Double = 1.0
                    val maxAcceleration: Double = 1.0
                }
            }

            val ElevatorMotorId: Int = 32
        }

        object limitSwitch {
            val ElevatorLimitSwitchId: Int = 9
            val offset: Double = Elevator.limits.bottomLimit//0.0
        }

        object limits {
            val bottomLimit: Double = inchesToMeters(20.0)
            val topLimit: Double = inchesToMeters(50.0)
        }
    }

    const val maxDriveAcceleration = 3.0
    const val powerPercent = 1.0

    const val BRZeroAngle = -10.0 - 5
    const val BLZeroAngle = -29.7 + 2.3
    const val FRZeroAngle = -102.2 - 3.7
    const val FLZeroAngle = -47.9 + 1.3
    const val FRDriveMotorId = 10//fr
    const val BLDriveMotorId = 11//bl
    const val FLDriveMotorId = 12//fl
    const val BRDriveMotorId = 13

    const val FRTurnMotorId = 14
    const val BLTurnMotorId = 15
    const val FLTurnMotorId = 16
    const val BRTurnMotorId = 17

    const val FRTurnEncoderId = 6
    const val BLTurnEncoderId = 7
    const val FLTurnEncoderId = 8
    const val BRTurnEncoderId = 9

    const val WHEEL_RADIUS = .0508
    const val WHEEL_CIRCUMFRENCE = WHEEL_RADIUS * 2 * PI
    const val DRIVE_GEAR_RATIO = 6.75

    const val MODULE_DISTANCE_X = 0.641
    const val MODULE_DISTANCE_Y = 0.539750

    object arm {
        object motor {
            const val positionTolerance = 0.05
            const val velocityTolerance = 0.05
            const val id = 34
            const val currentLimit = 40
            const val inverted = true
            const val gearRatio = 1 / 170.67
            const val maxVelocity = 1.0
            const val maxAcceleration = 1.0
            const val kP = 2.6426
            const val kI = 0.0
            const val kD = 1.2626

            const val kS = 0.086989
            const val kG = 0.42677
            const val kV = 3.2483
            const val kA = 0.34796
        }

        object encoder {
            const val id = 18
            const val offset = -170.2441528786//-76.15 + 17 + 25 - 5 - 5 + 77//-87.49
            const val inverted = true
        }

        val length = inchesToMeters(30.31)
        val maxAngle = degreesToRadians(130.0 - 5.0)
        val minAngle = degreesToRadians(-155.0 + 5.0)
        const val armMass = 4.76//lbsToKilograms(10.5)

        // 640.1 in^2 lbs
        const val momentOfInertia = 0.187318642 //kg m^2
    }

    val cadToCode = Transform3d(
        Translation3d(0.0, 0.029, 0.0),
        Rotation3d(0.0, 0.0, PI / 2)
    )

    object manipulator {
        const val motorId = 31
        const val leftSolenoidForward = 0
        const val leftSolenoidReverse = 1
        const val rightSolenoidForward = 3
        const val rightSolenoidReverse = 2
        const val manipulatorCurrentLimit = 20.0

        val wristToObj = inchesToMeters(8.0)
        const val confidenceThreshold = 0.75

        object Colors {
            val purpleCube = Color(0.19, 0.07, 0.77)
            val yellowCone = Color(0.95, 0.77, 0.06)
        }
    }

    object VisionConstants {
        const val cameraName: String = "OV9281"
        val robotToCam: Transform3d = Transform3d(
            Translation3d(-0.258, 0.0, 0.137),
            Rotation3d(
                0.0,
                degreesToRadians(-12.0),
                degreesToRadians(180.0)
            )
        )
    }

    @Suppress("unused") //TODO: Remove this suppression
    object FieldConstants {
        val heightLimit = feetToMeters(8.5) //FIXME: is this correct
        const val width = 3.048
        const val length = 5.486
    }

    const val DRIVE_P = 2.37
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0

    const val driveKS = 0.20285
    const val driveKV = 2.2335
    const val driveKA = 0.34271

    const val ANGLE_P = 0.7 //fixme pid
    const val ANGLE_I = 0.0
    const val ANGLE_D = 0.0

    const val angleKS = 0.44475
    const val angleKV = 0.25237
    const val angleKA = 0.01973

    @Suppress("unused") //TODO: Remove this suppression
    object Field2dLayout {
        //        val bounds = listOf(
//            // judge side red -> judge side blue -> far side blue -> far side red
//            Translation3d(8.25, -4.0, 0.0),
//            Translation3d(-8.25, -4.0, 0.0),
//            Translation3d(-8.25, 4.0, 0.0),
//            Translation3d(8.25, 4.0, 0.0)
//        )
//        val center = Translation3d(0.0, 0.0, 0.0)
//
//        object Areas {
//            val blueChargingStation = Area(
//                Translation3d(-3.5, 0.0, 0.0),
//                Translation3d(-5.5, 0.0, 0.0),
//                Translation3d(-5.5, 2.5, 0.0),
//                Translation3d(-3.5, 2.5, 0.0)
//            )
//            val redChargingStation = Area(
//                Translation3d(3.5, 0.0, 0.0),
//                Translation3d(5.5, 0.0, 0.0),
//                Translation3d(5.5, 2.5, 0.0),
//                Translation3d(3.5, 2.5, 0.0)
//            )
//            val blueCommunityZone = Area(
//                Translation3d(-5.0, -1.5, 0.0),
//                Translation3d(-5.0, -2.75, 0.0),
//                Translation3d(-1.6, -2.75, 0.0),
//                Translation3d(-1.6, -4.0, 0.0),
//                Translation3d(-8.25, -4.0, 0.0),
//                Translation3d(-8.25, -1.5, 0.0)
//            )
//            val redCommunityZone = Area(
//                Translation3d(5.0, -1.5, 0.0),
//                Translation3d(5.0, -2.75, 0.0),
//                Translation3d(1.6, -2.75, 0.0),
//                Translation3d(1.6, -4.0, 0.0),
//                Translation3d(8.25, -4.0, 0.0),
//                Translation3d(8.25, -1.5, 0.0)
//            )
//
//            val blueScoringZone = Area(
//                Translation3d(-5.0, -1.4, 0.0),
//                Translation3d(-8.25, -1.4, 0.0),
//                Translation3d(-8.25, -1.4, 0.0),
//                Translation3d(-8.25, -4.0, 0.0),
//                Translation3d(-3.4, -4.0, 0.0),
//                Translation3d(-3.4, 0.0, 0.0),
//                Translation3d(-5.0, 0.0, 0.0)
//            )
//            val redScoringZone = Area(
//                Translation3d(5.0, -1.4, 0.0),
//                Translation3d(8.25, -1.4, 0.0),
//                Translation3d(8.25, -1.4, 0.0),
//                Translation3d(8.25, -4.0, 0.0),
//                Translation3d(3.4, -4.0, 0.0),
//                Translation3d(3.4, 0.0, 0.0),
//                Translation3d(5.0, 0.0, 0.0)
//            )
//        }
//
        val size = Translation2d(16.5, 8.0)

        object Axes {
            object XInt {
                val communityPlacementLineBlue = 1.4
                val communityPlacementLineRed = size.x - communityPlacementLineBlue
                val loadingZonePlatformStartRed = .35
                val loadingZonePlatformStartBlue = size.x - loadingZonePlatformStartRed
                val loadingZoneStartRed = 3.35
                val loadingZoneStartBlue = size.x - loadingZoneStartRed
            }

            object YInt {
                private const val platform1 = 7.45
                private const val platform2 = 6.15
                private const val cone1Left = 4.975
                private const val cube1 = 4.425
                private const val cone1Right = 3.865
                private const val cone2Left = 3.305
                private const val cube2 = 2.75
                private const val cone2Right = 2.19
                private const val cone3Left = 1.63
                private const val cube3 = 1.065
                private const val cone3Right = .51
                const val barrier = 5.5
                val platforms = arrayOf(platform1, platform2)
                val cones = arrayOf(
                    cone1Left, cone1Right,
                    cone2Left, cone2Right,
                    cone3Left, cone3Right
                )
                val cubes = arrayOf(cube1, cube2, cube3)
                val score = arrayOf(
                    cone1Left, cube1, cone1Right,
                    cone2Left, cube2, cone2Right,
                    cone3Left, cube3, cone3Right
                )
            }

            interface AliancePointList {
                abstract val scoringPoints: List<Translation2d>
                abstract val loadingZonePlatforms: List<Translation2d>
                abstract val conePlacement: List<Translation2d>
                abstract val cubePlacement: List<Translation2d>
            }

            object Red : AliancePointList {
                const val fieldOffsetMultiplier = -1.0
                override val scoringPoints = YInt.score.map {
                    Translation2d(XInt.communityPlacementLineRed, it)
                }
                override val loadingZonePlatforms = YInt.platforms.map {
                    Translation2d(XInt.loadingZoneStartRed, it)
                }
                override val conePlacement = YInt.cones.map {
                    Translation2d(XInt.loadingZoneStartRed, it)
                }
                override val cubePlacement = YInt.cubes.map {
                    Translation2d(XInt.loadingZoneStartRed, it)
                }
            }

            object Blue : AliancePointList {
                const val fieldOffsetMultiplier = 1.0
                override val scoringPoints = YInt.score.map {
                    Translation2d(XInt.communityPlacementLineBlue, it)
                }
                override val loadingZonePlatforms = YInt.platforms.map {
                    Translation2d(XInt.loadingZoneStartBlue, it)
                }
                override val conePlacement = YInt.cones.map {
                    Translation2d(XInt.loadingZoneStartBlue, it)
                }
                override val cubePlacement = YInt.cubes.map {
                    Translation2d(XInt.loadingZoneStartBlue, it)
                }
            }

            fun closest(
                to: Translation2d,
                inIterable: Iterable<Translation2d>
            ): Translation2d? = inIterable.minByOrNull { it.getDistance(to) }
        }
    }
}
