package frc.robot.constants

import edu.wpi.first.math.geometry.Translation2d

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
            val scoringPoints: List<Translation2d>
            val loadingZonePlatforms: List<Translation2d>
            val conePlacement: List<Translation2d>
            val cubePlacement: List<Translation2d>
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