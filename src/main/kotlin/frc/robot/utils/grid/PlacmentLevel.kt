package frc.robot.utils.grid

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.constants.Field2dLayout
import frc.robot.constants.Field2dLayout.Axes

object GridConstants {
    const val centerDistX: Double = 6.87//feetToMeters(22.625)
    const val centerX: Double = Field2dLayout.xCenter
}

/**
 * This enum represents the different floor game pieces and their positions
 *
 * @param x the x value of the object
 * @param y the y value of the object
 */
enum class FloorGamePiecePosition(val x: Double, val y: Double){
    Closest(inchesToMeters(47.36), inchesToMeters(36.19)), //x 6.934 y 36.19 in
    MiddleClose(inchesToMeters(47.36), inchesToMeters(84.19)), // 47.36in y + 48in
    MiddleFar(inchesToMeters(47.36), inchesToMeters(132.19)), //y 3.517
    Farthest(inchesToMeters(47.36), inchesToMeters(180.19)), //
}
/**
 * This enum represents the different levels of the field.
 *
 * @param depth the distance from the metal bars at the front of the grid to
 * the center of the pole/center of the cube placement area in the group.
 * @param height the height of the level from the ground to the center of the
 * cube placement area in the group.
 */
enum class PlacmentLevel(val depth: Double, val height: Double) {
    Level1(inchesToMeters(13.0/2), inchesToMeters(0.0)),
    Level2(inchesToMeters(22.0), inchesToMeters(33.75)),
    Level3(inchesToMeters(40.0), inchesToMeters(45.75)),
}

/**
 * This enum represents the identical groups of placment areas on the field.
 * It does not specify the alliance, or if you are placing on the left, right,
 * or center of the group. ( left, right are for cones, center is for cubes )
 *
 * @param offset the offset from the field wall (on the side of the field that
 * is closest to the judging table) to the center of the cube placement area in
 * the group.
 */
enum class PlacementGroup(val offset: Double) {
    /** The group of game pieces that are closest to the judging table. */
    Closest(inchesToMeters(42.0/* + 16*/)),

    /** The group of game pieces that are farthest from the judging table. */
    Farthest(inchesToMeters(174.0/* + 2*/)),

    /** The group of game pieces that are in front of the charging station. */
    Middle(inchesToMeters(108.0/* + 9*/)),
}

enum class PlacementSide(val offset: Double) {
    /**
     * The cone placement area closest to the judging table in a group.
     */
    CloseCone(inchesToMeters(22.0)),
    /** The cube placement area, in between the two cone placement areas and in
     * the middle of the group.
     */
    Cube(0.0),
    /** The cone placement area farthest from the judging table in a group.
     */
    FarCone(inchesToMeters(-22.0)),
}

/**
 * This function can calculate the position of all placement points on the field.
 * It specifies the alliance, and if you are placing on the left, right,
 * or center of the group. ( left, right are for cones, center is for cubes )
 * It also specifies the level of the field.
 * It can calculate the transform from the placement specifications to the
 * odometry.
 *
 * @param level the level of the field (ie: placement level).
 * @param group the group of the field.
 * @param side the side of the group.
 */
fun getPlacementTransform(
    level: PlacmentLevel,
    group: PlacementGroup,
    side: PlacementSide,
    alliance: DriverStation.Alliance = DriverStation.getAlliance()
): Translation3d {
    val allianceMultiplier = when (alliance) {
        Blue -> Axes.Blue.fieldOffsetMultiplier
        Red -> Axes.Red.fieldOffsetMultiplier
        else -> throw IllegalArgumentException("Alliance is not Blue or Red")
    }// fixme: refactor to use alliance.mulitplier
    val x = GridConstants.centerX + (GridConstants.centerDistX * allianceMultiplier) + level.depth
    val y = side.offset + group.offset
    val z = level.height
    return Translation3d(x,y,z)//fixme BADDD PLEASE FIX
}