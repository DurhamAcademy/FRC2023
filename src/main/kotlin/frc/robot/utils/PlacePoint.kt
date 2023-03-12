package frc.robot.utils

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.constants.Field2dLayout

enum class ObjectType {
    Cone, Cube
}

enum class PlacePoint {
    Level1, Level2, Level3;
    fun asXInt(aliance: DriverStation.Alliance): Double? {
        val offset = when (this) {
            Level1 -> 0.0
            Level2 -> 0.0
            Level3 -> 0.0
        }
        return when (aliance) {
            Blue -> Field2dLayout.Axes.XInt.communityPlacementLineBlue
            Red -> Field2dLayout.Axes.XInt.communityPlacementLineRed
            else -> null
        }
    }
}

enum class Row {
    LClose, MClose, RClose,
    LMid, MMid, RMid,
    LFar, MFar, RFar;
    val asYInt: Double?
        get() = Field2dLayout.Axes.YInt.score
            .elementAtOrNull(this.ordinal)
}

enum class RowSection {
    Close, Mid, Far;
    fun withRowSide(rowSide: RowSide) = when (this) {
        Close -> when (rowSide) {
            RowSide.Left -> Row.LClose
            RowSide.Middle -> Row.MClose
            RowSide.Right -> Row.RClose
        }
        Mid -> when (rowSide) {
            RowSide.Left -> Row.LMid
            RowSide.Middle -> Row.MMid
            RowSide.Right -> Row.RMid
        }
        Far -> when (rowSide) {
            RowSide.Left -> Row.LFar
            RowSide.Middle -> Row.MFar
            RowSide.Right -> Row.RFar
        }

        else -> {
            throw IllegalArgumentException("RowSection $this is not supported")}
    }
}

enum class RowSide {
    Left, Middle, Right;
    fun withRowSection(rowSection: RowSection) =
        rowSection.withRowSide(this)
}

/**
 * state diagram for robot commands
 *
 * Key:
 *    A ->> B = when A is pressed, start running the B command
 *    [indent] = sub-state, actions that can be taken when in the super-state
 *    A -|-> B = while A is pressed, run the B command (interruptible)
 *    A -> B = after command A is finished, run command B
 *
 * X -|-> Idle
 *     A ->> PrepareToPlaceLow
 *         B ->> ReturnToIdle
 *         A ->> PlaceLow -> ReturnToIdle
 *         PovUp ->> PrepareToPlaceMid
 *         PovDown ->> Intake (because if we are at low level then just go further down)
 *     PovUp ->> PrepareToPlaceMid
 *
 */

class RobotCommandState {
    var currentObject: GamePiece = GamePiece.cone
    var placePoint: PlacePoint = PlacePoint.Level1
    var row: Row? = null

}