package frc.robot.utils

enum class ObjectType {
    Cone, Cube
}

enum class PlacePoint {
    Level1, Level2, Level3
}

enum class Row {
    L1, M1, R1,
    L2, M2, R2,
    L3, M3, R3
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