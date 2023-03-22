package frc.robot.utils.grid

import frc.robot.subsystems.Drivetrain
import kotlin.math.abs

class FindClosestNode {
    //yes this method is silly but idk what else to do and this works hehe
    fun nearestNode(drivetrain: Drivetrain) : Double{
        var distances: DoubleArray = doubleArrayOf(
            abs(drivetrain.estimatedPose2d.y - PlacementGroup.Closest.offset),
            abs(drivetrain.estimatedPose2d.y - PlacementGroup.Middle.offset),
            abs(drivetrain.estimatedPose2d.y - PlacementGroup.Farthest.offset)
        )
        if(distances.min() == distances[0]) return PlacementGroup.Closest.offset
        if(distances.min() == distances[1]) return PlacementGroup.Middle.offset
        if(distances.min() == distances[2]) return PlacementGroup.Farthest.offset
        else return -1.0
    }
}