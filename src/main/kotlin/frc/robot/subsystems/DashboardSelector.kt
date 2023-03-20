package frc.robot.subsystems

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementLevel
import frc.robot.utils.grid.PlacementSide
import frc.robot.utils.wrap
import kotlin.math.roundToInt

class DashboardSelector {
    val tab = Shuffleboard.getTab("Selector")

    /**
     * the grid that will hold the buttons
     * the grid is 9x3 and is a sendable grid
     */
    val grid = tab.getLayout("ClickGrid", BuiltInLayouts.kGrid)
        .withSize(6, 2)
        .withPosition(2, 0)
        .withProperties(
            mapOf(
                "Number of columns" to 9,
                "Number of rows" to 3,
                "Label position" to "HIDDEN",
            )
        )
    val gridEntries = List(9) { x ->
        List(3) { y ->
            grid.add("$x,$y", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(x, y)
                .withSize(1, 1)
                .entry
        }
    }
    val selectedXEntry = tab.add("Selected X", 0)
        .withWidget("Number Slider")
        .withProperties(
            mapOf(
                "min" to 0,
                "max" to 8,
                "Block increment" to 1
            )
        )
        .entry
    val selectedYEntry = tab.add("Selected Y", 0)
        .withWidget("Number Slider")
        .withProperties(
            mapOf(
                "min" to 0,
                "max" to 2,
                "Block increment" to 1
            )
        )
        .entry
    var selected: Pair<Int, Int> = 0 to 0
        set(value) {
            // set the old field to false, wrappping around if needed
            gridEntries[
                field.first.wrap(0, gridEntries.size)
            ][
                field.second.wrap(
                    0,
                    gridEntries[
                        field.first.wrap(0, gridEntries.size)
                    ].size
                )
            ]
                .setBoolean(false)
            // set the new field to true, wrapping around if needed
            gridEntries[
                value.first.wrap(0, gridEntries.size)
            ][
                value.second.wrap(
                    0,
                    gridEntries[
                        value.first.wrap(0, gridEntries.size)
                    ].size
                )
            ]
                .setBoolean(true)

            // set the selected x and y entries
            selectedXEntry.setInteger(value.first.toLong())
            selectedYEntry.setInteger(value.second.toLong())

            field = value
        }

    fun moveCommand(x: Int, y: Int) = InstantCommand({
        var newX = selected.first + x
        var newY = selected.second + y
        if (newX < 0) newX = gridEntries.size - 1
        if (newX >= gridEntries.size) newX = 0
        if (newY < 0) newY = gridEntries[newX].size - 1
        if (newY >= gridEntries[newX].size) newY = 0
        selected = newX to newY
    })

    fun update() {
        val selectedXEntryRounded = selectedXEntry.getDouble(0.0).roundToInt()
        if (selectedXEntryRounded != selected.first) {
            selected = selectedXEntryRounded to selected.second
        }
        val selectedYEntryRounded = selectedYEntry.getDouble(0.0).roundToInt()
        if (selectedYEntryRounded != selected.second) {
            selected = selected.first to selectedYEntryRounded
        }

        for (x in gridEntries.indices) {
            for (y in gridEntries[x].indices) {
                if (gridEntries[x][y].getBoolean(false) && selected != x to y) {
                    selected = x to y
                }
            }
        }
    }

    val placementLevel: PlacementLevel
        get() = when (selected.second) {
            0 -> PlacementLevel.Level1
            1 -> PlacementLevel.Level2
            2 -> PlacementLevel.Level3
            else -> PlacementLevel.Level1
        }

    val placementPosition: PlacementGroup
        get() = when (selected.first) {
            0, 1, 2 -> PlacementGroup.Closest
            3, 4, 5 -> PlacementGroup.Middle
            6, 7, 8 -> PlacementGroup.Farthest
            else -> PlacementGroup.Closest
        }

    val placementSide: PlacementSide
        get() = when (selected.first) {
            0, 3, 6 -> PlacementSide.CloseCone
            1, 4, 7 -> PlacementSide.Cube
            2, 5, 8 -> PlacementSide.FarCone
            else -> PlacementSide.CloseCone
        }
}