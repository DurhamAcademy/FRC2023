package frc.robot.subsystems

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.InstantCommand

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
                .withWidget("Boolean Box")
                .withPosition(x, y)
                .withSize(1, 1)
                .entry
        }
    }
    var selected: Pair<Int, Int> = 0 to 0
        set(value) {
            // set the old field to false, wrappping around if needed
            gridEntries[field.first % 9][field.second % 3].setBoolean(false)
            // set the new field to true, wrapping around if needed
            gridEntries[value.first % 9][value.second % 3].setBoolean(true)
            field = value
        }

    fun moveCommand(x: Int, y: Int) = InstantCommand({
        selected = selected.first + x to selected.second + y
    })
}