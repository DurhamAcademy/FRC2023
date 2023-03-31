package frc.robot.commands.drivetrain

/*
fun fullBalanceSequence(
    drivetrain: DrivetrainConstants,
    fromCenter: Boolean = false,
): Command {
    var state: Int = 0 // 0 = start, 1 = get on balance, 2 = balance

    return ConditionalCommand(
        ConditionalCommand(
            // get on balance, so drive until abs(pitch) < 11
            DriveCommand(
                drivetrain,
                x = { 2.0 },
                y = { 0.0 },
                rotation = { 0.0 },
                isFieldOriented = false,
                centerOffset = { Translation2d(0.0, 0.0) }
            ).until {
                drivetrain.gyro.pitch.absoluteValue < 11.0
            }.andThen(InstantCommand({ state = 2 })),
            // auto balance
            AutoBalance(drivetrain, getup = false, fromCenter = fromCenter, speed = 0.25),
            { state == 1 }
        ).andThen(InstantCommand({ state = 0 })),
        // start
        DriveCommand(
            drivetrain,
            x = { 0.0 },
            y = { 0.0 },
            rotation = { 1.0 },
            isFieldOriented = false,
            centerOffset = { Translation2d(0.0, 0.0) }
        ).withTimeout(0.5).andThen(InstantCommand({ state = 1 })),
        { state == 1 }
    )
}
        */