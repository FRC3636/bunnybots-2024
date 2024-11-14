package com.frcteam3636.frc2024.subsystems.indexer

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Indexer: Subsystem {
    private var io: IndexerIO = IndexerIOReal()

    var inputs = IndexerIO.IndexerInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)

        DriverStation.getAlliance().map {
            if (
                (inputs.balloonState == BalloonState.Blue && it == DriverStation.Alliance.Blue)
                || (inputs.balloonState == BalloonState.Red && it == DriverStation.Alliance.Red)
            ) {
                progressBalloon()
            } else {
                outtakeBalloon()
            }
        }

    }

    fun progressBalloon(): Command =
        startEnd(
            {io.setSpinSpeed(0.5)},
            {io.setSpinSpeed(0.0)}
        )

    fun outtakeBalloon(): Command =
        startEnd(
            {io.setSpinSpeed(-0.5)},
            {io.setSpinSpeed(0.0)}
        )
}