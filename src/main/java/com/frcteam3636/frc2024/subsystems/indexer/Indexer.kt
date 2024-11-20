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
    }

    /**
     * Runs the indexer forward if balloon matches the current alliance.
     * Does not run if no balloon.
     * Reverses if balloon is wrong alliance.
     */
    fun autoRun(): Command =
        runEnd(
            {
                DriverStation.getAlliance().map {
                    if (
                        (inputs.balloonState == BalloonState.Blue && it == DriverStation.Alliance.Blue)
                        || (inputs.balloonState == BalloonState.Red && it == DriverStation.Alliance.Red)
                    ) {
                        io.setSpinSpeed(0.5)
                    } else if (inputs.balloonState == BalloonState.None) {
                        io.setSpinSpeed(0.0)
                    } else {
                        io.setSpinSpeed(-0.5)
                    }
                }
            },
            {
                io.setSpinSpeed(0.0)
            }
        )

    fun outtakeBalloon(): Command =
        startEnd(
            {io.setSpinSpeed(-0.5)},
            {io.setSpinSpeed(0.0)}
        )
}