package com.frcteam3636.frc2024.subsystems.indexer

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Indexer {
    private var io: IndexerIO = IndexerIOReal()

    var inputs = IndexerIO.IndexerInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)
    }

    fun progressBalloon(): Command =
        startEnd(
            {io.setSpinSpeed(0.5)}
        )

    fun outtakeBalloon(): Command =
        startEnd(
            {io.setSpinSpeed(-0.5)}
        )

    fun stopIndexerSpin(): Command = 
        startEnd(
            {io.setSpinSpeed(0.0)}
        )
}