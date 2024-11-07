package com.frcteam3636.frc2024.subsystems.indexer

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IndexerIO{
    class IndexerInputs : LoggableInputs {
        var indexerVelocity = Rotation2d()
        var indexerCurrent: Double = 0.0
        var isSpinning: Boolean = false
        var balloonColor = null

        override fun toLog(table: LogTable?) {
            table?.put("Indexer Wheel Velocity", indexerVelocity)
            table?.put("Indexer Wheel Current", indexerCurrent)
            table?.put("Is spinning", isSpinning)
            table?.put("Balloon Color", balloonColor)
        }

        override fun fromLog(table: LogTable) {
            indexerVelocity = table.get("Indexer Velocity", indexerVelocity)!![0]
            indexerCurrent = table.get("Indexer Wheel Current", indexerCurrent)
            isSpinning = table.get("Is spinning", isSpinning)
            balloonColor = table.get("Balloon Color", balloonColor)
        }
    }
    fun updateInputs(inputs: IndexerInputs)

    fun setSpinSpeed(speed: Double)
    // percent of full speed
}