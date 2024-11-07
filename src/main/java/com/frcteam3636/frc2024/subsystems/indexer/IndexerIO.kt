package com.frcteam3636.frc2024.subsystems.indexer

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IndexerIO{
    class IndexerInputs : LoggableInputs {
        var indexerVelocity = Rotation2d()
        var indexerCurrent: Double = 0.0
        var isSpinningBalloon: Boolean = false
        var hasBalloon: Boolean = false
        var balloonColor = null

        override fun toLog(table: LogTable?) {
            table?.put("Indexer Wheel Velocity", indexerVelocity)
            table?.put("Indexer Wheel Current", indexerCurrent)
            table?.put("Is spinning", isSpinningBalloon)
            table?.put("Has balloon", hasBalloon)
            table?.put("Balloon Color", balloonColor)
        }

        override fun fromLog(table: LogTable) {
            indexerVelocity = table.get("Indexer Velocity", indexerVelocity)!![0]
            indexerCurrent = table.get("Indexer Wheel Current", indexerCurrent)
            isSpinningBalloon = table.get("Is spinning", isSpinningBalloon)
            hasBalloon = table.get("Has balloon", hasBalloon)
            balloonColor = table.get("Balloon Color", balloonColor)
        }
    }
    fun updateInputs(inputs: IndexerInputs)

    fun setSpinSpeed(speed: Double)
    // percent of full speed
}

class IndexerIOReal: IndexerIO{
    private var indexerWheel =
        CANSparkFlex(
            REVMotorControllerId.UnderTheBumperIntakeRoller,
            CANSparkLowLevel.MotorType.kBrushless
        )

    private var colorSensor: DigitalInput = DigitalInput(COLOR_SENSOR_PORT)

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.indexerVelocity = Rotation2d(indexerWheel.encoder.velocity)
        inputs.indexerCurrent = indexerWheel.outputCurrent
        inputs.isSpinningBalloon = colorSensor.get()
        inputs.hasBalloon = colorSensor.get()
        inputs.balloonColor = colorSensor.get()
    }

    override fun setSpinSpeed(speed: Double) {
        indexerWheel.set(speed)
    }

    internal companion object Constants {
        const val COLOR_SENSOR_PORT = 0
        const val RED_BALLOON_COLOR = (255, 0, 0)
        const val BLUE_BALLOON_COLOR = (0, 0, 255)
    }
}