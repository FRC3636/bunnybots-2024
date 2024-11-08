package com.frcteam3636.frc2024.subsystems.indexer

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import kotlin.doubleArrayOf
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.


interface IndexerIO{
    class IndexerInputs : LoggableInputs {
        var indexerVelocity = Rotation2d()
        var indexerCurrent: Double = 0.0
        var isSpinningBalloon: Boolean = false
        var hasBalloon: Boolean = false
        var balloonColor: Color = Color.kblack

        override fun toLog(table: LogTable?) {
            table?.put("Indexer Wheel Velocity", indexerVelocity)
            table?.put("Indexer Wheel Current", indexerCurrent)
            table?.put("Is spinning", isSpinningBalloon)
            table?.put("Has balloon", hasBalloon)
            table?.put("Balloon Color", doubleArrayOf(balloonColor.red, balloonColor.green, balloonColor.blue))
        }

        override fun fromLog(table: LogTable) {
            indexerVelocity = table.get("Indexer Velocity", indexerVelocity)!![0]
            indexerCurrent = table.get("Indexer Wheel Current", indexerCurrent)
            isSpinningBalloon = table.get("Is spinning", isSpinningBalloon)
            hasBalloon = table.get("Has balloon", hasBalloon)
            val balloonColorArray: DoubleArray = table.get("Balloon Color", doubleArrayOf(balloonColor.red, balloonColor.green, balloonColor.blue))
            balloonColor = Color(balloonColorArray[0], balloonColorArray[1], balloonColorArray[2])
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

    private val colorSensorDistance = NetworkTables.getEntry("/proximity1")

    private val colorSensorColor = NetworkTables.getEntry("/rawcolor1")

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.indexerVelocity = Rotation2d(indexerWheel.encoder.velocity)
        inputs.indexerCurrent = indexerWheel.outputCurrent
        inputs.isSpinningBalloon = colorSensor.get()
        inputs.hasBalloon = colorSensorDistance.getValue() > 1000
        inputs.balloonColor = colorSensorColor.getValue()
    }

    override fun setSpinSpeed(speed: Double) {
        indexerWheel.set(speed)
    }

    internal companion object Constants {
        const val COLOR_SENSOR_PORT = i2cPort
        const val RED_BALLOON_COLOR = (255, 0, 0)
        const val BLUE_BALLOON_COLOR = (0, 0, 255)
    }
}