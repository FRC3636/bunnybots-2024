package com.frcteam3636.frc2024.subsystems.indexer

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.networktables.NetworkTableInstance
import kotlin.doubleArrayOf
import edu.wpi.first.wpilibj.util.Color



interface IndexerIO{
    class IndexerInputs : LoggableInputs {
        var indexerVelocity = Rotation2d()
        var indexerCurrent: Double = 0.0
        var hasBalloon: Boolean = false
        var balloonColor: Color = Color.kBlack

        override fun toLog(table: LogTable?) {
            table?.put("Indexer Wheel Velocity", indexerVelocity)
            table?.put("Indexer Wheel Current", indexerCurrent)
            table?.put("Has balloon", hasBalloon)
            table?.put("Balloon Color", doubleArrayOf(balloonColor.red, balloonColor.green, balloonColor.blue))
        }

        override fun fromLog(table: LogTable) {
            indexerVelocity = table.get("Indexer Velocity", indexerVelocity)!![0]
            indexerCurrent = table.get("Indexer Wheel Current", indexerCurrent)
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

    private val colorSensorDistance = NetworkTableInstance.getDefault().getEntry("/proximity1")

    private val colorSensorColor = NetworkTableInstance.getDefault().getEntry("/rawcolor1")

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.indexerVelocity = Rotation2d(indexerWheel.encoder.velocity)
        inputs.indexerCurrent = indexerWheel.outputCurrent
        inputs.hasBalloon = colorSensorDistance.getDouble(0.0) > 1000.0
        val colorArray = colorSensorColor.getDoubleArray(doubleArrayOf(0.0,0.0,0.0))
        inputs.balloonColor = Color(colorArray[0], colorArray[1], colorArray[2])
    }

    override fun setSpinSpeed(speed: Double) {
        indexerWheel.set(speed)
    }
}