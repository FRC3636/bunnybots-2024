package com.frcteam3636.frc2024.subsystems.indexer

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.utils.LimelightHelpers
import com.revrobotics.CANSparkLowLevel

public enum class BalloonState {
    Blue,
    Red,
    None
}

interface IndexerIO{
    class IndexerInputs : LoggableInputs {
        var indexerVelocity = Rotation2d()
        var indexerCurrent: Double = 0.0
        var balloonState: BalloonState = BalloonState.None

        override fun toLog(table: LogTable?) {
            table?.put("Indexer Wheel Velocity", indexerVelocity)
            table?.put("Indexer Wheel Current", indexerCurrent)
            table?.put("Balloon Color", balloonState)
        }

        override fun fromLog(table: LogTable) {
            indexerVelocity = table.get("Indexer Velocity", indexerVelocity)!![0]
            indexerCurrent = table.get("Indexer Wheel Current", indexerCurrent)
            balloonState = table.get("Balloon Color", balloonState)
        }
    }
    fun updateInputs(inputs: IndexerInputs)

    fun setSpinSpeed(speed: Double)
    // percent of full speed
}

class IndexerIOReal: IndexerIO{
    companion object Constants {
        const val RED_CLASS  = "red";
        const val BLUE_CLASS = "blue";
        const val NONE_CLASS = "none";
    }

    private var indexerWheel =
        CANSparkFlex(
            REVMotorControllerId.UnderTheBumperIntakeRoller,
            CANSparkLowLevel.MotorType.kBrushless
        )

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.indexerVelocity = Rotation2d(indexerWheel.encoder.velocity)
        inputs.indexerCurrent = indexerWheel.outputCurrent

        val colorClass = LimelightHelpers.getClassifierClass("limelight");
        when (colorClass) {
            RED_CLASS -> inputs.balloonState = BalloonState.Red;
            BLUE_CLASS -> inputs.balloonState = BalloonState.Blue;
            NONE_CLASS -> inputs.balloonState = BalloonState.None;
        }
    }

    override fun setSpinSpeed(speed: Double) {
        indexerWheel.set(speed)
    }
}