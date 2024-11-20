package com.frcteam3636.frc2024.subsystems.indexer

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.utils.LimelightHelpers
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim

public enum class BalloonState {
    Blue,
    Red,
    None
}

interface IndexerIO {
    class Inputs : LoggableInputs {
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
    fun updateInputs(inputs: Inputs)

    fun setSpinSpeed(speed: Double)
    // percent of full speed
}

class IndexerIOReal : IndexerIO{
    companion object Constants {
        const val RED_CLASS  = "red";
        const val BLUE_CLASS = "blue";
        const val NONE_CLASS = "none";
    }

    private var indexerMotor =
        CANSparkFlex(
            REVMotorControllerId.IndexerMotor,
            CANSparkLowLevel.MotorType.kBrushless
        )

    override fun updateInputs(inputs: IndexerIO.Inputs) {
        inputs.indexerVelocity = Rotation2d(indexerMotor.encoder.velocity)
        inputs.indexerCurrent = indexerMotor.outputCurrent

        when (val colorClass = LimelightHelpers.getClassifierClass("limelight-sensor")) {
            RED_CLASS -> inputs.balloonState = BalloonState.Red;
            BLUE_CLASS -> inputs.balloonState = BalloonState.Blue;
            NONE_CLASS -> inputs.balloonState = BalloonState.None;
            else -> throw AssertionError("Unknown balloon class: $colorClass")
        }
    }

    override fun setSpinSpeed(speed: Double) {
        assert(speed in -1.0..1.0)
        indexerMotor.set(speed)
    }
}

class IndexerIOSim: IndexerIO {
    val flywheelSim = FlywheelSim(
        DCMotor.getNeoVortex(1),
        1.0,
        1.0
    )

    override fun updateInputs(inputs: IndexerIO.Inputs) {
        flywheelSim.update(Robot.period)
        inputs.indexerVelocity = flywheelSim.angularVelocityRadPerSec
    }

    override fun setSpinSpeed(speed: Double) {
        TODO("Not yet implemented")
    }

}