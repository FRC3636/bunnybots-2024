package com.frcteam3636.frc2024.subsystems.indexer

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.utils.LimelightHelpers
import com.frcteam3636.frc2024.utils.math.TAU
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.simulation.FlywheelSim

enum class BalloonState {
    Blue,
    Red,
    None
}

interface IndexerIO {
    class Inputs : LoggableInputs {
        var indexerVelocity = RotationsPerSecond.zero()
        var indexerCurrent = Amps.zero()
        var balloonState: BalloonState = BalloonState.None
        var position = Radians.zero()

        override fun toLog(table: LogTable?) {
            table?.put("Indexer Wheel Velocity", indexerVelocity)
            table?.put("Indexer Wheel Current", indexerCurrent)
            table?.put("Balloon Color", balloonState)
            table?.put("Indexer Wheel Angle", position)
        }

        override fun fromLog(table: LogTable) {
            indexerVelocity = table.get("Indexer Velocity", indexerVelocity)
            indexerCurrent = table.get("Indexer Wheel Current", indexerCurrent)
            balloonState = table.get("Balloon Color", balloonState)
            position = table.get("Indexer Wheel Angle", position)
        }
    }
    fun updateInputs(inputs: Inputs)

    fun setSpinSpeed(speed: Double)
    // percent of full speed
}

class IndexerIOReal : IndexerIO{
    companion object Constants {
        const val RED_CLASS  = "red"
        const val BLUE_CLASS = "blue"
        const val NONE_CLASS = "none"
    }

    private var indexerMotor =
        CANSparkFlex(
            REVMotorControllerId.IndexerMotor,
            CANSparkLowLevel.MotorType.kBrushless
        )

    override fun updateInputs(inputs: IndexerIO.Inputs) {
        inputs.indexerVelocity = Rotations.per(Minute).of(indexerMotor.encoder.velocity)
        inputs.indexerCurrent = Amps.of(indexerMotor.outputCurrent)
        inputs.position = Rotations.of(indexerMotor.encoder.position)

        when (val colorClass = LimelightHelpers.getClassifierClass("limelight-sensor")) {
            RED_CLASS -> inputs.balloonState = BalloonState.Red
            BLUE_CLASS -> inputs.balloonState = BalloonState.Blue
            NONE_CLASS -> inputs.balloonState = BalloonState.None
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
        inputs.indexerVelocity = RadiansPerSecond.of(flywheelSim.angularVelocityRadPerSec)
        inputs.position += Radians.of(flywheelSim.angularVelocityRadPerSec * Robot.period)
        inputs.position = Radians.of(inputs.position.`in`(Radians).mod(TAU))
    }

    override fun setSpinSpeed(speed: Double) {
        assert(speed in -1.0..1.0)
        flywheelSim.setInputVoltage(speed*12.0)
    }
}

class IndexerIOPrototype: IndexerIO {
    override fun updateInputs(inputs: IndexerIO.Inputs) {
    }

    override fun setSpinSpeed(speed: Double) {
    }

}