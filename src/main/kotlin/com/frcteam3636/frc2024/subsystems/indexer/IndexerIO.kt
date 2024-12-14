package com.frcteam3636.frc2024.subsystems.indexer

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.subsystems.indexer.IndexerIOReal.Constants
import com.frcteam3636.frc2024.utils.LimelightHelpers
import com.frcteam3636.frc2024.utils.math.TAU
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

enum class BalloonState {
    Blue,
    Red,
    None
}

@Logged
open class IndexerInputs {
    var indexerVelocity = RotationsPerSecond.zero()!!
    var indexerCurrent = Amps.zero()!!
    var balloonState: BalloonState = BalloonState.None
    var position = Radians.zero()!!
}

interface IndexerIO {
    fun updateInputs(inputs: IndexerInputs)

    fun setVoltage(voltage: Double)
    // percent of full voltage
}

class IndexerIOReal : IndexerIO{
    companion object Constants {
        const val RED_CLASS  = "0 red"
        const val BLUE_CLASS = "1 blue"
        const val NONE_CLASS = ""
    }

    private var indexerMotor =
        CANSparkFlex(
            REVMotorControllerId.IndexerMotor,
            CANSparkLowLevel.MotorType.kBrushless
        )


    override fun updateInputs(inputs: IndexerInputs) {
        inputs.indexerVelocity = Rotations.per(Minute).of(indexerMotor.encoder.velocity)
        inputs.indexerCurrent = Amps.of(indexerMotor.outputCurrent)
        inputs.position = Rotations.of(indexerMotor.encoder.position)

        val colorClass = LimelightHelpers.getClassifierClass("limelight")
        inputs.balloonState = when (colorClass) {
            RED_CLASS -> BalloonState.Red
            BLUE_CLASS -> BalloonState.Blue
            NONE_CLASS -> BalloonState.None
            else -> throw AssertionError("Unknown balloon class: $colorClass")
        }
    }

    override fun setVoltage(voltage: Double) {
        assert(voltage in -12.0..12.0)
        Logger.recordOutput("/Indexer/OutputVoltage", voltage)
        indexerMotor.setVoltage(voltage)
    }
}

class IndexerIOSim: IndexerIO {
    val flywheelSim = FlywheelSim(
        DCMotor.getNeoVortex(1),
        1.0,
        1.0
    )

    override fun updateInputs(inputs: IndexerInputs) {
        flywheelSim.update(Robot.period)
        inputs.indexerVelocity = RadiansPerSecond.of(flywheelSim.angularVelocityRadPerSec)
        inputs.position += Radians.of(flywheelSim.angularVelocityRadPerSec * Robot.period)
        inputs.position = Radians.of(inputs.position.`in`(Radians).mod(TAU))
    }

    override fun setVoltage(voltage: Double) {
        assert(voltage in -1.0..1.0)
        flywheelSim.setInputVoltage(voltage*12.0)
    }
}

class IndexerIOPrototype: IndexerIO {
    override fun updateInputs(inputs: IndexerInputs) {
    }

    override fun setVoltage(voltage: Double) {
    }
}