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

    override fun updateInputs(inputs: IndexerInputs) {
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

    override fun updateInputs(inputs: IndexerInputs) {
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
    override fun updateInputs(inputs: IndexerInputs) {
    }

    override fun setSpinSpeed(speed: Double) {
    }

}