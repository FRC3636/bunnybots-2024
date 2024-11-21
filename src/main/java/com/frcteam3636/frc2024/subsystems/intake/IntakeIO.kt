package com.frcteam3636.frc2024.subsystems.intake
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.utils.math.TAU
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface IntakeIO {
    class Inputs : LoggableInputs {
        var rollerVelocity = RotationsPerSecond.zero()
        var current = Amps.zero()
        var position = Radians.zero()

        override fun toLog(table: LogTable) {
            table.put("Intake Velocity", rollerVelocity)
            table.put("Intake Current", current)
            table.put("Intake Roller Position", position)
        }

        override fun fromLog(table: LogTable) {
            rollerVelocity = table.get("Intake Velocity", rollerVelocity)
            current = table.get("Intake Current", current)
            position = table.get("Intake Roller Position", position)
        }

    }

    fun setSpeed(percent: Double)

    fun updateInputs(inputs: Inputs)

    class IntakeIOReal: IntakeIO {
        private var motor =
            CANSparkFlex(
                REVMotorControllerId.IntakeMotor,
                CANSparkLowLevel.MotorType.kBrushless
            )

        override fun setSpeed(percent: Double) {
            assert(percent in -1.0..1.0)
            motor.set(percent)
        }

        override fun updateInputs(inputs: Inputs) {
            inputs.rollerVelocity = RotationsPerSecond.of(motor.encoder.velocity)
            inputs.current = Amps.of(motor.outputCurrent)
            inputs.position = Rotations.of(motor.encoder.position.mod(1.0))
        }
    }

    class IntakeIOSim: IntakeIO {

        var motor = FlywheelSim(
            DCMotor.getNeoVortex(1),
            1.0,
            1.0
        )

        override fun updateInputs(inputs: Inputs) {
            motor.update(Robot.period)
            inputs.rollerVelocity = RadiansPerSecond.of(motor.angularVelocityRadPerSec)
            inputs.current = Amps.of(motor.currentDrawAmps)
            inputs.position += Radians.of(motor.angularVelocityRadPerSec * Robot.period)
            inputs.position = Radians.of(inputs.position.`in`(Radians).mod(TAU))
        }

        override fun setSpeed(percent: Double) {
            assert(percent in -1.0..1.0)
            motor.setInputVoltage(percent * 12)
        }
    }
}