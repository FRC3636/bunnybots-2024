package com.frcteam3636.frc2024.subsystems.intake
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface IntakeIO {
    class Inputs : LoggableInputs {
        var rollerVelocity = RotationsPerSecond.zero()
        var current = Amps.zero()

        override fun toLog(table: LogTable) {
            table.put("Intake Velocity", rollerVelocity)
            table.put("Intake Current", current)
        }

        override fun fromLog(table: LogTable) {
            rollerVelocity = table.get("Intake Velocity", rollerVelocity)
            current = table.get("Intake Current", current)
        }

    }

    fun setSpeed(percent: Double)

    fun updateInputs(inputs: IntakeIO.Inputs)

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

        }

        override fun setSpeed(percent: Double) {
            assert(percent in -1.0..1.0)
            motor.setInputVoltage(percent * 12)
        }
    }
}