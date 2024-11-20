package com.frcteam3636.frc2024.subsystems.intake
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface IntakeIO {
    class Inputs : LoggableInputs {
        var rollerVelocity = Rotation2d()
        var current: Double = 0.0

        override fun toLog(table: LogTable) {
            table.put("Intake Velocity", rollerVelocity)
            table.put("Intake Current", current)
        }

        override fun fromLog(table: LogTable) {
            rollerVelocity = table.get("Intake Velocity", rollerVelocity)!![0]
            current = table.get("Intake Current", current)
        }

    }

    fun setSpeed(percent: Double)

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
    }

    class IntakeIOSim: IntakeIO {
        override fun setSpeed(percent: Double) {
            TODO("Not yet implemented")
        }
    }
}