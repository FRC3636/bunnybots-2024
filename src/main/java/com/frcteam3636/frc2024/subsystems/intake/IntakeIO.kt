package com.frcteam3636.frc2024.subsystems.intake
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface IntakeIO {
    class IntakeInputs : LoggableInputs{
        private var rollerVelocity = Rotation2d()
        private var current: Double = 0.0
        private var hasBalloon: Boolean = false
        private var balloonIsBlue: Boolean = false

        override fun toLog(table: LogTable) {
            table.put("UTB Roller Velocity", rollerVelocity)
            table.put("UTB Current", current)
            table.put("Has balloon", hasBalloon)
            table.put("Balloon color", balloonIsBlue)
        }

        override fun fromLog(table: LogTable) {
            rollerVelocity = table.get("UTB Roller Velocity", rollerVelocity)!![0]
            current = table.get("UTB Current", current)
            hasBalloon = table.get("Has balloon", hasBalloon)
            balloonIsBlue = table.get("Balloon color", balloonIsBlue)
        }


    }

    fun setSpeed(percent: Double)

    class IntakeIOReal: IntakeIO {
        private var motor =
            CANSparkFlex(
                REVMotorControllerId.UnderTheBumperIntakeRoller,
                CANSparkLowLevel.MotorType.kBrushless
            )

        override fun setSpeed(percent: Double) {
            motor.set(percent)
        }
    }

    class IntakeIOSim: IntakeIO {
        override fun setSpeed(percent: Double) {
            TODO("Not yet implemented")
        }
    }
}