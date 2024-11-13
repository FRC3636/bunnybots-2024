package com.frcteam3636.frc2024.subsystems.arm
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder

interface ArmIO{
        class ArmInputs : LoggableInputs {
            var rightPosition = Radians.zero()
            var leftPosition = Radians.zero()

            var rightCurrent: Double = 0.0
            var leftCurrent: Double = 0.0

            var rightVelocity: Double = 0.0
            var leftVelocity: Double = 0.0

            override fun toLog(table: LogTable) {
                table.put("Right Arm Motor Position", rightPosition)
                table.put("Left Arm Motor Position", leftPosition)

                table.put("Right Arm Motor Current", rightCurrent)
                table.put("Left Arm Motor Current", leftCurrent)

                table.put("Right Arm Motor Velocity", rightVelocity)
                table.put("Left Arm Motor Velocity", leftVelocity)
            }

            override fun fromLog(table: LogTable) {
                rightPosition = table.get("Right Arm Motor Position", rightPosition)
                leftPosition = table.get("Left Arm Motor Position", leftPosition)

                rightCurrent = table.get("Right Arm Motor Current", rightCurrent)
                leftCurrent = table.get("Left Arm Motor Current", leftCurrent)

                rightVelocity = table.get("Right Arm Motor Velocity", rightVelocity)
                leftVelocity = table.get("Left Arm Motor Velocity", leftVelocity)
            }
        }

        fun updateInputs(inputs: ArmInputs)

        fun setPosition(position: Measure<Angle>)
    }

    class ArmIOReal: ArmIO{

        private val leftMotor = TalonFX(CTREMotorControllerId.LeftArmMotor)

        private val rightMotor = TalonFX(CTREMotorControllerId.RightArmMotor)

        private val absoluteEncoder = DutyCycleEncoder(DigitalInput(7))

        override fun updateInputs(inputs: ArmIO.ArmInputs) {
            TODO("Not yet implemented")
        }

        override fun setPosition(position: Measure<Angle>) {
            TODO("Not yet implemented")
        }

    }