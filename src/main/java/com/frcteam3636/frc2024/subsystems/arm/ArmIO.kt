package com.frcteam3636.frc2024.subsystems.arm
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.utils.math.MotorFFGains
import com.frcteam3636.frc2024.utils.math.PIDGains
import com.frcteam3636.frc2024.utils.math.motorFFGains
import com.frcteam3636.frc2024.utils.math.pidGains
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs


interface ArmIO{
        class ArmInputs : LoggableInputs {

            var rightPosition = Radians.zero()!!
            var leftPosition = Radians.zero()!!
            var position = Radians.zero()!!

            var absoluteEncoderPosition = Radians.zero()!!

            var rightCurrent = Volts.zero()!!
            private var leftCurrent = Volts.zero()!!

            var rightVelocity = RadiansPerSecond.zero()!!
            var leftVelocity = RadiansPerSecond.zero()!!

            var absoluteEncoderConnected = false


            override fun toLog(table: LogTable) {
                table.put("Right Arm Motor Position", rightPosition)
                table.put("Left Arm Motor Position", leftPosition)

                table.put("Right Arm Motor Current", rightCurrent)
                table.put("Left Arm Motor Current", leftCurrent)

                table.put("Right Arm Motor Velocity", rightVelocity)
                table.put("Left Arm Motor Velocity", leftVelocity)

                table.put("Absolute Encoder Connected",absoluteEncoderConnected )
            }

            override fun fromLog(table: LogTable) {
                rightPosition = table.get("Right Arm Motor Position", rightPosition)
                leftPosition = table.get("Left Arm Motor Position", leftPosition)

                rightCurrent = table.get("Right Arm Motor Current", rightCurrent)
                leftCurrent = table.get("Left Arm Motor Current", leftCurrent)

                rightVelocity = table.get("Right Arm Motor Velocity", rightVelocity)
                leftVelocity = table.get("Left Arm Motor Velocity", leftVelocity)

                absoluteEncoderConnected = table.get("Absolute Encoder Connected", absoluteEncoderConnected)
            }
        }

        fun updateInputs(inputs: ArmInputs)

        fun setPosition(position: Measure<Angle>)

    fun setVoltage(volts: Measure<Voltage>)
    }

    class ArmIOReal: ArmIO{

        private val leftMotor = TalonFX(CTREMotorControllerId.LeftArmMotor)

        private val rightMotor = TalonFX(CTREMotorControllerId.RightArmMotor)

        private val absoluteEncoder = DutyCycleEncoder(DigitalInput(7))

        override fun updateInputs(inputs: ArmIO.ArmInputs) {
            inputs.position = Rotations.of(absoluteEncoder.absolutePosition)
            inputs.absoluteEncoderConnected = absoluteEncoder.isConnected

            inputs.absoluteEncoderPosition = Rotations.of(absoluteEncoder.absolutePosition)

            inputs.leftPosition = Rotations.of(leftMotor.position.value)
            inputs.leftVelocity = RotationsPerSecond.of(leftMotor.velocity.value)


            inputs.rightPosition = Rotations.of(rightMotor.position.value)
            inputs.rightVelocity = RotationsPerSecond.of(rightMotor.position.value)

            inputs.rightCurrent = Volts.of(leftMotor.motorVoltage.value)

            leftMotor.setPosition(inputs.absoluteEncoderPosition.`in`(Rotations))
            rightMotor.setPosition(inputs.absoluteEncoderPosition.`in`(Rotations))
        }

        override fun setPosition(position: Measure<Angle>) {
            Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)

            val leftControl = MotionMagicTorqueCurrentFOC(0.0).apply {
                Slot = 0
                Position = position.`in`(Rotations)
            }
            leftMotor.setControl(leftControl)

        }

        override fun setVoltage(volts: Measure<Voltage>) {
            val control = VoltageOut(volts.`in`(Volts))
            leftMotor.setControl(control)
            rightMotor.setControl(control)
        }

        init {
            val config = TalonFXConfiguration().apply {
                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                }

                Feedback.apply {
                    SensorToMechanismRatio = GEAR_RATIO
                    FeedbackRotorOffset = 0.0
                }

                Slot0.apply {
                    pidGains = PID_GAINS
                    motorFFGains = FF_GAINS
                    GravityType = GravityTypeValue.Arm_Cosine
                    kG = GRAVITY_GAIN
                }

                MotionMagic.apply {
                    MotionMagicCruiseVelocity = PROFILE_VELOCITY
                    MotionMagicAcceleration = PROFILE_ACCELERATION
                    MotionMagicJerk = PROFILE_JERK
                }
            }

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
            leftMotor.configurator.apply(
                config
            )

            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
            rightMotor.configurator.apply(config)
        }


        internal companion object Constants {
            private const val GEAR_RATIO = 0.0
            val PID_GAINS = PIDGains(120.0, 0.0, 100.0) //placeholders
            val FF_GAINS = MotorFFGains(7.8, 0.0, 0.0) //placeholders
            private const val GRAVITY_GAIN = 0.0
            private const val PROFILE_ACCELERATION = 0.0
            private const val PROFILE_JERK = 0.0
            private const val PROFILE_VELOCITY = 0.0
        }

    }