package com.frcteam3636.frc2024.subsystems.wrist

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
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
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

interface WristIO {
    class Inputs : LoggableInputs {
        var position = Radians.zero()
        var velocity = RadiansPerSecond.zero()
        var voltage = Volts.zero()

        override fun toLog(table: LogTable) {
            table.put("Position", position)
            table.put("Velocity", velocity)
            table.put("Voltage", voltage)
        }

        override fun fromLog(table: LogTable) {
            position = table.get("Position", position)
            velocity = table.get("Velocity", velocity)
            voltage = table.get("Voltage", voltage)
        }
    }
    fun updateInputs(inputs: Inputs)

    fun pivotToAndStop(position: Rotation2d)

    class WristIOKraken: WristIO {
        private val wristMotor = TalonFX(CTREMotorControllerId.WristMotor)

        private val absoluteEncoder = DutyCycleEncoder(DigitalInput(7)).apply {
            distancePerRotation = SENSOR_TO_PIVOT_RATIO
        }

        private val rawAbsoluteEncoderPosition
            get() = Rotation2d.fromRotations(-absoluteEncoder.absolutePosition)

        private var zeroOffset: Double = ABSOLUTE_ENCODER_OFFSET

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
            wristMotor.configurator.apply(
                config
            )

            wristMotor.setPosition(HARDSTOP_OFFSET.`in`(Rotations))
        }

        override fun updateInputs(inputs: Inputs) {
            inputs.position = Radians.zero()
            inputs.velocity = RadiansPerSecond.zero()
            inputs.voltage = Volts.zero()
        }

        override fun pivotToAndStop(position: Rotation2d) {
            Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)

            val wristControl = MotionMagicTorqueCurrentFOC(0.0).apply {
                Slot = 0
                Position = position.rotations
            }
            wristMotor.setControl(wristControl)
        }

        internal companion object Constants {
            private const val SENSOR_TO_PIVOT_RATIO = 0.0
            private const val ABSOLUTE_ENCODER_OFFSET = 0.0
            private const val GEAR_RATIO = 0.0
            val PID_GAINS = PIDGains(120.0, 0.0, 100.0) //placeholders
            val FF_GAINS = MotorFFGains(7.8, 0.0, 0.0) //placeholders
            private const val GRAVITY_GAIN = 0.0
            private const val PROFILE_ACCELERATION = 0.0
            private const val PROFILE_JERK = 0.0
            private const val PROFILE_VELOCITY = 0.0
            private val HARDSTOP_OFFSET = Radians.zero()
        }
    }
}