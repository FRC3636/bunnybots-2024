package com.frcteam3636.frc2024.subsystems.wrist

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
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
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team9432.annotation.Logged
import kotlin.concurrent.thread

interface WristIO {
    fun updateInputs(inputs: WristInputs)
    fun pivotToAndStop(position: Measure<Angle>)
}

@Logged
open class WristInputs {
    var position = Radians.zero()!!
    var velocity = RadiansPerSecond.zero()!!
    var voltage = Volts.zero()!!
}

class WristIOKraken: WristIO {
    private val wristMotor = TalonFX(CTREMotorControllerId.WristMotor)
    private val absoluteEncoder = DutyCycleEncoder(DigitalInput(7))

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
    }

    override fun updateInputs(inputs: WristInputs) {
//            wristMotor.setPosition(absoluteEncoder.absolutePosition)

        inputs.position = Rotations.of(wristMotor.position.value)
        inputs.velocity = RotationsPerSecond.of(wristMotor.velocity.value)
        inputs.voltage = Volts.of(wristMotor.motorVoltage.value)
    }

    override fun pivotToAndStop(position: Measure<Angle>) {
        Logger.recordOutput("Wrist/Position Setpoint", position)

        val wristControl = MotionMagicTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = position.`in`(Rotations)
        }
        wristMotor.setControl(wristControl)
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