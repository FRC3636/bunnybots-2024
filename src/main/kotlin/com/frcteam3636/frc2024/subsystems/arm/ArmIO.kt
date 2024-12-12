package com.frcteam3636.frc2024.subsystems.arm
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2024.CTREDeviceId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.utils.math.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged
open class ArmInputs {
    var rightPosition = Radians.zero()!!
    var leftPosition = Radians.zero()!!
    var position = Radians.zero()!!

    var rightCurrent = Volts.zero()!!
    var leftCurrent = Volts.zero()!!

    var rightVelocity = RadiansPerSecond.zero()!!
    var leftVelocity = RadiansPerSecond.zero()!!

    var absoluteEncoderConnected = false
}

interface ArmIO {

        fun updateInputs(inputs: ArmInputs)

        fun pivotToPosition(position: Measure<Angle>)

        fun setVoltage(volts: Measure<Voltage>)

        fun updatePosition(position: Measure<Angle>)
}


class ArmIOReal: ArmIO {

    private val leftMotor = TalonFX(CTREDeviceId.LeftArmMotor)

    private val rightMotor = TalonFX(CTREDeviceId.RightArmMotor)

    private val absoluteEncoder = DutyCycleEncoder(DigitalInput(0))

    override fun updateInputs(inputs: ArmInputs) {
        val unoffsetPosition = Rotations.of(-absoluteEncoder.get() * CHAIN_GEAR_RATIO)
        inputs.position = unoffsetPosition + ZERO_OFFSET
        Logger.recordOutput("/Arm/Required Offset", unoffsetPosition.negate())
//        inputs.position = Rotations.of(-absoluteEncoder.absolutePosition) + ZERO_OFFSET
        inputs.absoluteEncoderConnected = absoluteEncoder.isConnected

        inputs.leftPosition = Rotations.of(leftMotor.position.value)
        inputs.leftVelocity = RotationsPerSecond.of(leftMotor.velocity.value)


        inputs.rightPosition = Rotations.of(rightMotor.position.value)
        inputs.rightVelocity = RotationsPerSecond.of(rightMotor.position.value)

        inputs.rightCurrent = Volts.of(leftMotor.motorVoltage.value)
    }

    override fun updatePosition(position: Measure<Angle>) {
        leftMotor.setPosition(position.`in`(Rotations))
        rightMotor.setPosition(position.`in`(Rotations))
    }

    override fun pivotToPosition(position: Measure<Angle>) {
        val resolvedPosition = position
        Logger.recordOutput("Shooter/Pivot/Position Setpoint", resolvedPosition)

        val control = MotionMagicTorqueCurrentFOC(0.0).apply {
            Slot = 0
            Position = resolvedPosition.`in`(Rotations)
        }
        leftMotor.setControl(control)
//        rightMotor.setControl(control)

    }

    override fun setVoltage(volts: Measure<Voltage>) {
        assert(volts in Volts.of(-12.0)..Volts.of(12.0))
        Logger.recordOutput("/Arm/Voltage", volts)
        val control = VoltageOut(volts.`in`(Volts))
        leftMotor.setControl(control) // TODO: fix gearbox
//        rightMotor.setControl(control)
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

            ClosedLoopGeneral.apply {
                ContinuousWrap = true
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
//        private const val GEAR_RATIO = 125.0
//        val PID_GAINS = PIDGains(60.88, 0.0, 0.40035) //placeholders
//        val FF_GAINS = MotorFFGains(0.1448, 0.11929, 0.001579) //placeholders
//        private const val GRAVITY_GAIN = 0.0095692
//        private const val PROFILE_ACCELERATION = 1.0
//        private const val PROFILE_JERK = 1.0
//        private const val PROFILE_VELOCITY = 1.0

        private const val CHAIN_GEAR_RATIO = 1.0 / 3.0
        private const val GEAR_RATIO = 125.0 / CHAIN_GEAR_RATIO
        val PID_GAINS = PIDGains(64.857, 0.0, 20.319) //placeholders
        val FF_GAINS = MotorFFGains(0.33157, 14.214, 0.15033) //placeholders
        private const val GRAVITY_GAIN = 0.28233
        private const val PROFILE_ACCELERATION = 1.0
        private const val PROFILE_JERK = 1.0
        private const val PROFILE_VELOCITY = 1.0

        val ZERO_OFFSET = Radians.of(1.01)
    }

}

class ArmIOSim : ArmIO {
    val armSim = SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        3.0,
        0.244,
        0.331414,
        -0.142921,
        2.32814365359,
        true,
        -0.142921
    )

    // TODO: Once we know the PID gains, this should be added back.
//        val pidController = PIDController(ArmIOReal.PID_GAINS)
    var setPoint = 0.0

    override fun updateInputs(inputs: ArmInputs) {
        armSim.update(Robot.period)
        inputs.rightVelocity = RadiansPerSecond.of(armSim.velocityRadPerSec)
        inputs.leftVelocity = RadiansPerSecond.of(armSim.velocityRadPerSec)
        inputs.position = Radians.of(armSim.angleRads)
//            val pidVoltage = pidController.calculate(inputs.position.`in`(Radians),setPoint)
//            armSim.setInputVoltage(pidVoltage)
    }

    override fun pivotToPosition(position: Measure<Angle>) {
        setPoint = position.`in`(Radians)
    }

    override fun setVoltage(volts: Measure<Voltage>) {
        armSim.setInputVoltage(volts.`in`(Volts))
        Logger.recordOutput("/Arm/OutVolt", volts)
    }

    override fun updatePosition(position: Measure<Angle>) {
        // no drifting in sim so no need to update
    }
}

class ArmIOPrototype: ArmIO {
    //placeholder
    override fun updateInputs(inputs: ArmInputs) {
    }

    override fun pivotToPosition(position: Measure<Angle>) {
    }

    override fun setVoltage(volts: Measure<Voltage>) {
    }

    override fun updatePosition(position: Measure<Angle>) {
    }
}
