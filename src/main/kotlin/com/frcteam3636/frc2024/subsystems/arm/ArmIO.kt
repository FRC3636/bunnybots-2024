package com.frcteam3636.frc2024.subsystems.arm
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2024.CTREDeviceId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.TalonFX
import com.frcteam3636.frc2024.utils.Elastic
import com.frcteam3636.frc2024.utils.ElasticNotification
import com.frcteam3636.frc2024.utils.math.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged
open class ArmInputs {
    var reference = Radians.zero()!!

    var rightRelativePosition = Radians.zero()!!
    var leftRelativePosition = Radians.zero()!!
    var leftPosition = Radians.zero()!!
    var rightPosition = Radians.zero()!!

    var rightCurrent = Volts.zero()!!
    var leftCurrent = Volts.zero()!!

    var rightVelocity = RadiansPerSecond.zero()!!
    var leftVelocity = RadiansPerSecond.zero()!!

    var leftAbsoluteEncoderConnected = false
    var rightAbsoluteEncoderConnected = false
}

interface ArmIO {

        fun updateInputs(inputs: ArmInputs)

        fun pivotToPosition(position: Measure<Angle>)

        fun setVoltage(volts: Measure<Voltage>)

        fun updatePosition(left: Measure<Angle>, right: Measure<Angle>)

        fun setCoastMode(enabled: Boolean)

        fun zeroHere(inputs: ArmInputs)

        val armZeroed: Boolean
            get() = true
}


class ArmIOReal: ArmIO {
    private val leftMotor = TalonFX(CTREDeviceId.LeftArmMotor)
    private val rightMotor = TalonFX(CTREDeviceId.RightArmMotor)

    private val leftAbsoluteEncoder = DutyCycleEncoder(DigitalInput(0))
    private val rightAbsoluteEncoder = DutyCycleEncoder(DigitalInput(1))

    private var leftZero: Measure<Angle>
    private var rightZero: Measure<Angle>

    override var armZeroed = true
        private set

    private val leftConfig = TalonFXConfiguration().apply {
        Slot0.apply {
            pidGains = LEFT_PID_GAINS
            motorFFGains = LEFT_FF_GAINS
            GravityType = GravityTypeValue.Arm_Cosine
            kG = LEFT_GRAVITY_GAIN
        }

        MotorOutput.apply {
            NeutralMode = NeutralModeValue.Brake
            Inverted = InvertedValue.CounterClockwise_Positive
        }

        Feedback.apply {
            SensorToMechanismRatio = GEAR_RATIO
            FeedbackRotorOffset = 0.0
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

    private val rightConfig = TalonFXConfiguration().apply {
        deserialize(leftConfig.serialize()) // TalonFXConfiguration doesn't implement Cloneable, so we do this instead

        MotorOutput.apply {
            Inverted = InvertedValue.Clockwise_Positive
        }

        Slot0.apply {
            pidGains = RIGHT_PID_GAINS
            motorFFGains = RIGHT_FF_GAINS
            GravityType = GravityTypeValue.Arm_Cosine
            kG = RIGHT_GRAVITY_GAIN
        }
    }

    private fun warnArmNotZeroed() {
        Elastic.sendAlert(ElasticNotification("ARM MUST BE ZEROED!!!",
            "The arm will not work unless it is zeroed before the current match."))
    }

    init {
        Logger.recordOutput("/Arm/CoastMode", false)

        leftMotor.configurator.apply(leftConfig)
        rightMotor.configurator.apply(rightConfig)

        if (!Preferences.containsKey("LeftArmZero") || !Preferences.containsKey("RightArmZero")) {
            warnArmNotZeroed()
            armZeroed = false
        }
        leftZero = Radians.of(Preferences.getDouble("LeftArmZero", 0.0))
        rightZero = Radians.of(Preferences.getDouble("RightArmZero", 0.0))
    }

    override fun updateInputs(inputs: ArmInputs) {
        val offsetlessLeftPosition = Rotations.of(-leftAbsoluteEncoder.get() * CHAIN_GEAR_RATIO)
        inputs.leftPosition = offsetlessLeftPosition + leftZero
        Logger.recordOutput("/Arm/Required Left Offset", offsetlessLeftPosition.negate())

        val offsetlessRightPosition = Rotations.of(rightAbsoluteEncoder.get() * CHAIN_GEAR_RATIO)
        inputs.rightPosition = offsetlessRightPosition + rightZero
        Logger.recordOutput("/Arm/Required Right Offset", offsetlessRightPosition.negate())

        inputs.leftRelativePosition = Rotations.of(leftMotor.position.value)
        inputs.rightRelativePosition = Rotations.of(rightMotor.position.value)

        inputs.leftAbsoluteEncoderConnected = leftAbsoluteEncoder.isConnected
        inputs.rightAbsoluteEncoderConnected = rightAbsoluteEncoder.isConnected

        inputs.leftVelocity = RotationsPerSecond.of(leftMotor.velocity.value)
        inputs.rightVelocity = RotationsPerSecond.of(rightMotor.velocity.value)

        inputs.leftCurrent = Volts.of(leftMotor.motorVoltage.value)
        inputs.rightCurrent = Volts.of(rightMotor.motorVoltage.value)

        inputs.reference = Rotations.of(leftMotor.closedLoopReference.value)
    }

    override fun updatePosition(left: Measure<Angle>, right: Measure<Angle>) {
        leftMotor.setPosition(left.`in`(Rotations))
        rightMotor.setPosition(right.`in`(Rotations))
    }

    private val pivotControl = MotionMagicTorqueCurrentFOC(0.0)
    override fun pivotToPosition(position: Measure<Angle>) {
        if (!armZeroed) return warnArmNotZeroed()
        Logger.recordOutput("Arm/Position Setpoint", position)

        val control = pivotControl.apply {
            Slot = 0
            Position = position.`in`(Rotations)
        }
        leftMotor.setControl(control)
        rightMotor.setControl(control)
    }

    override fun setCoastMode(enabled: Boolean) {
        Logger.recordOutput("/Arm/CoastMode", enabled)
        val neutralModeValue = if (enabled) {
            NeutralModeValue.Coast
        } else {
            NeutralModeValue.Brake
        }

        leftMotor.configurator.apply(leftConfig.apply {
            MotorOutput.apply {
                NeutralMode = neutralModeValue
            }
        })
        rightMotor.configurator.apply(rightConfig.apply {
            MotorOutput.apply {
                NeutralMode = neutralModeValue
            }
        })
    }

    override fun zeroHere(inputs: ArmInputs) {
        armZeroed = true
        leftZero = (inputs.leftPosition - leftZero).negate() + Degrees.of(90.0)
        rightZero = (inputs.rightPosition - rightZero).negate() + Degrees.of(90.0)
        Preferences.setDouble("LeftArmZero", leftZero.`in`(Radians))
        Preferences.setDouble("RightArmZero", rightZero.`in`(Radians))
    }

    override fun setVoltage(volts: Measure<Voltage>) {
        assert(volts in Volts.of(-12.0)..Volts.of(12.0))
        if (!armZeroed) return warnArmNotZeroed()
        Logger.recordOutput("/Arm/Voltage", volts)
        val control = VoltageOut(volts.`in`(Volts))
//        leftMotor.setControl(control)
//        rightMotor.setControl(control)
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
        val LEFT_PID_GAINS = PIDGains(54.138, 0.0, 169.411)
        val LEFT_FF_GAINS = MotorFFGains(0.3289, 42.817, 0.39107)
        private const val LEFT_GRAVITY_GAIN = 0.17119
        val RIGHT_PID_GAINS = PIDGains(53.339, 0.0, 169.134)
        val RIGHT_FF_GAINS = MotorFFGains(0.29364, 43.77, 0.34751)
        private const val RIGHT_GRAVITY_GAIN = 0.12181
        private const val PROFILE_ACCELERATION = 1.0
        private const val PROFILE_JERK = 1.0
        private const val PROFILE_VELOCITY = 1.0
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
        inputs.leftPosition = Radians.of(armSim.angleRads)
        inputs.rightPosition = Radians.of(armSim.angleRads)
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

    override fun updatePosition(left: Measure<Angle>, right: Measure<Angle>) {
        // no drifting in sim so no need to update
    }

    override fun setCoastMode(enabled: Boolean) {

    }

    override fun zeroHere(inputs: ArmInputs) {
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

    override fun updatePosition(left: Measure<Angle>, right: Measure<Angle>) {
    }

    override fun setCoastMode(enabled: Boolean) {

    }

    override fun zeroHere(inputs: ArmInputs) {
    }
}
