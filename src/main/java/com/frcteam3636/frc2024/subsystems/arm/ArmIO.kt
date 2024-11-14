package com.frcteam3636.frc2024.subsystems.arm
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TalonFXConfigurator
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.TalonFX
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.littletonrobotics.junction.Logger



interface ArmIO{
        class ArmInputs : LoggableInputs {

            var rightPosition = Radians.zero()
            var leftPosition = Radians.zero()

            var absoluteEncoderPostion = Radians.zero()

            var rightCurrent = Volts.zero()
            var leftCurrent = Volts.zero()

            var rightVelocity = RadiansPerSecond.zero()
            var leftVelocity = RadiansPerSecond.zero()

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
    }

    class ArmIOReal: ArmIO{

        private val leftMotor = TalonFX(CTREMotorControllerId.LeftArmMotor)

        private val rightMotor = TalonFX(CTREMotorControllerId.RightArmMotor)

        private val absoluteEncoder = DutyCycleEncoder(DigitalInput(7))

        override fun updateInputs(inputs: ArmIO.ArmInputs) {
            inputs.absoluteEncoderConnected = absoluteEncoder.isConnected

            inputs.absoluteEncoderPostion = Rotations.of(absoluteEncoder.absolutePosition)

            inputs.leftPosition = Rotations.of(leftMotor.position.value)
            inputs.leftVelocity = RotationsPerSecond.of(leftMotor.velocity.value)


            inputs.rightPosition = Rotations.of(rightMotor.position.value)
            inputs.rightVelocity = RotationsPerSecond.of(rightMotor.position.value)

            inputs.rightCurrent = Volts.of(leftMotor.motorVoltage.value)

        }

        override fun setPosition(position: Measure<Angle>) {
            Logger.recordOutput("Shooter/Pivot/Position Setpoint", position)

            val leftControl = 

            TODO("Not yet implemented")
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

            leftMotor.setPosition(HARDSTOP_OFFSET.rotations)
            rightMotor.setPosition(HARDSTOP_OFFSET.rotations)
        }


        }

    }