package com.frcteam3636.frc2024.subsystems.intake
import com.frcteam3636.frc2024.CANSparkFlex
import com.frcteam3636.frc2024.REVMotorControllerId
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.ColorMatch
import com.revrobotics.ColorSensorV3
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.util.Color
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface IntakeIO {
    class IntakeInputs : LoggableInputs{
        var rollerVelocity = Rotation2d()
        var current: Double = 0.0
        var isIntaking: Boolean = false
        var hasBalloon: Boolean = false
        var balloonIsBlue: Boolean = false

        override fun toLog(table: LogTable) {
            table.put("UTB Roller Velocity", rollerVelocity)
            table.put("UTB Current", current)
            table.put("Is Intaking", isIntaking)
            table.put("Has balloon", hasBalloon)
            table.put("Balloon color", balloonIsBlue)
        }

        override fun fromLog(table: LogTable) {
            rollerVelocity = table.get("UTB Roller Velocity", rollerVelocity)!![0]
            current = table.get("UTB Current", current)
            isIntaking = table.get("Is Intaking", isIntaking)
            hasBalloon = table.get("Has balloon", hasBalloon)
            balloonIsBlue = table.get("Balloon color", balloonIsBlue)
        }


    }

    fun updateInputs(inputs: IntakeInputs)

    fun setUnderBumperRoller(speed: Double)

    class IntakeIOReal: IntakeIO {
        private var rollerVelocity =
            CANSparkFlex(
                REVMotorControllerId.UnderTheBumperIntakeRoller,
                CANSparkLowLevel.MotorType.kBrushless
            )

        private var colorSensorV3 = ColorSensorV3(I2C.Port.kOnboard)

        private val m_colorMatcher = ColorMatch()


        private val kBlueTarget: Color = Color(0.143, 0.427, 0.429)
        private val kRedTarget: Color = Color(0.561, 0.232, 0.114)

        init {
            m_colorMatcher.addColorMatch(kBlueTarget)
            m_colorMatcher.addColorMatch(kRedTarget)
        }

        override fun updateInputs(inputs: IntakeInputs) {
            val detectedColor: Color = colorSensorV3.getColor()

            var colorString: String
            val match = m_colorMatcher.matchClosestColor(detectedColor)

            if (match.color == kRedTarget) {
                inputs.balloonIsBlue = false
            }
            else if (match.color == kBlueTarget) {
                inputs.balloonIsBlue = true
            }
        }

        override fun setUnderBumperRoller(speed: Double) {
            TODO("Not yet implemented")
        }
    }
}