package com.frcteam3636.frc2024.subsystems.arm

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.utils.math.TAU
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Time
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.littletonrobotics.junction.Logger
import kotlin.math.sin
import kotlin.time.Duration


private const val SECONDS_BETWEEN_ARM_UPDATES = 1.0

object Arm : Subsystem {
    private var io: ArmIO = when (Robot.model) {
        Robot.Model.SIMULATION -> ArmIOSim()
        Robot.Model.COMPETITION -> ArmIOReal()
        Robot.Model.PROTOTYPE -> ArmIOPrototype()
    }

    var inputs = LoggedArmInputs()
    var mechanism = Mechanism2d(200.0, 200.0)
    var armAngleLigament = MechanismLigament2d("Arm Ligament", 50.0, 90.0, 5.0, Color8Bit(Color.kGreen))
    var armWristAngleLigament = MechanismLigament2d("Arm Wrist Ligament", 20.0, 90.0, 5.0, Color8Bit(Color.kGreen))

    init {
        mechanism.getRoot("Arm", 100.0, 100.0).apply{
            append(armAngleLigament)
        }
        armAngleLigament.append(armWristAngleLigament)
    }

    private var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Volts.of(4.0),
            null
        ) { state -> SignalLogger.writeString("state", state.toString()) },
        Mechanism(io::setVoltage, null, this)
    )

//    var inSysIdUpperRange = Trigger {
//        val lowerLimit = Radians.of(3.167)
//        inputs.position < lowerLimit
//                && inputs.leftPosition < lowerLimit
////                && inputs.rightPosition < lowerLimit
//    }

    private var timer = Timer().apply {
        start()
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("/Arm", inputs)
        armAngleLigament.angle = inputs.leftPosition.`in`(Degrees)
        armWristAngleLigament.angle = 90.0 - inputs.leftPosition.`in`(Degrees)

        if (timer.advanceIfElapsed(SECONDS_BETWEEN_ARM_UPDATES)
            && inputs.leftAbsoluteEncoderConnected
            && inputs.rightAbsoluteEncoderConnected) {
            io.updatePosition(left = inputs.leftPosition, right = inputs.rightPosition)
        }

        Logger.recordOutput("/Arm/Mechanism", mechanism)

    }

    fun moveToPosition(position: Position) =
        startEnd({
            io.pivotToPosition(position.angle)
        }, {
            io.pivotToPosition(inputs.leftPosition)
        })!!

    fun coastMode() =
        startEnd({
            io.setCoastMode(true)
        }, {
            io.setCoastMode(false)
        })!!

    /**
     * Follow a sine wave to test the arm's motion control.
     *
     * The wave begins as soon as the command is instantiated, not when the command is scheduled.
     *
     * @param center the angle which the wave oscillates around
     * @param magnitude the size of the wave
     * @param period the time before the wave returns to its original position
     */
    fun followWave(center: Measure<Angle>, magnitude: Measure<Angle>, period: Measure<Time> = Seconds.of(5.0)): Command {
        val timer = Timer().apply {
            start()
        }
        return runEnd({
            // Use the formula for a sine wave with a magnitude, period, and center point.
            val setpoint = magnitude * sin(timer.get() * TAU / period.`in`(Seconds)) + center
            io.pivotToPosition(setpoint)
        }, {
            io.pivotToPosition(inputs.leftPosition)
        })!!
    }

    fun sysIdQuasistatic(direction: Direction) =
        sysID.quasistatic(direction)!!

    fun sysIdDynamic(direction: Direction) =
        sysID.dynamic(direction)!!

    enum class Position(val angle: Measure<Angle>) {
        /** All the way up */
        Stowed(Degrees.of(-127.0)),
//        Stowed(Degrees.of(-80.0)),
        /** Slightly picked up */
        PickUp(Degrees.of(0.0)),
        /** Ready to pick up */
        Lower(Degrees.of(15.0))
    }
}