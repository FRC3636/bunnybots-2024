package com.frcteam3636.frc2024.subsystems.arm

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.frc2024.Robot
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.littletonrobotics.junction.Logger


private const val SECONDS_BETWEEN_ARM_UPDATES = 0.5

object Arm : Subsystem {
    private var io: ArmIO = when (Robot.model) {
        Robot.Model.SIMULATION -> ArmIOSim()
        Robot.Model.COMPETITION -> ArmIOReal()
        Robot.Model.PROTOTYPE -> ArmIOPrototype()
    }

    var inputs = ArmIO.ArmInputs()

    private var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Volts.of(4.0),
            null
        ) { state -> SignalLogger.writeString("state", state.toString()) },
        Mechanism(io::setVoltage, null, this)
    )

    private var timer = Timer().apply {
        start()
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("/Arm", inputs)
        if (timer.advanceIfElapsed(SECONDS_BETWEEN_ARM_UPDATES) && inputs.absoluteEncoderConnected){
                io.updatePosition(inputs.absoluteEncoderPosition)
        }

    }

    fun moveToPosition(position: Position) =
        startEnd({
            io.pivotToPosition(position.angle)
        }, {
            io.pivotToPosition(inputs.position)
        })!!

    fun sysIdQuasistatic(direction: Direction) =
        sysID.quasistatic(direction)!!

    fun sysIdDynamic(direction: Direction) =
        sysID.dynamic(direction)!!

    enum class Position(val angle: Measure<Angle>) {
        Stowed(Degrees.of(135.0)),
        PickUp(Degrees.of(30.0)),
        Lower(Degrees.of(-15.0))
    }
}