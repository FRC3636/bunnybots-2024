package com.frcteam3636.frc2024.subsystems.arm

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.littletonrobotics.junction.Logger


object Arm : Subsystem {
    var io = ArmIOReal()

    var inputs = ArmIO.ArmInputs()

    var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Volts.of(4.0),
            null,
            { state -> SignalLogger.writeString("state", state.toString()) }
        ),
        Mechanism(this::voltageDrive, null, this)
    )

    private fun voltageDrive(measure: Measure<Voltage>) {
        io.setVoltage(measure)
    }

    override fun periodic() {
        io.updateInputs(inputs)

        Logger.processInputs("/Arm", inputs)
    }

    fun moveToPosition(position: Position) =
        startEnd({
            io.setPosition(position.angle)
        }, {
            io.setPosition(inputs.position)
        })

    fun sysIdQuasistatic(direction: Direction) =
        sysID.quasistatic(direction)

    fun sysIdDynamic(direction: Direction) =
        sysID.dynamic(direction)

    enum class Position(val angle: Measure<Angle>) {
        Stowed(Degrees.of(135.0)),
        PickUp(Degrees.of(30.0)),
        Lower(Degrees.of(-15.0))
    }
}