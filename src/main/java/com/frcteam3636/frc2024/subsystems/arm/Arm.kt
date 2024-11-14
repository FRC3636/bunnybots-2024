package com.frcteam3636.frc2024.subsystems.arm

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Arm : Subsystem {
    var io = ArmIOReal()

    var inputs = ArmIO.ArmInputs()

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


    enum class Position(val angle: Measure<Angle>) {
        Stowed(Degrees.of(135.0)),
        PickUp(Degrees.of(30.0)),
        Lower(Degrees.of(-15.0))
    }
}