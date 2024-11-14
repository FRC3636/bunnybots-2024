package com.frcteam3636.frc2024.subsystems.intake

import com.frcteam3636.frc2024.Robot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Intake: Subsystem {
    private var io: IntakeIO = when (Robot.model) {
        Robot.Model.SIMULATION -> IntakeIO.IntakeIOSim()
        Robot.Model.COMPETITION -> IntakeIO.IntakeIOReal()
        Robot.Model.PROTOTYPE -> IntakeIO.IntakeIOSim()
    }

    var inputs = IntakeIO.IntakeInputs()

    override fun periodic() {
        Logger.processInputs("Intake", inputs)
    }

    fun outtake(): Command =
        startEnd(
            {
                io.setSpeed(-0.5)
            },
            {
                io.setSpeed(0.0)
            }
        )

    fun intake(): Command =
        startEnd(
            {
                io.setSpeed(0.7)
            },
            {
                io.setSpeed(0.0)
            }
        )

}