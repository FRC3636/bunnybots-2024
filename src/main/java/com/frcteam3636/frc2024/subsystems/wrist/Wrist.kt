package com.frcteam3636.frc2024.subsystems.wrist

import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.subsystems.intake.IntakeIO
import com.frcteam3636.frc2024.subsystems.wrist.WristIO
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Wrist: Subsystem {
    private var io: WristIO = WristIO.WristIOKraken()

    var inputs = IntakeIO.IntakeInputs()

    override fun periodic() {
        Logger.processInputs("Intake", inputs)
        //the wrist should always be parallel to the ground
        io.pivotToAndStop(Radians.zero()) //use position variable from arm
    }
}