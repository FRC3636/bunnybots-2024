package com.frcteam3636.frc2024.subsystems.wrist

import com.frcteam3636.frc2024.subsystems.arm.Arm
import com.frcteam3636.frc2024.subsystems.intake.IntakeIO
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Wrist: Subsystem {
    private var io: WristIO = WristIO.WristIOKraken()

    var inputs = IntakeIO.IntakeInputs()

    override fun periodic() {
        Logger.processInputs("Wrist", inputs)
        //the wrist should always be parallel to the ground
        io.pivotToAndStop(Arm.inputs.position * -1.0) //use position variable from arm
    }
}