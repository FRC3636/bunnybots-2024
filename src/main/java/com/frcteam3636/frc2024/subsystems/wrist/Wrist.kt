package com.frcteam3636.frc2024.subsystems.wrist

import com.frcteam3636.frc2024.subsystems.arm.Arm
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Wrist: Subsystem {
    private var io: WristIO = WristIO.WristIOKraken()

    var inputs = WristIO.Inputs()

    override fun periodic() {
        Logger.processInputs("Wrist", inputs)
        //the wrist should always be parallel to the ground
        io.pivotToAndStop(Arm.inputs.position * -1.0) //use position variable from arm
    }
}