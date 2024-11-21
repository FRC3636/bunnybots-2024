package com.frcteam3636.frc2024.subsystems.indexer

import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.subsystems.intake.Intake
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Indexer: Subsystem {
    private var io: IndexerIO = when (Robot.model) {
        Robot.Model.SIMULATION -> IndexerIOSim()
        Robot.Model.COMPETITION -> IndexerIOReal()
        Robot.Model.PROTOTYPE -> IndexerIOPrototype()
    }

    var inputs = IndexerIO.Inputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)

        Intake.indexerAngleLigament.angle = inputs.position.`in`(Degrees)
        Intake.indexerAngleLigament.color = when (inputs.balloonState) {
            BalloonState.Blue -> Color8Bit(Color.kBlue)
            BalloonState.Red -> Color8Bit(Color.kRed)
            else -> Color8Bit(Color.kGreen)
        }
    }

    /**
     * Runs the indexer forward if balloon matches the current alliance.
     * Does not run if no balloon.
     * Reverses if balloon is wrong alliance.
     */
    fun autoRun(): Command =
        runEnd(
            {
                DriverStation.getAlliance().map {
                    if (
                        (inputs.balloonState == BalloonState.Blue && it == DriverStation.Alliance.Blue)
                        || (inputs.balloonState == BalloonState.Red && it == DriverStation.Alliance.Red)
                    ) {
                        io.setSpinSpeed(0.5)
                    } else if (inputs.balloonState == BalloonState.None) {
                        io.setSpinSpeed(0.0)
                    } else {
                        io.setSpinSpeed(-0.5)
                    }
                }
            },
            {
                io.setSpinSpeed(0.0)
            }
        )

    fun indexBalloon(): Command = runEnd(
        {io.setSpinSpeed(0.5)},
        {io.setSpinSpeed(0.0)}
    )

    fun outtakeBalloon(): Command = runEnd(
        {io.setSpinSpeed(-0.5)},
        {io.setSpinSpeed(0.0)}
    )
}