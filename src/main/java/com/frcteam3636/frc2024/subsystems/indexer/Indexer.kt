package com.frcteam3636.frc2024.subsystems.indexer

import com.revrobotics.ColorMatch
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.startEnd
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Indexer: Subsystem {
    private var io: IndexerIO = IndexerIOReal()
    private var colorMatcher = ColorMatch().apply {
        addColorMatch(RED_BALLOON_COLOR)
        addColorMatch(BLUE_BALLOON_COLOR)
    }

    var inputs = IndexerIO.IndexerInputs()
    var currentColor: Color? = null

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)
        currentColor = colorMatcher.matchClosestColor(inputs.balloonColor).color
    }

    fun progressBalloon(): Command =
        startEnd(
            {io.setSpinSpeed(0.5)},
            {io.setSpinSpeed(0.0)}
        )

    fun outtakeBalloon(): Command =
        startEnd(
            {io.setSpinSpeed(-0.5)},
            {io.setSpinSpeed(0.0)}
        )

    val RED_BALLOON_COLOR = Color(255, 0, 0)
    val BLUE_BALLOON_COLOR = Color(0, 0, 255)

}