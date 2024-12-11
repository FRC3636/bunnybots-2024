package com.frcteam3636.frc2024.subsystems.arm

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.subsystems.intake.Intake
import com.frcteam3636.frc2024.subsystems.intake.Intake.indexerAngleLigament
import com.frcteam3636.frc2024.subsystems.intake.Intake.intakeAngleLigament
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.littletonrobotics.junction.Logger
import kotlin.math.cos
import kotlin.math.sin


private const val SECONDS_BETWEEN_ARM_UPDATES = 2.0

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

    private var timer = Timer().apply {
        start()
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("/Arm", inputs)
        armAngleLigament.angle = inputs.position.`in`(Degrees)
        armWristAngleLigament.angle = 90.0 - inputs.position.`in`(Degrees)

        if (timer.advanceIfElapsed(SECONDS_BETWEEN_ARM_UPDATES) && inputs.absoluteEncoderConnected){
                io.updatePosition(inputs.absoluteEncoderPosition)
        }
        Logger.recordOutput("/Arm/Mechanism", mechanism)

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