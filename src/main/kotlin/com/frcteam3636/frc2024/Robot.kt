package com.frcteam3636.frc2024

import com.ctre.phoenix6.StatusSignal
import com.frcteam3636.frc2024.subsystems.arm.Arm
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain
import com.frcteam3636.frc2024.subsystems.indexer.Indexer
import com.frcteam3636.frc2024.subsystems.intake.Intake
import com.frcteam3636.version.BUILD_DATE
import com.frcteam3636.version.DIRTY
import com.frcteam3636.version.GIT_BRANCH
import com.frcteam3636.version.GIT_SHA
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.cscore.VideoException
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.PatchedLoggedRobot
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import java.util.*
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI


/**
 * The VM is configured to automatically run this object (which basically functions as a singleton
 * class), and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. This is written as an object rather than a class since there should only ever be a
 * single instance, and it cannot take any constructor arguments. This makes it a natural fit to be
 * an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also
 * update the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when
 * renaming the object or package, it will get changed everywhere.)
 */
object Robot : PatchedLoggedRobot() {
    private val controller = CommandXboxController(2)
    private val joystickLeft = Joystick(0)
    private val joystickRight = Joystick(1)

    @Suppress("unused")
    private val joystickDev = Joystick(3)

    /** A model of robot, depending on where we're deployed to. */
    enum class Model {
        SIMULATION, COMPETITION, PROTOTYPE
    }

    /** The model of this robot. */
    val model: Model = if (RobotBase.isSimulation()) {
        Model.SIMULATION
    } else {
        when (val key = Preferences.getString("Model", "competition")) {
            "competition" -> Model.COMPETITION
            "prototype" -> Model.PROTOTYPE
            else -> throw AssertionError("Invalid model found in preferences: $key")
        }
    }

    private var autoCommand: Command = Drivetrain.defer {
            val xMagnitude = 5.6
            val xOffset = if (DriverStation.getAlliance() == Optional.of(Alliance.Red)) {
                xMagnitude
            } else {
                -xMagnitude
            }

            Drivetrain.pathfindToPose(Drivetrain.estimatedPose
                + Transform2d(Translation2d(xOffset, 0.0), Rotation2d()))
        }
        .finallyDo(Runnable { Drivetrain.brake() })
        .andThen(Intake.intake()
            .alongWith(Indexer.outtakeBalloon()))

    /** Status signals used to check the health of the robot's hardware */
    val statusSignals = mutableMapOf<String, StatusSignal<*>>()

    override fun robotInit() {
        // Report the use of the Kotlin Language for "FRC Usage Report" statistics
        HAL.report(
            tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version
        )

        // Joysticks are likely to be missing in simulation, which usually isn't a problem.
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation())

        configureAdvantageKit()
        configureSubsystems()
        configureAutos()
        configureBindings()
        configureDashboard()

        try {
            val camera = CameraServer.startAutomaticCapture()
            camera.setResolution(160, 160)
        } catch (err: VideoException) {
            print("Failed to start driver camera")
        }
    }

    /** Start logging or pull replay logs from a file */
    private fun configureAdvantageKit() {
        Logger.recordMetadata("Git SHA", GIT_SHA)
        Logger.recordMetadata("Build Date", BUILD_DATE)
        Logger.recordMetadata("Git Tree Dirty", (DIRTY == 1).toString())
        Logger.recordMetadata("Git Branch", GIT_BRANCH)
        Logger.recordMetadata("Model", model.name)

        if (isReal()) {
            // This competition we aren't using a USB log stick because there's no extra USB port on the RoboRIO.
//            Logger.addDataReceiver(WPILOGWriter()) // Log to a USB stick
//            if (!Path("/U").exists()) {
//                Elastic.sendAlert(
//                    ElasticNotification(
//                        "Insert Log USB drive into RoboRIO",
//                        "This match will not have debug logs.",
//                        NotificationLevel.WARNING
//                    )
//                )
//            }
            Logger.addDataReceiver(WPILOGWriter("/home/lvuser/logs")) // Log on-device instead

            Logger.addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
            // Enables power distribution logging
            if (model == Model.COMPETITION) {
                PowerDistribution(
                    62, PowerDistribution.ModuleType.kRev
                )
            } else {
                PowerDistribution(
                    1, PowerDistribution.ModuleType.kCTRE
                )
            }
        } else {
            val logPath = try {
                // Pull the replay log from AdvantageScope (or prompt the user)
                LogFileUtil.findReplayLog()
            } catch (_: java.util.NoSuchElementException) {
                null
            }

            if (logPath == null) {
                // No replay log, so perform physics simulation
                Logger.addDataReceiver(NT4Publisher())
            } else {
                // Replay log exists, so replay data
                setUseTiming(false) // Run as fast as possible
                Logger.setReplaySource(WPILOGReader(logPath)) // Read replay log
                Logger.addDataReceiver(
                    WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))
                ) // Save outputs to a new log
            }
        }
        Logger.start() // Start logging! No more data receivers, replay sources, or metadata values may be added.
    }

    /** Start robot subsystems so that their periodic tasks are run */
    private fun configureSubsystems() {
        Drivetrain.register()
        Indexer.register()
        Intake.register()
        Arm.register()
    }

    /** Expose commands for autonomous routines to use and display an auto picker in Shuffleboard. */
    private fun configureAutos() {
//        NamedCommands.registerCommand(
//            "revAim",
//            Commands.parallel(
//                Shooter.Pivot.followMotionProfile(Shooter.Pivot.Target.AIM),
//                Shooter.Flywheels.rev(580.0, 0.0)
//            )
//        )
    }

    /** Configure which commands each joystick button triggers. */
    private fun configureBindings() {
         Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks(joystickLeft, joystickRight)
        Indexer.defaultCommand = Indexer.autoRun()

        // (The button with the yellow tape on it)
        JoystickButton(joystickLeft, 8).onTrue(Commands.runOnce({
            println("Zeroing gyro.")
            Drivetrain.zeroGyro()
        }).ignoringDisable(true))

        // Intake
        controller.leftBumper()
            .whileTrue(Intake.intake())

        controller.rightBumper()
            .whileTrue(Intake.intake())

        controller.rightTrigger().whileTrue(Indexer.indexBalloon())
        controller.leftTrigger().whileTrue(Indexer.outtakeBalloon())




//        Arm positions
        controller.y()
            .onTrue(
                Arm.moveToPosition(Arm.Position.Stowed)
            )

        controller.x()
            .onTrue(
                Arm.moveToPosition(Arm.Position.PickUp)
            )


        controller.a()
            .onTrue(
                Arm.moveToPosition(Arm.Position.Lower))


        controller.b()
            .whileTrue(Arm.coastMode().ignoringDisable(true))

        controller.button(7)
            .and(controller.button(8))
            .and(controller.y())
            .onTrue(Arm.zeroIt().ignoringDisable(true))

    }

    /** Add data to the driver station dashboard. */
    private fun configureDashboard() {
        Dashboard.showTeleopTab(Shuffleboard.getTab("Teleoperated"))
    }

    override fun robotPeriodic() {
        Dashboard.update()
        Diagnostics.collect(statusSignals).reportAlerts()
        CommandScheduler.getInstance().run()
    }


    override fun autonomousInit() {
        Drivetrain.zeroGyro(inverted = true)
        Drivetrain.estimatedPose = Pose2d(if (DriverStation.getAlliance() == Optional.of(Alliance.Blue)) {
            7.0
        } else {
            9.0
        }, 4.0, Drivetrain.estimatedPose.rotation)
        autoCommand.schedule()
    }

    override fun autonomousExit() {
        autoCommand.cancel()
    }

    override fun teleopInit() {
    }

    override fun testInit() {
    }

    override fun testExit() {
    }
}
