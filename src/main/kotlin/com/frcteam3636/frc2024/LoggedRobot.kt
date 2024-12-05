package org.littletonrobotics.junction

import edu.wpi.first.hal.DriverStationJNI
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.hal.NotifierJNI
import edu.wpi.first.wpilibj.IterativeRobotBase
import org.littletonrobotics.junction.AutoLogOutputManager
import org.littletonrobotics.junction.CheckInstall
import org.littletonrobotics.junction.Logger
import java.lang.management.GarbageCollectorMXBean
import java.lang.management.ManagementFactory


/**
 * LoggedRobot implements the IterativeRobotBase robot program framework.
 *
 *
 *
 * The LoggedRobot class is intended to be subclassed by a user creating a robot
 * program, and will call all required AdvantageKit periodic methods.
 *
 *
 *
 * periodic() functions from the base class are called on an interval by a
 * Notifier instance.
 */
open class MyLoggedRobot protected constructor(period: Double = defaultPeriodSecs) :
    IterativeRobotBase(period) {
    private val notifier = NotifierJNI.initializeNotifier()
    private val periodUs = (period * 1000000).toLong()
    private var nextCycleUs: Long = 0
    private val gcStatsCollector = GcStatsCollector()

    private var useTiming = true

    /**
     * Constructor for LoggedRobot.
     *
     * @param period Period in seconds.
     */
    /** Constructor for LoggedRobot.  */
    init {
        NotifierJNI.setNotifierName(notifier, "LoggedRobot")

        HAL.report(FRCNetComm.tResourceType.kResourceType_Framework, FRCNetComm.tInstances.kFramework_AdvantageKit)
    }

    protected fun finalize() {
        NotifierJNI.stopNotifier(notifier)
        NotifierJNI.cleanNotifier(notifier)
    }

    /** Provide an alternate "main loop" via startCompetition().  */
    override fun startCompetition() {
        // Robot init methods
        val initStart = Logger.getRealTimestamp()
        robotInit()
        if (isSimulation()) {
            simulationInit()
        }
        val initEnd = Logger.getRealTimestamp()

        // Register auto logged outputs
        AutoLogOutputManager.registerFields(this)

        // Save data from init cycle
        Logger.periodicAfterUser(initEnd - initStart, 0)

        // Tell the DS that the robot is ready to be enabled
        println("********** Robot program startup complete **********")
        DriverStationJNI.observeUserProgramStarting()

        // Loop forever, calling the appropriate mode-dependent function
        while (true) {
            if (useTiming) {
                val currentTimeUs = Logger.getRealTimestamp()
                if (nextCycleUs < currentTimeUs) {
                    // Loop overrun, start next cycle immediately
                    nextCycleUs = currentTimeUs
                } else {
                    // Wait before next cycle
                    NotifierJNI.updateNotifierAlarm(notifier, nextCycleUs)
                    NotifierJNI.waitForNotifierAlarm(notifier)
                }
                nextCycleUs += periodUs
            }

            val periodicBeforeStart = Logger.getRealTimestamp()
            Logger.periodicBeforeUser()
            val userCodeStart = Logger.getRealTimestamp()
            loopFunc()
            val userCodeEnd = Logger.getRealTimestamp()

            gcStatsCollector.update()
            Logger.periodicAfterUser(userCodeEnd - userCodeStart, userCodeStart - periodicBeforeStart)
        }
    }

    /** Ends the main loop in startCompetition().  */
    override fun endCompetition() {
        NotifierJNI.stopNotifier(notifier)
    }

    /** Sets whether to use standard timing or run as fast as possible.  */
    fun setUseTiming(useTiming: Boolean) {
        this.useTiming = useTiming
    }

    private class GcStatsCollector {
        private val gcBeans: List<GarbageCollectorMXBean> = ManagementFactory.getGarbageCollectorMXBeans()
        private val lastTimes = LongArray(gcBeans.size)
        private val lastCounts = LongArray(gcBeans.size)

        fun update() {
            var accumTime: Long = 0
            var accumCounts: Long = 0
            for (i in gcBeans.indices) {
                val gcTime = gcBeans[i].collectionTime
                val gcCount = gcBeans[i].collectionCount
                accumTime += gcTime - lastTimes[i]
                accumCounts += gcCount - lastCounts[i]

                lastTimes[i] = gcTime
                lastCounts[i] = gcCount
            }

            Logger.recordOutput("LoggedRobot/GCTimeMS", accumTime.toDouble())
            Logger.recordOutput("LoggedRobot/GCCounts", accumCounts.toDouble())
        }
    }

    companion object {
        const val defaultPeriodSecs: Double = 0.02
    }
}
