package org.firstinspires.ftc.teamcode.helpers.commands;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.monitoring.LoopTimeMonitor;

import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Runnable class to run FTCLib command scheduler on an
 * independent thread from the main opmode loop.
 */
@Config
public class CommandRunner implements Runnable {
    final OpModeRunningInterface runningInterface;
    private final HardwareMap hardwareMap;

    public static boolean debugCommandScheduler = false;
    private static final Logger logger = Logger.getLogger("CommandRunner");

    public CommandRunner(OpModeRunningInterface runningInterface, HardwareMap hardwareMap) {
        this.runningInterface = runningInterface;
        this.hardwareMap = hardwareMap;

        if (debugCommandScheduler) {
            CommandScheduler.getInstance().onCommandInitialize((Command command) -> commandSchedulerLog(command, "Initialized"));
            CommandScheduler.getInstance().onCommandExecute((Command command) -> commandSchedulerLog(command, "Executing"));
            CommandScheduler.getInstance().onCommandFinish((Command command) -> commandSchedulerLog(command, "Finished"));
            CommandScheduler.getInstance().onCommandInterrupt((Command command) -> commandSchedulerLog(command, "Interrupted"));
        }
    }

    private void commandSchedulerLog(Command command, String message) {
        logger.log(Level.INFO, message + " " + command.getClass().getSimpleName() + " (" + command.hashCode() + ")");
    }

    public void run() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        while (!runningInterface.isOpModeRunning()) {
            try {
                sleep(10); // Wait for the opmode to start to start running commands
            } catch (InterruptedException e) {
                throw new RuntimeException(e); // some stupid shit so it compiles
            }
        }

        while (runningInterface.isOpModeRunning()) {
            while (runningInterface.isOpModeRunning()) {

                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }

                CommandScheduler.getInstance().run();
            }
        }
    }
}