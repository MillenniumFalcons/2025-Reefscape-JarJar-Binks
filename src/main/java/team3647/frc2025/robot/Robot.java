// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2025.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import team3647.frc2025.constants.LEDConstants;
import team3647.lib.ModifiedSignalLogger;
import team3647.lib.team6328.VirtualSubsystem;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private final boolean isReplay = false;

    private final boolean rio1 = false;

    public Robot() {
        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        if (isReal() && !rio1) {
            Logger.addDataReceiver(
                    new WPILOGWriter("/home/lvuser/logs")); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution nlogging
        } else if (rio1) {
            Logger.addDataReceiver(new NT4Publisher());

        } else {
            setUseTiming(false); // Run as fast as possible
            if (isReplay) {
                String logPath =
                        LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or
                // prompt the
                // user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(
                        new WPILOGWriter(
                                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to
                // a new log
            } else {
                Logger.addDataReceiver(new NT4Publisher());
            }
        }

        AutoLogOutputManager.addPackage("team3647.lib");

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata
        // values may
        // be added.
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);
        SignalLogger.stop();
        SignalLogger.enableAutoLogging(false);
        SignalLogger.stop();
        // SignalLogger.start();
        // ModifiedSignalLogger.start();
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Logger.recordOutput(
                "Superstructure/pivotoffset", m_robotContainer.superstructure.getPivotOffset());
        Logger.recordOutput(
                "Superstructure/elevOffset", m_robotContainer.superstructure.getElevOffset());
        Logger.recordOutput(
                "Superstructure/Level", m_robotContainer.superstructure.getWantedLevel());

        Logger.recordOutput("Robot/mode", m_robotContainer.autoDrive.getWantedMode());

        Logger.recordOutput(
                "Superstructure/shouldClear", m_robotContainer.superstructure.needToClear());

        m_robotContainer.updateRobotPoseForSmartdashboard();
        VirtualSubsystem.periodicAll();
        LEDConstants.m_candle.animate(LEDConstants.RAINBOW);
    }

    @Override
    public void disabledInit() {
        SignalLogger.stop();
        ModifiedSignalLogger.stop();
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.allianceChecker.periodic();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {

        SimulatedArena.getInstance().simulationPeriodic();
    }
}
