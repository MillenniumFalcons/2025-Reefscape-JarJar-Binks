// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2025.robot;

import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2025.commands.SwerveDriveCommands;
import team3647.frc2025.constants.GlobalConstants;
import team3647.frc2025.constants.SwerveDriveConstants;
import team3647.frc2025.constants.TunerConstants;
import team3647.frc2025.subsystems.SwerveDrive;
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceChecker;
import team3647.lib.team9442.AllianceObserver;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
        configureAllianceObservers();
    }


    private void configureAllianceObservers(){
        allianceChecker.registerObservers(swerveCommands, swerve);
    }

    private void configureBindings() {
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(swerveCommands.driveCmd(mainController::getLeftStickX, mainController::getLeftStickY,
                mainController::getRightStickX));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    @SuppressWarnings("unchecked")
    public final SwerveDrive swerve = new SwerveDrive(
            TunerConstants.DrivetrainConstants,
            SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
            SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec,
            GlobalConstants.kDt,
            SwerveDriveConstants.simConfig,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);

    public final SwerveDriveCommands swerveCommands = new SwerveDriveCommands(swerve);

    public final Joysticks mainController = new Joysticks(0);

    public final AllianceChecker allianceChecker = new AllianceChecker();
}
