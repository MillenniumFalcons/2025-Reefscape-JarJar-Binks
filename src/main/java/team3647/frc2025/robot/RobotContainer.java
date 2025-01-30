// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2025.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2025.autos.AutoCommands;
import team3647.frc2025.commands.SwerveDriveCommands;
import team3647.frc2025.constants.AutoConstants;
import team3647.frc2025.constants.GlobalConstants;
import team3647.frc2025.constants.SwerveDriveConstants;
import team3647.frc2025.constants.TunerConstants;
import team3647.frc2025.constants.TunerSimConstants;
import team3647.frc2025.subsystems.SwerveDrive;
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceChecker;
import team3647.lib.team9442.AllianceObserver;
import team3647.lib.team9442.AutoChooser;
import team3647.lib.vision.AprilTagLimelight;
import team3647.lib.vision.AprilTagPhotonVision;
import team3647.lib.vision.VisionController;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
        configureAllianceObservers();
		SmartDashboard.putData(autoChooser);
		swerve.setRobotPose(new Pose2d(2,2,new Rotation2d()));
    }


    private void configureAllianceObservers(){
        allianceChecker.registerObservers(swerveCommands, swerve, swerveCommands, autoCommands, autoChooser);
    }

    private void configureBindings() {
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(swerveCommands.driveCmd(mainController::getLeftStickX, mainController::getLeftStickY,
                mainController::getRightStickX));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected().getAutoCommand();
    }



    @SuppressWarnings("unchecked")
    public final SwerveDrive swerve = new SwerveDrive(
            TunerConstants.DrivetrainConstants,
            SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
            SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec,
            GlobalConstants.kDt,
            TunerSimConstants.FrontLeft,
            TunerSimConstants.FrontRight,
            TunerSimConstants.BackLeft,
            TunerSimConstants.BackRight);

    public final SwerveDriveCommands swerveCommands = new SwerveDriveCommands(swerve, MetersPerSecond.of(SwerveDriveConstants.kDrivePossibleMaxSpeedMPS));

    public final Joysticks mainController = new Joysticks(0);

    public final AllianceChecker allianceChecker = new AllianceChecker();

	public final AutoConstants autoConstants = new AutoConstants();

	public final AutoCommands autoCommands = new AutoCommands(autoConstants, swerve);

	public final AutoChooser autoChooser = new AutoChooser(autoCommands, swerve::setRobotPose);

	AprilTagPhotonVision cam1ChangeName = new AprilTagPhotonVision("ballschangename", new Transform3d(), VecBuilder.fill(1,1,1));

	AprilTagLimelight ll1ChangeName = new AprilTagLimelight("LL1ChangeName", new Transform3d(), swerve::getPigeonOrientation, VecBuilder.fill(1,1,1));

	public final VisionController controller = new VisionController(swerve::addVisionData, swerve::shouldAddData, swerve::resetPose, cam1ChangeName, ll1ChangeName);
}
