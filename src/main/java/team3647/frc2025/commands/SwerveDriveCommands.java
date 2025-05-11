package team3647.frc2025.commands;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Second;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2025.Util.AutoDrive.DriveMode;
import team3647.frc2025.subsystems.Drivetrain.SwerveDrive;
import team3647.lib.team9442.AllianceObserver;

public class SwerveDriveCommands implements AllianceObserver {
    private final SwerveDrive swerve;
    private final double kMaxSpeedMps;
    private final double kMaxRotRadPs;
    private Alliance color = Alliance.Red;

    public SwerveDriveCommands(SwerveDrive swerve, double maxSpeedMps, double kMaxRotRadPs) {
        this.swerve = swerve;
        this.kMaxSpeedMps = maxSpeedMps;
        this.kMaxRotRadPs = kMaxRotRadPs;
    }

    @Override
    public void onAllianceFound(Alliance color) {
        this.color = color;
    }

    public Command driveVisionTeleop(
            DoubleSupplier x, // X axis on joystick is Left/Right
            DoubleSupplier y, // Y axis on Joystick is Front/Back
            DoubleSupplier rot,
            Supplier<Twist2d> autoDriveVelocities,
            Supplier<DriveMode> getMode,
            BooleanSupplier autoDriveEnabled,
            BooleanSupplier hasTargets,
            BooleanSupplier robotRelative) {

        return Commands.run(
                () -> {
                    var isAutoDrive = autoDriveEnabled.getAsBoolean();
                    var velocities = autoDriveVelocities.get();
                    var robotRel = robotRelative.getAsBoolean();
                    SmartDashboard.putBoolean("MONBEIONS", robotRel);
                    // Logger.recordOutput("hastarget@cmds", toRun);

                    int invert = 1;
                    if (DriverStation.getAlliance().isPresent()) {
                        if (DriverStation.getAlliance().get() == Alliance.Red) {
                            invert = -1;
                        }
                    }

                    double ySquared =
                            Math.pow(y.getAsDouble(), 2) * Math.signum(y.getAsDouble()) * 1.05;
                    double xSquared =
                            Math.pow(x.getAsDouble(), 2) * Math.signum(x.getAsDouble()) * 1.05;

                    double motionXComponent = ySquared * invert * kMaxSpeedMps;
                    double motionYComponent = -xSquared * invert * kMaxSpeedMps;
                    double motionTurnComponent = rot.getAsDouble() * -1 * kMaxRotRadPs;

                    if (!isAutoDrive || getMode.get().equals(DriveMode.NONE)) {

                        if (robotRel && Utils.isSimulation()) {
                            swerve.drive(-motionXComponent, -motionYComponent, motionTurnComponent);
                        } else {
                            swerve.driveFieldOriented(
                                    motionXComponent, motionYComponent, motionTurnComponent);
                        }
                    }
                    else if(isAutoDrive && getMode.get().equals(DriveMode.TEST)){
                        motionXComponent = velocities.dx + motionXComponent * 0.3;
                        motionYComponent = velocities.dy + motionYComponent * 0.3;
                        motionTurnComponent = velocities.dtheta + motionTurnComponent * 0.3;

                        swerve.driveFieldOriented(
                                motionXComponent, motionYComponent, motionTurnComponent);
                        

                    }else if (isAutoDrive && (getMode.get().equals(DriveMode.SCORE))) {
                        motionXComponent = velocities.dx + motionXComponent * 0.3;
                        motionYComponent = velocities.dy + motionYComponent * 0.3;
                        motionTurnComponent = velocities.dtheta + motionTurnComponent * 0.3;

                        swerve.driveFieldOriented(
                                motionXComponent, motionYComponent, motionTurnComponent);
                        return;
                    } else if (isAutoDrive && getMode.get().equals(DriveMode.INTAKE)) {
                        motionXComponent = velocities.dx;
                        // Logger.recordOutput("DEBUG/Coraldetection/velocities.dx", velocities.dx);
                        motionYComponent = velocities.dy;

                        swerve.drive(motionXComponent, motionYComponent, motionTurnComponent);
                    }
                },
                swerve);
    }
}
