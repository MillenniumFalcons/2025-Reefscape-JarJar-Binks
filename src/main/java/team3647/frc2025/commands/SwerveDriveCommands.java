package team3647.frc2025.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import team3647.frc2025.Util.AutoDrive.DriveMode;
import team3647.frc2025.subsystems.SwerveDrive;
import team3647.lib.team9442.AllianceObserver;

public class SwerveDriveCommands implements AllianceObserver {
    private final SwerveDrive swerve;
    private final LinearVelocity kMaxSpeed;
    private Alliance color = Alliance.Red;

    public SwerveDriveCommands(SwerveDrive swerve, LinearVelocity maxSpeed) {
        this.swerve = swerve;
        this.kMaxSpeed = maxSpeed;
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
			BooleanSupplier slowMode) {
        var corrector = new PIDController(1, 0, 0);
        return Commands.run(
                () -> {
                    var isAutoDrive = autoDriveEnabled.getAsBoolean();
                    var velocities = autoDriveVelocities.get();
                    var toRun = hasTargets.getAsBoolean();
					var slow = slowMode.getAsBoolean()? 0.4 : 1;
					Logger.recordOutput("hastarget@cmds", toRun);

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

                    double motionXComponent = ySquared * invert * kMaxSpeed.in(MetersPerSecond) * slow;
                    double motionYComponent = -xSquared * invert * kMaxSpeed.in(MetersPerSecond) * slow;
                    double motionTurnComponent =
                            rot.getAsDouble() * -1 * kMaxSpeed.in(MetersPerSecond);

                    if (!isAutoDrive || getMode.get().equals(DriveMode.NONE)) {

                        swerve.driveFieldOriented(
                                motionXComponent, motionYComponent, motionTurnComponent);

                    } else if (isAutoDrive && (getMode.get().equals(DriveMode.SCORE))) {
                        if (toRun) {
							// DriverStation.reportError("FIREEEEEEEEE", false);
                            // motionXComponent = velocities.dx + motionXComponent * 0.3;
                            motionYComponent = velocities.dy + motionYComponent * 0.3;
                        }
						
                        motionTurnComponent = velocities.dtheta + motionTurnComponent * 0.8;

                        swerve.driveFieldOriented(motionXComponent, motionYComponent, motionTurnComponent);
                        return;
                    } else if (isAutoDrive && getMode.get().equals(DriveMode.INTAKE)) {
                        motionXComponent = velocities.dx;
                        motionYComponent = velocities.dy;
                        motionTurnComponent = velocities.dtheta;

                        swerve.drive(motionXComponent, motionYComponent, motionTurnComponent);
                    }
                },
                swerve);
    }
}
