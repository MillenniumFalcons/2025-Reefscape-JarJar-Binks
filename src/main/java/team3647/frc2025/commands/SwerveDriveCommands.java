package team3647.frc2025.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

    public Command driveCmd(
            DoubleSupplier x, // X axis on joystick is Left/Right
            DoubleSupplier y, // Y axis on Joystick is Front/Back
            DoubleSupplier rot,
			Supplier<Twist2d> autoDriveVelocities,
			Supplier<DriveMode> getMode,
			BooleanSupplier autoDriveEnabled) {
        return Commands.run(
                () -> {
					var isAutoDrive = autoDriveEnabled.getAsBoolean();
					var velocities = autoDriveVelocities.get();
                    double invert = color == Alliance.Red ? -1 : 1;

                    double ySquared =
                            Math.pow(y.getAsDouble(), 2) * Math.signum(y.getAsDouble()) * 1.05;
                    double xSquared =
                            Math.pow(x.getAsDouble(), 2) * Math.signum(x.getAsDouble()) * 1.05;

                    double motionXComponent = -ySquared * invert * kMaxSpeed.in(MetersPerSecond);
                    double motionYComponent = xSquared * invert * kMaxSpeed.in(MetersPerSecond);
                    double motionTurnComponent =
                            rot.getAsDouble() * -1 * kMaxSpeed.in(MetersPerSecond);
				if (!isAutoDrive || getMode.get().equals(DriveMode.NONE)) {
					swerve.driveFieldOriented(motionXComponent, motionYComponent, motionTurnComponent);
				}

				if (isAutoDrive && (getMode.get().equals(DriveMode.SCORE) || getMode.get().equals(DriveMode.SRCINTAKE))) {
					motionXComponent = velocities.dx + motionXComponent * 0.3;
					motionYComponent = velocities.dy + motionYComponent * 0.3;
					motionTurnComponent = velocities.dtheta + motionTurnComponent + 0.2;

					swerve.driveFieldOriented(motionXComponent, motionYComponent, motionTurnComponent);
				}

				if (isAutoDrive && getMode.get().equals(DriveMode.INTAKE)) {
					motionXComponent = velocities.dx;
					motionYComponent = velocities.dy;
					motionTurnComponent = velocities.dtheta;

					swerve.drive(motionXComponent, motionYComponent, motionTurnComponent);
				}

                },
                swerve);
    }

    public Command driveVisionTeleop(
            DoubleSupplier xSpeedFunction, // X axis on joystick is Left/Right
            DoubleSupplier ySpeedFunction, // Y axis on Joystick is Front/Back
            DoubleSupplier turnSpeedFunction,
            // BooleanSupplier slowTriggerFunction,
            // BooleanSupplier enableAutoSteer,
            BooleanSupplier getIsFieldOriented
            // Supplier<DriveMode> autoDriveMode,
            // BooleanSupplier autoDriveEnabled,
            // Supplier<Twist2d> autoDriveVelocities
            // Supplier<Twist2d> autoSteerVelocitiesSupplier
            ) {
        return Commands.run(
                () -> {
                    int invert = 1;
                    // boolean enabeld = autoDriveEnabled.getAsBoolean();
                    // DriveMode mode = autoDriveMode.get();
                    // Twist2d autoDriveTwist2d = autoDriveVelocities.get();
                    // double triggerSlow = slowTriggerFunction.getAsBoolean() ? 0.6 : 1;
                    // boolean autoSteer = enableAutoSteer.getAsBoolean();
                    boolean fieldOriented = getIsFieldOriented.getAsBoolean();
                    boolean openloop = true;

                    if (DriverStation.getAlliance().isPresent() && RobotBase.isSimulation()) {
                        if (DriverStation.getAlliance().get() == Alliance.Red) {
                            invert = -1;
                        }
                    }
                    double y =
                            Math.pow(ySpeedFunction.getAsDouble(), 2)
                                    * Math.signum(ySpeedFunction.getAsDouble())
                                    * 1.05;
                    double x =
                            Math.pow(xSpeedFunction.getAsDouble(), 2)
                                    * Math.signum(xSpeedFunction.getAsDouble())
                                    * 1.05;
                    var motionXComponent = y * invert;
                    // right stick X, (negative so that left positive)
                    var motionYComponent = -x * invert;

                    var motionTurnComponent = -turnSpeedFunction.getAsDouble();

                    // if (mode == DriveMode.SHOOT_AT_AMP && enabeld) {
                    // motionXComponent = autoDriveTwist2d.dx + motionXComponent * 0.1;
                    // motionTurnComponent = autoDriveTwist2d.dtheta + motionTurnComponent * 0.1;

                    // var translation = new Translation2d(motionXComponent, motionYComponent);

                    // var rotation = motionTurnComponent;
                    // swerve.driveFieldOriented(translation.getX(), translation.getY(), rotation);
                    // } else if (mode == DriveMode.INTAKE_IN_AUTO && enabeld) {
                    // motionXComponent = autoDriveTwist2d.dx;
                    // motionYComponent = autoDriveTwist2d.dy;

                    // var translation = new Translation2d(motionXComponent, motionYComponent);

                    // var rotation = motionTurnComponent;
                    // swerve.drive(translation.getX(), translation.getY(), rotation);
                    // } else if (mode == DriveMode.FEED && enabeld) {
                    // motionTurnComponent = autoDriveTwist2d.dtheta + motionTurnComponent;
                    // swerve.driveFieldOriented(motionXComponent, motionYComponent,
                    // motionTurnComponent);
                    // } else {

                    // if (mode != DriveMode.NONE && enabeld) {
                    // motionTurnComponent = autoDriveTwist2d.dtheta;
                    // }

                    // SmartDashboard.putNumber("theta", autoDriveTwist2d.dtheta);

                    var translation = new Translation2d(motionXComponent, motionYComponent);

                    var rotation = motionTurnComponent;
                    swerve.drive(translation.getX(), translation.getY(), rotation);
                },
                swerve);
    }
}
