package team3647.frc2025.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2025.subsystems.SwerveDrive;
import team3647.lib.team9442.AllianceObserver;

public class SwerveDriveCommands implements AllianceObserver {
    private final SwerveDrive swerve;
    private Alliance color = Alliance.Red;

    public SwerveDriveCommands(SwerveDrive swerve) {
        this.swerve = swerve;
    }

    @Override
    public void onAllianceFound(Alliance color) {
        this.color = color;
    }

    public Command driveCmd(
        DoubleSupplier x,
        DoubleSupplier y,
        DoubleSupplier rot
    ){
        return Commands.run(() -> {
            
            double invert = color == Alliance.Red? -1 : 1;
            swerve.driveFieldOriented(x.getAsDouble() * invert, y.getAsDouble() * invert, -rot.getAsDouble());
        }, swerve);
    }
}
