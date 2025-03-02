package team3647.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2025.subsystems.Climb;

public class ClimbCommands {

    public Command climbOut() {
        return Commands.run(() -> climb.setOpenLoop(0.9), climb);
    }

    public Command climbIn() {
        return Commands.run(() -> climb.setOpenLoop(-0.9), climb);
    }

    public Command kill() {
        return Commands.run(() -> climb.setOpenLoop(0.0), climb);
    }

    Climb climb;

    public ClimbCommands(Climb climb) {
        this.climb = climb;
    }
}
