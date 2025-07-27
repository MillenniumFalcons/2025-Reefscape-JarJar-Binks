package team3647.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2025.subsystems.Kicker;

public class KickerCommands {
    Kicker kicker;

    public KickerCommands(Kicker kicker) {
        this.kicker = kicker;
    }

    public Command setOpenLoop(double output) {
        return Commands.run(() -> kicker.setOpenLoop(output), kicker);
    }

    public Command kill() {
        return Commands.run(() -> setOpenLoop(0), kicker).withTimeout(0.3);
    }
}
