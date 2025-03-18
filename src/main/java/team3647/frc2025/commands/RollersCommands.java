package team3647.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2025.subsystems.Rollers;
import team3647.frc2025.subsystems.Seagull;

public class RollersCommands {
    Rollers rollers;
    Seagull seagull;

    public RollersCommands(Rollers rollers, Seagull seagull) {
        this.rollers = rollers;
        this.seagull = seagull;
    }

    public Command setOpenLoop(double output) {
        return Commands.run(
                () -> {
                    rollers.setOpenLoop(output);
                    seagull.setOpenLoop(output);
                },
                rollers,
                seagull);
    }

    public Command setOpenLoop(double rollersOut, double seagullOut) {
        return Commands.run(
                () -> {
                    rollers.setOpenLoop(rollersOut);
                    seagull.setOpenLoop(seagullOut);
                },
                rollers,
                seagull);
    }

    public boolean currentGreater(double current) {
        return rollers.getMasterCurrent() >= current;
    }

    public boolean seagullCurrentGreater(double current) {
        return seagull.getMasterCurrent() > current;
    }

    public Command kill() {
        return setOpenLoop(0).withTimeout(0.3);
    }
}
