package team3647.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2025.subsystems.rollers;

public class RollersCommands {
    rollers rollers;

    public RollersCommands(rollers rollers){
        this.rollers = rollers;
    }

    public Command setOpenLoop(double output){
        return Commands.run(() -> rollers.setOpenLoop(output), rollers);

    }
    public boolean currentGreater(double current){
        return rollers.getMasterCurrent() > current;
    }

    public Command kill(){
        return setOpenLoop(0);
    }
}
