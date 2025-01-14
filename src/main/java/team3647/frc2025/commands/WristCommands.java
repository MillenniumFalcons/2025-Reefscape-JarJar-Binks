package team3647.frc2025.commands;

import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2025.subsystems.Wrist;

public class WristCommands {
    
    public Command setAngle(double angle){
        return Commands.run(() -> wrist.setAngle(Radian.of(angle)), wrist);
    }

    

    final Wrist wrist;
    public WristCommands(Wrist wrist){
        this.wrist = wrist;
    }
}
