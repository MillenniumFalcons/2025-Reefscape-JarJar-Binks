package team3647.frc2025.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2025.subsystems.Elevator;

public class ElevatorCommands {
    
    
    
    
    public Command setHeight(double height){
        return Commands.run(() -> {elevator.setHeight(height);}, elevator);
    }
    
    public Command setHeight(DoubleSupplier height){
        return Commands.run(() -> {elevator.setHeight(height.getAsDouble());}, elevator);
    }
    
    
    Elevator elevator;
    public ElevatorCommands(Elevator elevator){
        this.elevator =elevator;
    }
}
