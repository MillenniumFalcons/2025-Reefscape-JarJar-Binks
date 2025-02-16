package team3647.frc2025.commands;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.subsystems.Elevator;

public class ElevatorCommands {

    public Command setHeight(double height) {
        return Commands.run(() -> elevator.setHeight(height), elevator)
                .until(() -> elevator.heightReached(Meters.of(height), Inches.of(1)));
    }

	public Command setHeightNative(double height) {
        return Commands.run(() -> elevator.setHeightNative(height), elevator)
                .until(() -> elevator.heightReached(Meters.of(height), Inches.of(1)));
    }
	

    public Command setHeight(DoubleSupplier height) {
        return Commands.run(() -> elevator.setHeight(height.getAsDouble()), elevator)
                .until(() -> elevator.heightReached(Meters.of(height.getAsDouble()), Inches.of(1)));
    }

    public Command setHeight(Supplier<Distance> height) {
        return Commands.run(() -> elevator.setHeight(height.get()), elevator)
                .until(() -> elevator.heightReached(height.get(), Inches.of(1)));
    }

    public Command setHeight(Distance height) {
        return Commands.run(() -> elevator.setHeight(height), elevator)
                .until(() -> elevator.heightReached(height, Inches.of(1)));
    }

		public Command holdPositionAtCall() {
        return new Command() {
            Distance degreeAtStart = ElevatorConstants.kStowHeight;

            @Override
            public void initialize() {
                degreeAtStart = elevator.getHeight();
            }

            @Override
            public void execute() {
                elevator.setHeight(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(elevator);
            }
		};
	}

    public Command setOpenLoop(DoubleSupplier output) {
        return Commands.run(() -> elevator.setOpenLoop(output.getAsDouble()), elevator);
    }

	// public Command holdPositionAtCall(){
	// 	Distance startingPosition;
	// 	return new FunctionalCommand(
	// 		() -> startingPosition = elevator.getHeight(), 
	// 		() -> elevator., null, null, null)
	// }

    Elevator elevator;

    public ElevatorCommands(Elevator elevator) {
        this.elevator = elevator;
    }
}
