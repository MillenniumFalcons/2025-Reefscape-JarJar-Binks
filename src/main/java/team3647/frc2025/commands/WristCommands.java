package team3647.frc2025.commands;

import static edu.wpi.first.units.Units.Degree;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.WristConstants;
import team3647.frc2025.subsystems.Wrist;

public class WristCommands {
	public final Wrist wrist;

	public WristCommands(Wrist wrist){
		this.wrist = wrist;
	}

	public Command goToIntake(){
		return Commands.run(
			() -> wrist.setAngle(WristConstants.kIntakeAngle), 
			wrist);
	}

	public Command stow(){
		return setAngle(WristConstants.kStowAngle);
	}

	public Command setAngle(Angle angle){
		return Commands.run(
			() -> wrist.setAngle(angle), wrist).until(() -> wrist.angleReached(angle.in(Degree), 2));
	}

	public Command setAngle(Supplier<Angle> angle){
		return Commands.run(
			() -> wrist.setAngle(angle.get()), wrist);
	}

	public Command holdPositionAtCall() {
        return new Command() {
            Angle degreeAtStart = WristConstants.kStowAngle;

            @Override
            public void initialize() {
                degreeAtStart = wrist.getAngle();
            }

            @Override
            public void execute() {
                wrist.setAngle(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(wrist);
            }
		};
	}

	
}
