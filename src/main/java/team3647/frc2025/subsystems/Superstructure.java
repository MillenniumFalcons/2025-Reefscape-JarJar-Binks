package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.objdetect.CascadeClassifier;

import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.commands.CoralerCommands;
import team3647.frc2025.commands.ElevatorCommands;
import team3647.frc2025.commands.PivotCommands;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants.ScoringPos;
import team3647.frc2025.constants.PivotConstants;

public class Superstructure {

	public final Coraler coraler;
	public final Elevator elevator;
	public final Pivot pivot;

	public final CoralerCommands coralerCommands;
	public final ElevatorCommands elevatorCommands;
	public final PivotCommands pivotCommands;

	private BooleanSupplier isAligned;

	private Level wantedLevel;

	public ScoringPos wantedScoringPos = ScoringPos.NONE;

	private Side wantedSide;

	private Branch wantedBranch;

	public enum Side {
		A,
		B,
		C,
		D,
		E,
		F
	}

	public enum Branch {
		ONE,
		TWO
	}

	public Superstructure(
			Coraler coraler, Elevator elevator, Pivot pivot) {
		this.coraler = coraler;
		this.elevator = elevator;
		this.pivot = pivot;

		this.coralerCommands = new CoralerCommands(this.coraler);
		this.elevatorCommands = new ElevatorCommands(this.elevator);
		this.pivotCommands = new PivotCommands(this.pivot);

		this.wantedLevel = Level.NONE;
		this.isAligned = () -> false;
	}

	public enum Level {
		TROUGH,
		LOW,
		MID,
		HIGH,
		INTAKE,
		NONE
	}

	public void setIsAlignedFunction(BooleanSupplier isAligned){
		this.isAligned = isAligned;
	}

	public SuperstructureState getWantedStateByLevel(Level level) {
		switch (level) {
			case TROUGH:
				return new SuperstructureState(
						PivotConstants.kLevel1Angle, ElevatorConstants.kLevel1Height, -0.7);
			case LOW:
				return new SuperstructureState(
						PivotConstants.kLevel2Angle, ElevatorConstants.kLevel2Height, -0.7);
			case MID:
				return new SuperstructureState(
						PivotConstants.kLevel3Angle, ElevatorConstants.kLevel3Height, -0.7);
			case HIGH:
				return new SuperstructureState(
						PivotConstants.kLevel4Angle, ElevatorConstants.kLevel4Height, -0.7);
			case INTAKE:
				return new SuperstructureState(
						PivotConstants.kIntakeAngle, ElevatorConstants.kIntakeHeight, 1);
			default:
				return new SuperstructureState(
						PivotConstants.kStowAngle, ElevatorConstants.kStowHeight, 0);
		}
	}

	public SuperstructureState getWantedStateByWantedLevel() {
		return getWantedStateByLevel(wantedLevel);
	}


	public void setWantedScoringPosBySideBranch() {
		switch (wantedSide) {
			case A:
				switch (wantedBranch) {
					case ONE:
						wantedScoringPos = ScoringPos.A1;
						break;
					case TWO:
						wantedScoringPos = ScoringPos.A2;
						break;
					default:
						DriverStation.reportError("Cannot set Wanted Scoring pos, no branch provided", false);
						break;
				}
				break;
			case B:

				switch (wantedBranch) {
					case ONE:
						wantedScoringPos = ScoringPos.B1;
						break;
					case TWO:
						wantedScoringPos = ScoringPos.B2;
						break;
					default:
						DriverStation.reportError("Cannot set Wanted Scoring pos, no branch provided", false);
						break;
				}
			case C:
				switch (wantedBranch) {
					case ONE:
						wantedScoringPos = ScoringPos.C1;
						break;
					case TWO:
						wantedScoringPos = ScoringPos.C2;
						break;
					default:
						DriverStation.reportError("Cannot set Wanted Scoring pos, no branch provided", false);
						break;
				}
			case D:
				switch (wantedBranch) {
					case ONE:
						wantedScoringPos = ScoringPos.D1;
						break;
					case TWO:
						wantedScoringPos = ScoringPos.D2;
						break;
					default:
						DriverStation.reportError("Cannot set Wanted Scoring pos, no branch provided", false);
						break;
				}
			case E:
				switch (wantedBranch) {
					case ONE:
						wantedScoringPos = ScoringPos.E1;
						break;
					case TWO:
						wantedScoringPos = ScoringPos.E2;
						break;
					default:
						DriverStation.reportError("Cannot set Wanted Scoring pos, no branch provided", false);
						break;
				}
			case F:
				switch (wantedBranch) {
					case ONE:
						wantedScoringPos = ScoringPos.F1;
						break;
					case TWO:
						wantedScoringPos = ScoringPos.F2;
						break;
					default:
						DriverStation.reportError("Cannot set Wanted Scoring pos, no branch provided", false);
						break;
				}
			default:
				DriverStation.reportError("Cannot set Wanted scoring pos, no SIDE provided", false);
				break;
		}
	}

	public ScoringPos getWantedScoringPos() {
		return wantedScoringPos;
	}

	/**
	 * NOTE: WILL CHANGE BASED ON INTAKE GEOMETRY
	 *
	 * @return
	 */
	public Command goToStateParalell(SuperstructureState state) {
		return Commands.parallel(
				elevatorCommands.setHeight(state.elevatorHeight),
				pivotCommands.setAngle(state.pivotAngle),
				coralerCommands.setOpenLoop(state.rollers).withTimeout(0.5));
	}

	// TUNE MINUS VALUE ONCE ROBOT
	/**
	 * For Scoring only
	 *
	 * @param state
	 * @return
	 */
	public Command goToStatePerpendicular(SuperstructureState state) {
		return Commands.sequence(
				pivotCommands.setAngle(state.pivotAngle),
				Commands.waitUntil(isAligned),
				elevatorCommands.setHeight(state.elevatorHeight),
				pivotCommands.setAngle(state.pivotAngle.minus(Degree.of(10))),
				Commands.waitSeconds(0.01), // basically 0.02
				pivotCommands
						.setAngle(state.pivotAngle)
						.alongWith(coralerCommands.setOpenLoop(state.rollers))
						.withTimeout(Units.Seconds.of(0.6)));
	}

	/**
	 * REMEMBER TO TUNE THIS
	 *
	 * @param state
	 * @return
	 */
	public Command goToStateIntake(SuperstructureState state) {
		return goToStateParalell(state);
	}

	public Command setWantedBranch(Branch wantedBranch) {
		return Commands.runOnce(
				() -> {
					this.wantedBranch = wantedBranch;
				});
	}

	public Command setWantedSide(Side wantedSide) {
		return Commands.runOnce(
				() -> {
					this.wantedSide = wantedSide;
				});
	}

	public Command setWantedLevel(Level wantedLevel) {
		return Commands.runOnce(
				() -> {
					this.wantedLevel = wantedLevel;
				});
	}

	public Level getWantedLevel() {
		return wantedLevel;
	}

	public Side getWantedSide() {
		return wantedSide;
	}

	public Branch getWantedBranch() {
		return wantedBranch;
	}
}
