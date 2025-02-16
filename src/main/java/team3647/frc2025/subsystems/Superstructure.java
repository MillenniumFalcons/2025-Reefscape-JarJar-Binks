package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;

import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.commands.CoralerCommands;
import team3647.frc2025.commands.ElevatorCommands;
import team3647.frc2025.commands.PivotCommands;
import team3647.frc2025.commands.WristCommands;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants.ScoringPos;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.WristConstants;

public class Superstructure {

    private final Coraler coraler;
    private final Elevator elevator;
    private final Pivot pivot;

    public final CoralerCommands coralerCommands;
    public final ElevatorCommands elevatorCommands;
    public final PivotCommands pivotCommands;
	public final WristCommands wristCommands;

    private BooleanSupplier isAligned;

    private Level wantedLevel;

	private SuperstructureState wantedSuperstructureState = SuperstructureState.kInvalidState;

    private ScoringPos wantedScoringPos = ScoringPos.NONE;

    private Side wantedSide;

    private Branch wantedBranch;

	private Map<Level, Command> kLevelToCmdMap;

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

    public Superstructure(Coraler coraler, Elevator elevator, Pivot pivot, Wrist wrist) {
        this.coraler = coraler;
        this.elevator = elevator;
        this.pivot = pivot;

        this.coralerCommands = new CoralerCommands(this.coraler);
        this.elevatorCommands = new ElevatorCommands(this.elevator);
        this.pivotCommands = new PivotCommands(this.pivot);

        this.wantedLevel = Level.NONE;
        this.isAligned =
                () -> {
                    DriverStation.reportError(
                            "Haven't added isAligned Function yet, check for added message. "
                                    + Timer.getFPGATimestamp(),
                            false);
                    return false;
                };
		this.kLevelToCmdMap = Map.of(Level.TROUGH, prepL1(),
									Level.LOW, prepL2(),
									Level.MID, prepL3(),
									Level.HIGH, prepL4());

		this.wristCommands = new WristCommands(wrist);
		

    }

    public enum Level {
        TROUGH,
        LOW,
        MID,
        HIGH,
        INTAKE,
        NONE
    }

    public void setIsAlignedFunction(BooleanSupplier isAligned) {
        this.isAligned = isAligned;
        DriverStation.reportError("IsAligned function has been set, all is well", false);
    }

    public SuperstructureState getWantedStateByLevel(Level level) {
        switch (level) {
            case TROUGH:
                return new SuperstructureState(
                        PivotConstants.kLevel1Angle, ElevatorConstants.kLevel1Height);
            case LOW:
                return new SuperstructureState(
                        PivotConstants.kLevel2Angle, ElevatorConstants.kLevel2Height);
            case MID:
                return new SuperstructureState(
                        PivotConstants.kLevel3Angle, ElevatorConstants.kLevel3Height);
            case HIGH:
                return new SuperstructureState(
                        PivotConstants.kLevel4Angle, ElevatorConstants.kLevel4Height);
            case INTAKE:
                return new SuperstructureState(
                        PivotConstants.kIntakeAngle, ElevatorConstants.kIntakeHeight);
            default:
                return new SuperstructureState(
                        PivotConstants.kStowAngle, ElevatorConstants.kStowHeight);
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
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
						Logger.recordOutput("robot/robotErrors", "Cannot set Wanted Scoring pos, no branch provided" + Timer.getFPGATimestamp());
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
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
								Logger.recordOutput("robot/robotErrors", "Cannot set Wanted Scoring pos, no branch provided" + Timer.getFPGATimestamp());
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
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
						Logger.recordOutput("robot/robotErrors", "Cannot set Wanted Scoring pos, no branch provided" + Timer.getFPGATimestamp());
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
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
						Logger.recordOutput("robot/robotErrors", "Cannot set Wanted Scoring pos, no branch provided" + Timer.getFPGATimestamp());
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
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
						Logger.recordOutput("robot/robotErrors", "Cannot set Wanted Scoring pos, no branch provided" + Timer.getFPGATimestamp());
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
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
						Logger.recordOutput("robot/robotErrors", "Cannot set Wanted Scoring pos, no branch provided" + Timer.getFPGATimestamp());
                        break;
                }
            default:
                DriverStation.reportError("Cannot set Wanted scoring pos, no SIDE provided", false);
				Logger.recordOutput("robot/robotErrors", "Cannot set Wanted Scoring pos, no SIDE provided");
                break;
        }
    }

    public ScoringPos getWantedScoringPos() {
        return wantedScoringPos;
    }

	public void logError(String message){
		Logger.recordOutput("robot/robotErrors", message + Timer.getFPGATimestamp());
		DriverStation.reportError(message, false);
	}

    /**
     * NOTE: WILL CHANGE BASED ON INTAKE GEOMETRY
     *
     * @return
     */
	@Deprecated
    public Command goToStateParalell(SuperstructureState state) {
	
        return Commands.parallel(
                elevatorCommands.setHeight(state.elevatorHeight),
                pivotCommands.setAngle(state.pivotAngle));
    }

   
    /**
     * For Scoring only
     *
     * @param state
     * @return
     */
	@Deprecated
    public Command goToStatePerpendicular(SuperstructureState state) {
		if (state.equals(SuperstructureState.kInvalidState)) {
			logError("Invalid state given to gotoState function!!");
			return Commands.none();
		}
        return Commands.sequence(
				clearElevatorGoingUp(),
                pivotCommands.setAngle(state.pivotAngle),
                // Commands.waitUntil(isAligned),
                elevatorCommands.setHeight(state.elevatorHeight)
		);
    }

	public Command prepL1(){
		return Commands.sequence(
			clearElevatorGoingUpNoDown(),
			pivotCommands.setAngle(PivotConstants.kLevel1Angle),
			elevatorCommands.setHeight(ElevatorConstants.kLevel1Height)
		);
	}

	public boolean shouldClearGoingUp(){
		return pivot.angleWithin(PivotConstants.kMinAngle.minus(Degree.of((5))).in(Radian), PivotConstants.kClearAngle.in(Radian)) && elevator.getHeight().lt(ElevatorConstants.kClearHeight);
	}
//
	public Command clearElevatorGoingUp(){
		return Commands.either(
			Commands.sequence(
				Commands.deadline(
					elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
					pivotCommands.setAngle(PivotConstants.kStartingAngle)),
				pivotCommands.setAngle(PivotConstants.kClearAngle),
				elevatorCommands.setHeight(ElevatorConstants.kStowHeight)
			).alongWith(Commands.run(() -> Logger.recordOutput("toclear?", true))), 
			Commands.none().alongWith(Commands.runOnce(() -> Logger.recordOutput("toClear?", false))), 
			() -> pivot.angleWithin(PivotConstants.kStartingAngle.in(Radian), PivotConstants.kClearAngle.in(Radian)) && elevator.getHeight().lt(ElevatorConstants.kClearHeight));
	}

	public Command clearElevatorGoingDown(){
		return Commands.either(
			Commands.sequence(
				elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
				pivotCommands.setAngle(PivotConstants.kStartingAngle),
				elevatorCommands.setHeight(ElevatorConstants.kStowHeight)
			), 
			Commands.none(), 
			() -> pivot.angleWithin(PivotConstants.kStartingAngle.in(Radian), PivotConstants.kClearAngle.in(Radian)) && elevator.getHeight().lt(ElevatorConstants.kClearHeight));
	}

	@Deprecated
	public Command clearElevatorGoingUpNoDown(){
		return Commands.either(
			Commands.sequence(
				Commands.deadline(
					elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
					pivotCommands.setAngle(PivotConstants.kStartingAngle)),
				pivotCommands.setAngle(PivotConstants.kClearAngle)
			), 
			Commands.none(), 
			() -> pivot.angleWithin(PivotConstants.kStartingAngle.in(Radian), PivotConstants.kClearAngle.in(Radian)) && elevator.getHeight().lt(ElevatorConstants.kClearHeight));
	}

	public Command prepL2(){
		return Commands.sequence(
			clearElevatorGoingUpNoDown(),
			elevatorCommands.setHeight(ElevatorConstants.kLevel2Height),
			pivotCommands.setAngle(PivotConstants.kLevel2Angle)
		);
	}



	public Command prepIntake(){
		return Commands.sequence(
			clearElevatorGoingDown(),
			wristCommands.goToIntake()	
		);
	}

	// public Command handoff(){
	// 	return Commands.sequence(
	// 		wristCommands.setAngle(WristConstants.)	
	// 	);
	// }

	// public Command intake(){
	// 	return Commands.parallel(
	// 		wristcommands.setAngle(kHandoffAngle),
	// 		pivotCommands.setAngle(PivotConstants.kIntakeAngle),
	// 		rollersCommands.intake()
	// 	);
	// }

	public Command stow(Level level){
		return Commands.sequence(
			Commands.parallel(
				elevatorCommands.holdPositionAtCall(),
				pivotCommands.holdPositionAtCall(),
				coralerCommands.spitOut()
			),
			pivotCommands.setAngle(PivotConstants.kStowAngle),
			elevatorCommands.setHeight(ElevatorConstants.kStowHeight)
		);
	}

	public Command prepL3(){
		return Commands.sequence(
			clearElevatorGoingUpNoDown(),
			elevatorCommands.setHeight(ElevatorConstants.kLevel3Height),
			pivotCommands.setAngle(PivotConstants.kLevel3Angle)
		);
	}

	public Command prepL4(){
		return Commands.sequence(
			clearElevatorGoingUpNoDown(),
			elevatorCommands.setHeight(ElevatorConstants.kLevel4Height),
			pivotCommands.setAngle(PivotConstants.kLevel4Angle)
		);
	}

	// public Command score(SuperstructureState state){
	// 	return Commands.sequence(
	// 		goToStatePerpendicular(state),
	// 		coralerCommands.spitOut()
	// 	);
	// }

	// public Command scoreAuto(){
	// 	return score(wantedSuperstructureState);
	// }

	public Command autoPrepByWantedLevel(){

		return Commands.select(kLevelToCmdMap, this::getWantedLevel);
	}

	public Command autoPrepBylevel(Level level){
		
		
		if (level.equals(Level.TROUGH)) {
			
			return prepL1();
		}
		if (level.equals(Level.LOW)) {
			DriverStation.reportError("BALLLLLLA", false);
			return prepL2();
		}
		if (level == Level.MID) {
			return prepL3();
		}
		if (level == Level.HIGH) {
			return prepL4();
		}
		return Commands.none().alongWith(Commands.runOnce(() -> DriverStation.reportError("default case on autoprep command!!!" + wantedLevel.toString(), false)));
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


	//called when score button pressed on mainController
	public Command setWantedSuperstructureState(){
		return Commands.runOnce(
			() -> setSSStateAuto());
	}

	private void setSSStateAuto(){
		switch (wantedLevel) {
			case TROUGH:
				wantedSuperstructureState = SuperstructureState.troughScore;
				break;
			case LOW:
				wantedSuperstructureState = SuperstructureState.lowScore;
			case MID:
				wantedSuperstructureState = SuperstructureState.midScore;
			case HIGH:
				wantedSuperstructureState = SuperstructureState.highScore;
		
			default:
				break;
		}
	}
}
