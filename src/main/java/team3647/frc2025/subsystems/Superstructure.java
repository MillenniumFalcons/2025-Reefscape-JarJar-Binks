package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.security.PublicKey;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.commands.CoralerCommands;
import team3647.frc2025.commands.ElevatorCommands;
import team3647.frc2025.commands.PivotCommands;
import team3647.frc2025.commands.RollersCommands;
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
    public final RollersCommands rollersCommands;

    private BooleanSupplier isAligned;

    private MutDistance elevOffset;

    private MutAngle pivotOffset;

    private Level wantedLevel;

    private SuperstructureState wantedSuperstructureState = SuperstructureState.kInvalidState;

    private ScoringPos wantedScoringPos = ScoringPos.NONE;

    private Trigger overridePiece;

    private double currentLimit = 35;

    private double wristOffset = 0;

    private Map<Level, Command> kLevelToPrepMap;

    private Map<Level, Command> kLevelToScoreMap;

    private Map<Level, Command> stowFromShotMap;

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
            Coraler coraler,
            Elevator elevator,
            Pivot pivot,
            Wrist wrist,
            Rollers rollers,
            Trigger pieceOverride) {
        this.coraler = coraler;
        this.elevator = elevator;
        this.pivot = pivot;

        this.coralerCommands = new CoralerCommands(this.coraler);
        this.elevatorCommands = new ElevatorCommands(this.elevator);
        this.pivotCommands = new PivotCommands(this.pivot);
        this.rollersCommands = new RollersCommands(rollers);

        this.elevOffset = Meter.of(0).mutableCopy();
        this.pivotOffset = Radian.of(0).mutableCopy();

        this.overridePiece = pieceOverride;

        this.wantedLevel = Level.NONE;
        this.isAligned =
                () -> {
                    DriverStation.reportError(
                            "Haven't added isAligned Function yet, check for added message. "
                                    + Timer.getFPGATimestamp(),
                            false);
                    return false;
                };
        this.kLevelToPrepMap =
                Map.of(
                        Level.TROUGH,
                        prepL1(),
                        Level.LOW,
                        prepL2(),
                        Level.MID,
                        prepL3(),
                        Level.HIGH,
                        prepL4());

        this.kLevelToScoreMap =
                Map.of(
                        Level.TROUGH,
                        scoreL1(),
                        Level.LOW,
                        scoreL2(),
                        Level.MID,
                        scoreL3(),
                        Level.HIGH,
                        scoreL4());

        this.stowFromShotMap =
                Map.of(
                        Level.TROUGH,
                        poopCoral(),
                        Level.LOW,
                        poopCoral(),
                        Level.MID,
                        stowFromL3(),
                        Level.HIGH,
                        stowFromL4());

        this.wristCommands = new WristCommands(wrist);
    }

    public enum Level {
        TROUGH,
        LOW,
        MID,
        HIGH,
        INTAKE,
		ALGAEHIGH,
		ALGAELOW,
        NONE
    }

	

    public void setIsAlignedFunction(BooleanSupplier isAligned) {
        this.isAligned = isAligned;
        DriverStation.reportError("IsAligned function has been set, all is well", false);
    }

    public boolean isAligned() {
        return isAligned.getAsBoolean();
    }

    public ScoringPos getWantedScoringPos() {
        return wantedScoringPos;
    }



    public void logError(String message) {
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
                elevatorCommands.setHeight(state.elevatorHeight));
    }

	public Command prepAlgae(){
		return Commands.sequence(
			clearElevatorGoingUp(PivotConstants.kStowAngleUp),
			pivotCommands.setAngle(PivotConstants.kStowAngleUp)
		);
	}

	public Command takeOffAlgaeHigh(){
		return Commands.sequence(
			elevatorCommands.setHeight(ElevatorConstants.kHighAlgaeHeight),
			pivotCommands.setAngle(PivotConstants.kAlgaeAngleHigh)
			
		);
	}

	public Command takeOffAlgaeLow(){
		return Commands.sequence(
			elevatorCommands.setHeight(ElevatorConstants.kLowAlgaeHeight),
			pivotCommands.setAngle(PivotConstants.kAlgaeAngleLow)
			
		);
	}
	

    public Command prepL1() {
        return Commands.sequence(
                clearElevatorGoingUp(PivotConstants.KL1Prep),
                Commands.parallel(
                        elevatorCommands.setHeight(
                                ElevatorConstants.kLevel1Height.plus(elevOffset)),
                        pivotCommands.setAngle(PivotConstants.KL1Prep.plus(pivotOffset))));
    }

    public boolean shouldClearGoingUp() {
        return pivot.angleWithin(
                        PivotConstants.kMinAngle.minus(Degree.of((5))).in(Radian),
                        PivotConstants.kClearAngle.in(Radian))
                && elevator.getHeight().lt(ElevatorConstants.kClearHeight);
    }

    public Command autoStowFromShot() {
        return Commands.select(stowFromShotMap, this::getWantedLevel);
    }

    //
    public Command clearElevatorGoingUp() {
        return Commands.either(
                Commands.sequence(
                                Commands.deadline(
                                        elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                                        pivotCommands.setAngle(PivotConstants.kStartingAngle)),
                                pivotCommands.setAngle(PivotConstants.kClearAngle),
                                elevatorCommands.setHeight(ElevatorConstants.kStowHeight))
                        .alongWith(Commands.run(() -> Logger.recordOutput("toclear?", true))),
                Commands.none()
                        .alongWith(Commands.runOnce(() -> Logger.recordOutput("toClear?", false))),
                () ->
                        pivot.angleWithin(
                                        PivotConstants.kStartingAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian))
                                && elevator.getHeight().lt(ElevatorConstants.kClearHeight));
    }

    public Command clearElevatorGoingUp(Angle finalAngle) {
        return Commands.either(
                Commands.sequence(
                        Commands.deadline(
                                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                                pivotCommands.setAngle(PivotConstants.kStartingAngle)),
                        pivotCommands
                                .setAngle(
                                        MathUtil.clamp(
                                                finalAngle.in(Radian),
                                                PivotConstants.kClearAngle.in(Radian),
                                                PivotConstants.kMaxAngle.in(Radian)))
                                .withTimeout(0.5),
                        elevatorCommands.setHeight(ElevatorConstants.kStowHeight)),
                Commands.none(),
                () ->
                        pivot.angleWithin(
                                        PivotConstants.kStartingAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian))
                                && elevator.getHeight().lt(ElevatorConstants.kClearHeight));
    }

    public Command clearElevatorGoingDown() {
        return Commands.either(
                Commands.sequence(
                        elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                        pivotCommands.setAngle(PivotConstants.kStartingAngle),
                        elevatorCommands.setHeight(ElevatorConstants.kStowHeight)),
                Commands.none(),
                () ->
                        pivot.angleWithin(
                                        PivotConstants.kClearAngle.in(Radian),
                                        PivotConstants.kMaxAngle.in(Radian))
                                || elevator.getHeight().gt(ElevatorConstants.kClearHeight));
    }

    public Command stowElevAndPivot() {
        return Commands.sequence(
                clearElevatorGoingDown(),
                Commands.parallel(
                        pivotCommands.setAngle(PivotConstants.kStowAngle),
                        elevatorCommands.setHeight(ElevatorConstants.kStowHeight)))
						.alongWith(wristCommands.stow());
    }

    public Command clearElevatorGoingDown(Angle finalPivotAngle) {
        return Commands.either(
                Commands.sequence(
                        elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                        pivotCommands.setAngle(
                                MathUtil.clamp(
                                        finalPivotAngle.in(Radian),
                                        PivotConstants.kMinAngle.in(Radian),
                                        1.4)),
                        elevatorCommands.setHeight(ElevatorConstants.kStowHeight)),
                Commands.none(),
                () ->
                        pivot.angleWithin(
                                        PivotConstants.kClearAngle.in(Radian),
                                        PivotConstants.kMaxAngle.in(Radian))
                                || elevator.getHeight().gt(ElevatorConstants.kClearHeight));
    }

    public Command clearElevatorGoingDown(Angle finalPivotAngle, Distance finalElevHeight) {
        return Commands.either(
                Commands.sequence(
                        elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                        pivotCommands.setAngle(
                                MathUtil.clamp(
                                        finalPivotAngle.in(Radian),
                                        PivotConstants.kMinAngle.in(Radian),
                                        1.4)),
                        elevatorCommands.setHeight(
                                MathUtil.clamp(
                                        finalElevHeight.in(Meter),
                                        ElevatorConstants.kStowHeight.in(Meter),
                                        ElevatorConstants.kMaxHeight.in(Meter)))),
                Commands.none(),
                () ->
                        pivot.angleWithin(
                                        PivotConstants.kClearAngle.in(Radian),
                                        PivotConstants.kMaxAngle.in(Radian))
                                || elevator.getHeight().gt(ElevatorConstants.kClearHeight));
    }

    public Command stowc() {
        return Commands.sequence(
                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                pivotCommands.setAngle(PivotConstants.kStartingAngle),
                elevatorCommands.setHeight(ElevatorConstants.kStowHeight));
    }

    public Command clearElevatorGoingUpNoDown() {
        return Commands.either(
                Commands.sequence(
                        Commands.deadline(
                                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                                pivotCommands.setAngle(PivotConstants.kStartingAngle)),
                        pivotCommands.setAngle(PivotConstants.kClearAngle)),
                Commands.none(),
                () ->
                        pivot.angleWithin(
                                        PivotConstants.kStartingAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian))
                                && elevator.getHeight().lt(ElevatorConstants.kClearHeight));
    }

    public Command clearElevatorGoingUpNoDown(Angle finalPivotAngle) {
        return Commands.either(
                Commands.sequence(
                        Commands.deadline(
                                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                                pivotCommands.setAngle(PivotConstants.kStartingAngle)),
                        pivotCommands.setAngle(
                                MathUtil.clamp(
                                        finalPivotAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian),
                                        PivotConstants.kMaxAngle.in(Radian)))),
                Commands.none(),
                () ->
                        pivot.angleWithin(
                                        PivotConstants.kStartingAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian))
                                && elevator.getHeight().lt(ElevatorConstants.kClearHeight));
    }

    public Command prepL2() {
        return Commands.sequence(
                clearElevatorGoingUp(PivotConstants.kL2Prep),
                Commands.parallel(
                        elevatorCommands.setHeight(
                                ElevatorConstants.kLevel2Height.plus(elevOffset)),
                        pivotCommands.setAngle(PivotConstants.kL2Prep.plus(pivotOffset))));
    }

    public Command scoreL4() {
        return Commands.sequence(
                clearElevatorGoingUpNoDown(PivotConstants.kStowAngleUp),
                elevatorCommands.setHeight(ElevatorConstants.kLevel4Height),
                pivotCommands.setAngle(PivotConstants.kLevel4Angle.plus(Degree.of(10))));
    }

    public Command scoreL3() {
        return Commands.sequence(
                clearElevatorGoingUpNoDown(PivotConstants.kStowAngleUp),
                elevatorCommands.setHeight(ElevatorConstants.kLevel3Height),
                pivotCommands.setAngle(PivotConstants.kLevel3Angle));
    }

    public Command scoreL2() {
        return Commands.sequence(
                clearElevatorGoingUpNoDown(PivotConstants.kL2Prep),
                elevatorCommands.setHeight(ElevatorConstants.kLevel2Height),
                pivotCommands.setAngle(PivotConstants.kLevel2Angle));
    }

    public Command scoreL1() {
        return Commands.sequence(
                clearElevatorGoingUpNoDown(PivotConstants.KL1Prep),
                elevatorCommands.setHeight(ElevatorConstants.kLevel1Height),
                pivotCommands.setAngle(PivotConstants.kLevel1Angle));
    }

    public Command poopCoral() {
        return coralerCommands.setOpenLoop(-0.3).withTimeout(0.1);
    }

    public Command stowFromL4() {
        return Commands.sequence(poopCoral(), pivotCommands.setAngle(-0.28));
    }

    public Command stowFromL3() {
        return Commands.parallel(poopCoral(), pivotCommands.setAngle(Degree.of(-10)));
    }

    public Command stowAll() {
        return Commands.either(
                Commands.parallel(
                        elevatorCommands.setHeight(ElevatorConstants.kStowHeight),
                        pivotCommands.setAngle(PivotConstants.kStowAngleUp),
                        coralerCommands.kill(),
                        rollersCommands.kill()),
                Commands.sequence(
                        clearElevatorGoingDown()
                                .alongWith(coralerCommands.kill(), rollersCommands.kill()),
                        wristCommands.setAngle(WristConstants.kStowAngle)),
                () ->
                        !(pivot.angleWithin(
                                        PivotConstants.kStartingAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian))
                                && elevator.getHeight().lt(ElevatorConstants.kClearHeight)));
    }

    public MutDistance getElevOffset() {
        return elevOffset;
    }

    public MutAngle getPivotOffset() {
        return pivotOffset;
    }

    public Command stowHigh() {
        return Commands.sequence(
                clearElevatorGoingUp(), pivotCommands.setAngle(PivotConstants.kStowAngleUp));
        // }

        // public Command letGoFromL4(){
        // 	return pivotCommands.setAngle(PivotConstants.kLevel4Angle.minus(Radian.of(0.436)));
        // }
        // public Command letGoFromL3(){
        // 	return pivotCommands.setAngle(PivotConstants.kLevel3Angle.minus(Radian.of(0.2967)));
        // }
        // public Command letGoFromL2(){
        // 	return pivotCommands.setAngle(PivotConstants.kLevel2Angle.minus(Radian.of(0.22)));
        // }
        // public Command letGoFromL1(){
        // 	return coralerCommands.setOpenLoop(-0.1).withTimeout(1);
    }

    public Command prepIntake() {
        return Commands.sequence(
                        wristCommands.goToIntake().withTimeout(1),
                        clearElevatorGoingDown(
                                PivotConstants.kHandoffAngle, ElevatorConstants.kHandoffHeight),
                        Commands.sequence(
                                elevatorCommands.setHeight(ElevatorConstants.kHandoffHeight),
                                (pivotCommands.setAngle(PivotConstants.kHandoffAngle))))
                .alongWith(rollersCommands.setOpenLoop(-0.3));
    }

    public Trigger intakeCurrent() {
        return new Trigger(() -> rollersCommands.currentGreater(currentLimit)).debounce(0.00);
    }

    public Command stowFromIntake() {
        return Commands.sequence(
                wristCommands
                        .setAngle(WristConstants.kHandoffAngle)
                        .alongWith(
                                pivotCommands.setAngle(PivotConstants.kHandoffAngle),
                                rollersCommands.setOpenLoop(-0.17).withTimeout(0.1)),
                Commands.waitSeconds(0.2),
                elevatorCommands
                        .setHeight(ElevatorConstants.kStartingHeight)
                        .alongWith(
                                coralerCommands
                                        .setOpenLoop(0.5)
                                        .until(coralerCommands.current().or(overridePiece))),
                wristCommands.setAngle(Degree.of(50.14)),
                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                pivotCommands.setAngle(PivotConstants.kStowAngleUp),
                elevatorCommands.setHeight(ElevatorConstants.kStowHeight),
                wristCommands.setAngle(WristConstants.kStowAngle));
    }

    public Angle getPivotAngleByLevel() {
        switch (wantedLevel) {
            case TROUGH:
                return PivotConstants.KL1Prep;
            case LOW:
                return PivotConstants.kL2Prep;
            case MID:
                return PivotConstants.kStowAngleUp;
            case HIGH:
                return PivotConstants.kStowAngleUp;
            default:
                return PivotConstants.kStowAngleUp;
        }
    }

    public Command stowIntake() {
        return Commands.parallel(
                Commands.either(
                        pivotCommands.setAngle(PivotConstants.kStowAngle),
                        pivotCommands.setAngle(PivotConstants.kStowAngleUp),
                        pivot::needToClearElevator),
                elevatorCommands.setHeight(ElevatorConstants.kStowHeight),
                wristCommands.setAngle(WristConstants.kStowWithPiece),
                rollersCommands.kill(),
                coralerCommands.kill());
    }

    public Command intakeTemp() {
        return Commands.sequence(
                Commands.parallel(
                        prepIntake(),
                        rollersCommands
                                .setOpenLoop(-0.5)
                                .until(() -> rollersCommands.currentGreater(45))),
                wristCommands.setAngle(WristConstants.kStowAngle).withTimeout(1));
    }

    public Command handoff() {
        return Commands.sequence(
                wristCommands.setAngle(WristConstants.kHandoffAngle).withTimeout(1),
                pivotCommands.setAngle(PivotConstants.kHandoffAngle),
                coralerCommands.setOpenLoop(0.5).withTimeout(2),
                pivotCommands.setAngle(PivotConstants.kStowAngle),
                wristCommands.setAngle(WristConstants.kStowAngle));
    }

    // public Command intake(){
    // 	return Commands.parallel(
    // 		wristcommands.setAngle(kHandoffAngle),
    // 		pivotCommands.setAngle(PivotConstants.kIntakeAngle),
    // 		rollersCommands.intake()
    // 	);
    // }

    public Command prepL3() {
        return Commands.sequence(
                clearElevatorGoingUp(PivotConstants.kStowAngleUp),
                Commands.parallel(
                        elevatorCommands.setHeight(
                                ElevatorConstants.kLevel3Height.plus(elevOffset)),
                        pivotCommands.setAngle(PivotConstants.kStowAngleUp.plus(pivotOffset))));
    }

    public Command prepL4() {
        return Commands.sequence(
                clearElevatorGoingUp(PivotConstants.kStowAngleUp),
                Commands.parallel(
                        elevatorCommands.setHeight(
                                ElevatorConstants.kLevel4Height.plus(elevOffset)),
                        pivotCommands.setAngle(PivotConstants.kStowAngleUp.plus(pivotOffset))));
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

    public Command autoPrepByWantedLevel() {

        return Commands.select(kLevelToPrepMap, this::getWantedLevel);
    }

    public Command autoScoreByLevel() {
        return Commands.select(kLevelToScoreMap, this::getWantedLevel);
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

    // called when score button pressed on mainController
    public Command setWantedSuperstructureState() {
        return Commands.runOnce(() -> setSSStateAuto());
    }

    private void setSSStateAuto() {
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

    public Command pivotOffsetUp() {
        return Commands.runOnce(() -> pivotOffset.mut_plus(Degree.of(1)));
    }

    public Command pivotOffsetDown() {
        return Commands.runOnce(() -> pivotOffset.mut_minus(Degree.of(1)));
    }

    public Command elevOffsetUp() {
        return Commands.runOnce(() -> elevOffset.mut_plus(Inch.of(1)));
    }

    public Command elevOffsetDown() {
        return Commands.runOnce(() -> elevOffset.mut_minus(Inch.of(1)));
    }
}
