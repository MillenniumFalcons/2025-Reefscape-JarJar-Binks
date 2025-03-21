package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.Util.InverseKinematics;
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
    private final Wrist wrist;
    private final Seagull seagull;
    private final Rollers rollers;

    public final CoralerCommands coralerCommands;
    public final ElevatorCommands elevatorCommands;
    public final PivotCommands pivotCommands;
    public final WristCommands wristCommands;
    public final RollersCommands rollersCommands;

    private BooleanSupplier isAligned;

    private MutDistance elevOffset;

    private MutAngle pivotOffset;

    private Level wantedLevel;

    private boolean hasPeice = true;

    private boolean hasAlgae = false;

    private ScoringPos wantedScoringPos = ScoringPos.NONE;

    private Trigger overridePiece;

    private double currentLimit = 47;
    private double SeagullCurrentLimit = 8;
    private double algaeCurrentLimit = 47;

    private double wristOffset = 0;

    private Map<Level, SuperstructureState> kLevelToScoreMap;

    public enum Side {
        A,
        B,
        C,
        D,
        E,
        F
    }

    public enum Branch {
        ONE(-1),
        TWO(1);

        public final int sign;

        Branch(int sign) {
            this.sign = sign;
        }
    }

    public Superstructure(
            Coraler coraler,
            Elevator elevator,
            Pivot pivot,
            Wrist wrist,
            Rollers rollers,
            Seagull seagull,
            Trigger pieceOverride) {
        this.coraler = coraler;
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;
        this.seagull = seagull;
        this.rollers = rollers;

        this.coralerCommands = new CoralerCommands(this.coraler);
        this.elevatorCommands = new ElevatorCommands(this.elevator);
        this.pivotCommands = new PivotCommands(this.pivot);
        this.rollersCommands = new RollersCommands(rollers, seagull);
        this.wristCommands = new WristCommands(wrist);

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

        this.kLevelToScoreMap =
                Map.of(
                        Level.TROUGH,
                        SuperstructureState.TroughScore,
                        Level.LOW,
                        SuperstructureState.LowScore,
                        Level.MID,
                        SuperstructureState.MidScore,
                        Level.HIGH,
                        SuperstructureState.HighScore);
    }

    public enum Level {
        TROUGH,
        LOW,
        MID,
        HIGH,
        ALGAEHIGH,
        ALGAELOW,
        NONE
    }

    public void setIsAlignedFunction(BooleanSupplier isAligned) {
        this.isAligned = isAligned;
        DriverStation.reportError("IsAligned function has been set, all is well", false);
    }

    public boolean shouldClear() {
        return pivot.getAngle().gt(Radian.of(-0.7))
                && elevator.getHeight().lt(ElevatorConstants.kClearHeight);
    }

    public boolean isAligned() {
        return isAligned.getAsBoolean();
    }

    public ScoringPos getWantedScoringPos() {
        return wantedScoringPos;
    }

    public void logError(String message) {
        Logger.recordOutput("Robot/robotErrors", message + Timer.getFPGATimestamp());
        DriverStation.reportError(message, true);
    }

    public Command goToStateParalell(Supplier<SuperstructureState> state) {

        return Commands.either(
                Commands.parallel(
                        // Commands.run(() -> Logger.recordOutput("monkeyballs",
                        // state.get().elevatorHeight)),
                        elevatorCommands.setHeight(() -> state.get().elevatorHeight),
                        pivotCommands.setAngle(
                                () ->
                                        MathUtil.clamp(
                                                state.get().pivotAngle.in(Radian),
                                                getPivotMinAngle(),
                                                pivot.getMaxAngle().in(Radian))),
                        Commands.either(
                                Commands.none(),
                                wristCommands.setAngle(state.get().wristAngle),
                                () -> state.get().wristAngle.equals(WristConstants.idrc))
                        // Commands.run(() ->
                        // Logger.recordOutput("ballsmonkey",state.get().pivotAngle))

                        ),
                Commands.none(),
                () -> !state.get().equals(SuperstructureState.kInvalidState));
    }

    public Command goToStateParalellNoWrist(Supplier<SuperstructureState> state) {

        return Commands.either(
                Commands.parallel(
                        // Commands.run(() -> Logger.recordOutput("monkeyballs",
                        // state.get().elevatorHeight)),
                        elevatorCommands.setHeight(() -> state.get().elevatorHeight),
                        pivotCommands.setAngle(
                                () ->
                                        MathUtil.clamp(
                                                state.get().pivotAngle.in(Radian),
                                                pivot.getMinAngle().in(Radian),
                                                pivot.getMaxAngle().in(Radian)))
                        // Commands.run(() ->
                        // Logger.recordOutput("ballsmonkey",state.get().pivotAngle))

                        ),
                Commands.none(),
                () -> !state.get().equals(SuperstructureState.kInvalidState));
    }

    /**
     * delayed elevator
     *
     * @param state
     * @return gotostate with a delayed elevator
     */
    public Command goToStatePerpendicular(Supplier<SuperstructureState> state) {

        return Commands.either(
                Commands.parallel(
                        // .run(() -> Logger.recordOutput("monkeyballs",
                        // state.get().elevatorHeight)),
                        Commands.waitSeconds(0.1)
                                .andThen(
                                        elevatorCommands.setHeight(
                                                () -> state.get().elevatorHeight)),
                        pivotCommands.setAngle(
                                () ->
                                        MathUtil.clamp(
                                                state.get().pivotAngle.in(Radian),
                                                getPivotMinAngle(),
                                                pivot.getMaxAngle().in(Radian))),
                        Commands.either(
                                Commands.none(),
                                wristCommands.setAngle(state.get().wristAngle),
                                () -> state.get().wristAngle.equals(WristConstants.idrc))
                        // Commands.run(() ->
                        // Logger.recordOutput("ballsmonkey",state.get().pivotAngle))

                        ),
                Commands.none(),
                () -> !state.get().equals(SuperstructureState.kInvalidState));
    }

    public Command goToStateNoWrist(Supplier<SuperstructureState> state) {
        return Commands.either(
                Commands.parallel(
                        // Commands.run(() -> Logger.recordOutput("monkeyballs",
                        // state.get().elevatorHeight)),
                        Commands.waitSeconds(0.1)
                                .andThen(
                                        elevatorCommands.setHeight(
                                                () -> state.get().elevatorHeight)),
                        (pivotCommands.setAngle(() -> state.get().pivotAngle))
                        // Commands.either(
                        // 		Commands.none(),
                        // 		wristCommands.setAngle(state.get().wristAngle),
                        // 		() -> state.get().wristAngle.equals(WristConstants.idrc))
                        // Commands.run(() ->
                        // Logger.recordOutput("ballsmonkey",state.get().pivotAngle))

                        ),
                Commands.none(),
                () -> !state.get().equals(SuperstructureState.kInvalidState));
    }

    public SuperstructureState getStateScoreAuto() {
        var prelimWantedState =
                kLevelToScoreMap.getOrDefault(getWantedLevel(), SuperstructureState.kInvalidState);

        return prelimWantedState;
    }

    /**
     * delayed elevator
     *
     * @param state
     * @return gotostate with a delayed elevator
     */
    public Command goToStatePerpendicular(
            Supplier<SuperstructureState> state, DoubleSupplier delay) {

        return Commands.either(
                Commands.parallel(
                        // Commands.run(() -> Logger.recordOutput("monkeyballs",
                        // state.get().elevatorHeight)),
                        Commands.waitSeconds(delay.getAsDouble())
                                .andThen(
                                        elevatorCommands.setHeight(
                                                () -> state.get().elevatorHeight)),
                        (pivotCommands.setAngle(() -> state.get().pivotAngle))
                        // Commands.either(
                        // 		Commands.none(),
                        // 		wristCommands.setAngle(state.get().wristAngle),
                        // 		() -> state.get().wristAngle.equals(WristConstants.idrc))
                        // Commands.run(() ->
                        // Logger.recordOutput("ballsmonkey",state.get().pivotAngle))

                        ),
                Commands.none(),
                () -> !state.get().equals(SuperstructureState.kInvalidState));
    }

    public boolean hasPeice() {
        // TODO: implememtation neded
        return hasPeice;
        // for now no logic cause no handoff code
    }

    public Command setPeice() {
        return Commands.runOnce(() -> this.hasPeice = true);
    }

    public Command setNoPeice() {
        return Commands.runOnce(() -> this.hasPeice = false);
    }

    public double getPivotMinAngle() {
        return InverseKinematics.getMinAngle(getCurrentState()).in(Radian);
    }

    public SuperstructureState getCurrentState() {
        return new SuperstructureState(pivot.getAngle(), elevator.getHeight(), wrist.getAngle());
    }

    // algae stuff

    // unimplemented lmao
    public Command autoTakeOffByLevel() {
        return Commands.either(
			takeOffAlgaeLow(), 
			takeOffAlgaeHigh(), 
			() -> getWantedLevel() == Level.ALGAELOW);
    }

    public Command takeOffAlgaeHigh() {
        return Commands.parallel(
                elevatorCommands.setHeight(ElevatorConstants.kHighAlgaeHeight),
                pivotCommands.setAngle(PivotConstants.kAlgaeAngleHigh),
                coralerCommands.intake());
    }

	public Command takeOffAlgaeLow(){
		return Commands.parallel(
                elevatorCommands.setHeight(ElevatorConstants.kStowHeight),
                pivotCommands.setAngle(PivotConstants.kAlgaeAngleLow),
                coralerCommands.intake());
	}

    public Command scoreAlgaeBarge() {
        return Commands.parallel(
			elevatorCommands.setHeight(ElevatorConstants.kMaxHeight),
			
			Commands.sequence(
				Commands.waitSeconds(0.3),
				pivotCommands.setAngle(PivotConstants.kStowAngle)
			),
			Commands.sequence(
				Commands.waitSeconds(0.35),
				coralerCommands.setOpenLoop(-1.0).withTimeout(0.5)
			)

		);
    }

    public Command stowAlgaeBarge() {
        return Commands.parallel(
                stow(),
                Commands.sequence(
                        Commands.waitSeconds(0.025),
                        coralerCommands.setOpenLoop(-1.0).withTimeout(0.5)));
    }

    public Command setHasAlgae() {
        return Commands.runOnce(() -> this.hasAlgae = true);
    }

    public Command setNoAlgae() {
        return Commands.runOnce(() -> this.hasAlgae = false);
    }

    public boolean getAlgae() {
        return hasAlgae;
    }

    // end of algae stuff

    public boolean shouldClearGoingUp() {
        return pivot.angleWithin(
                        PivotConstants.kMinAngle.minus(Degree.of((5))).in(Radian),
                        PivotConstants.kClearAngle.in(Radian))
                && elevator.getHeight().lt(ElevatorConstants.kClearHeight);
    }

    public Command intake() {
        return Commands.parallel(
                goToStateParalell(() -> SuperstructureState.Intake),
                rollersCommands.setOpenLoop(0.25));
    }

    public Command transfer() {
        return Commands.parallel(
                goToStateParalell(() -> SuperstructureState.Transfer),
                rollersCommands.setOpenLoop(0.6, -0.1));
    }

    public Command handoff() {
        return Commands.parallel(
                rollersCommands.kill(),
                goToStateParalell(() -> SuperstructureState.Handoff),
                coralerCommands.intake());
    }

    //

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
                                pivotCommands.setAngle(PivotConstants.kStowAngle),
                                wristCommands.stow()),
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

    public Command clearElevatorGoingUpNoDownNew(Angle finalPivotAngle) {
        return Commands.either(
                Commands.sequence(
                        Commands.deadline(
                                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                                wristCommands.stow()),
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

    public Command scoreL4() {
        return Commands.sequence(
                clearElevatorGoingUpNoDown(PivotConstants.kStowAngleUp),
                elevatorCommands.setHeight(ElevatorConstants.kLevel4Height),
                pivotCommands.setAngle(PivotConstants.kL4Prep));
    }

    public Command scoreL3() {
        return Commands.sequence(
                clearElevatorGoingUpNoDown(PivotConstants.kStowAngleUp),
                elevatorCommands.setHeight(ElevatorConstants.kLowScoreHeight),
                pivotCommands.setAngle(PivotConstants.kL3prep));
    }

    public Command scoreL2() {
        return Commands.sequence(
                clearElevatorGoingUpNoDown(PivotConstants.kL2Prep),
                elevatorCommands.setHeight(ElevatorConstants.kLowScoreHeight),
                pivotCommands.setAngle(PivotConstants.kLevel2Angle));
    }

    public Command poopCoral() {
        return Commands.waitSeconds(0.15)
                .andThen(coralerCommands.setOpenLoop(-0.3).withTimeout(0.1))
                .withTimeout(0.3);
    }

    public Command stowFromL4() {
        // put it on the reef
        return Commands.sequence(pivotCommands.setAngle(PivotConstants.kLevel4Angle), poopCoral());
    }

    public Command stowFromL3() {
        return Commands.parallel(pivotCommands.setAngle(PivotConstants.kLevel3Angle), poopCoral());
    }

    public MutDistance getElevOffset() {
        return elevOffset;
    }

    public MutAngle getPivotOffset() {
        return pivotOffset;
    }

    public boolean intakeCurrent() {
        return rollers.getMasterCurrent() > currentLimit;
    }

    public boolean coralerAlgaeCurrent() {
        return coraler.getMasterCurrent() > algaeCurrentLimit;
    }

    public Command stowFromIntake() {
        return Commands.sequence(
                wristCommands
                        .setAngle(WristConstants.kHandoffAngle)
                        .alongWith(
                                pivotCommands.setAngle(PivotConstants.kHandoffAngle),
                                rollersCommands.setOpenLoop(-0.17).withTimeout(0.1)),
                Commands.waitSeconds(0.3),
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
                wristCommands.setAngle(WristConstants.kStowWithPiece),
                elevatorCommands.setHeight(ElevatorConstants.kStowHeight),
                rollersCommands.kill(),
                coralerCommands.kill());
    }

    // public Command intake(){
    // return Commands.parallel(
    // wristcommands.setAngle(kHandoffAngle),
    // pivotCommands.setAngle(PivotConstants.kIntakeAngle),
    // rollersCommands.intake()
    // );
    // }

    public Command autoScoreByLevel() {

        return goToStatePerpendicular(this::getStateScoreAuto, () -> getDelayByLevel(wantedLevel));
    }

    public double getDelayByLevel(Level level) {
        switch (level) {
            case HIGH:
                return 0;

            default:
                return 0.3;
        }
    }

    public Command stow() {
        return Commands.either(
                Commands.sequence(
                        goToStatePerpendicular(() -> SuperstructureState.ToStow),
						Commands.waitSeconds(0.01),
                        goToStateParalell(() -> SuperstructureState.Stow)),
                goToStatePerpendicular(() -> SuperstructureState.Stow, () -> 0.3),
                this::shouldClear);
    }

    public Command sequencialStow() {
        return Commands.sequence(
                goToStateNoWrist(() -> SuperstructureState.ToStow),
                goToStateParalellNoWrist(() -> SuperstructureState.Stow));
    }

    public Command paralellStow() {
        return Commands.either(
                Commands.sequence(
                        goToStateNoWrist(() -> SuperstructureState.ToStow),
                        goToStateParalellNoWrist(() -> SuperstructureState.Stow)),
                goToStatePerpendicular(() -> SuperstructureState.Stow, () -> 0.3),
                this::shouldClear);
    }

    public Command killAll() {
        return Commands.parallel(rollersCommands.kill(), coralerCommands.kill());
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

    public boolean isIntaking() {
        return wrist.angleReached(WristConstants.kIntakeAngle.in(Degree), 5);
    }

    public boolean seagullCurrent() {
        return rollersCommands.seagullCurrentGreater(SeagullCurrentLimit);
    }
}
