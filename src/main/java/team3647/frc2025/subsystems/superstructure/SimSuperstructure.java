package team3647.frc2025.subsystems.superstructure;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.subsystems.Elevator.Elevator;
import team3647.frc2025.subsystems.Coraler;
import team3647.frc2025.subsystems.Rollers;
import team3647.frc2025.subsystems.Seagull;
import team3647.frc2025.subsystems.pivot.Pivot;
import team3647.frc2025.subsystems.wrist.Wrist;

public class SimSuperstructure extends Superstructure {

    public IntakeSimulation intakeSim;
    public final Pose3d kCoralRelativePose =
            new Pose3d(
                    Units.inchesToMeters(7.295), 0, Units.inchesToMeters(8.196), new Rotation3d());
    public final Pose3d kManipZero =
            new Pose3d(
                    Units.inchesToMeters(24.315),
                    0,
                    Units.inchesToMeters(0.5),
                    new Rotation3d(0, Math.PI / 2, 0));
        public final Pose3d kPreloadManipZero = new Pose3d(
                Units.inchesToMeters(24.315),
                0,
                Units.inchesToMeters(-4.5),
                new Rotation3d(0, Math.PI / 2, 0));
    public Pose3d[] coralPoseLogged;
    private final AbstractDriveTrainSimulation dtSim;

    public SimSuperstructure(
            Coraler coraler,
            Elevator elevator,
            Pivot pivot,
            Wrist wrist,
            Rollers rollers,
            Seagull seagull,
            Trigger pieceOverride,
            Supplier<Pose2d> robotPose,
            AbstractDriveTrainSimulation dtSim) {
        super(coraler, elevator, pivot, wrist, rollers, seagull, pieceOverride, robotPose);
        this.dtSim = dtSim;

        this.intakeSim =
                IntakeSimulation.OverTheBumperIntake(
                        "Coral",
                        dtSim,
                        Meters.of(0.6604),
                        Meters.of(0.2),
                        IntakeSimulation.IntakeSide.BACK,
                        1);
    }

    @Override
    public Command intake() {
        return intake(false);
    }

    public Command intake(boolean overridden){
        return Commands.sequence(
                Commands.parallel(
                                goToState(SuperstructureState.Intake, overridden),
                                rollersCommands.setOpenLoop(0.25),
                                Commands.runOnce(intakeSim::startIntake))
                        .until(() -> intakeSim.getGamePiecesAmount() == 1),
                transfer().withTimeout(0.2) // add Current sensing here
                );
    }

    @Override
    public Command transfer() {
        return Commands.parallel(
                new GoToState(() -> SuperstructureState.Transfer),
                rollersCommands.setOpenLoop(0.6, -0.1),
                Commands.runOnce(
                        () -> {
                            if (intakeSim.getGamePiecesAmount() == 1) {
                                hasPeice = true;
                                intakeSim.obtainGamePieceFromIntake();
                            }

                        //     SimulatedArena.getInstance()
                        //             .addGamePiece(
                        //                     new ReefscapeCoralOnField(
                        //                             // We must specify a heading since the coral is
                        //                             // a tube
                        //                             new Pose2d(2, 2, Rotation2d.fromDegrees(90))));
                        }));
    }

    @Override
    public Command stopIntaking() {
        System.out.println("Intaking stopped");
        return Commands.runOnce(intakeSim::stopIntake);
    }

    // actually transfer coral

    @Override
    public Command handoff() {

        return Commands.parallel(
                rollersCommands.kill(),
                new GoToState(() -> SuperstructureState.Handoff),
                coralerCommands.intake());
    }

    @Override
    public Command setNoPeice() {
        return Commands.runOnce(
                () -> {
                    this.hasPeice = false;
                    intakeSim.obtainGamePieceFromIntake();
                });
    }

    // ew wha happened here
    @Override
    public Command poopCoral() {
        return Commands.waitSeconds(0.15)
                .andThen(
                        Commands.parallel(
                                coralerCommands.setOpenLoop(-0.3).withTimeout(0.1),
                                Commands.runOnce(
                                        () -> {
                                                Distance dontEvenRemember;
                                                if (preloaded){
                                                        dontEvenRemember = elevator.getHeight().plus(Meters.of(
                                                                        PivotConstants.kZeroedPivotPose.getZ() - kPreloadManipZero.getZ()
                                                                        ))
                                                        .plus(Inches.of( 20.335).times(
                                                                                Math.sin(pivot.getAngleRads()+ Math.PI/ 2)));
                                                } else {
                                                        dontEvenRemember = elevator.getHeight().plus(
                                                                Meters.of(PivotConstants.kZeroedPivotPose.getZ()))
                                                        .plus(Inches.of(20.335).times(
                                                                                Math.sin(pivot.getAngleRads()+ Math.PI/ 2)));
                                                }
                                            if (hasPeice) {
                                                System.out.println("PLEASEWORK");
                                                SimulatedArena.getInstance()
                                                        .addGamePieceProjectile(
                                                                new ReefscapeCoralOnFly(
                                                                        dtSim.getSimulatedDriveTrainPose()
                                                                                .getTranslation(),
                                                                        new Translation2d(Inches.of(20.335).times(Math.cos(
                                                                                                                pivot.getAngleRads() + Math.PI/ 2))
                                                                                        .plus(Meters.of(PivotConstants.kZeroedPivotPose.getX())).abs(Meters),
                                                                                        0),
                                                                        dtSim
                                                                                .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                                                        dtSim.getSimulatedDriveTrainPose()
                                                                                .getRotation(),
                                                                        dontEvenRemember,
                                                                        MetersPerSecond.of(0.01),
                                                                        pivot.getAngle()));
                                                hasPeice = false;
                                            }
                                        })))
                .withTimeout(0.3);
    }

    // public Optional<Pose3d> getCoralPose(){
    //     return intakeSim.getGamePiecesAmount() == 1 ? Optional.of(new Pose3d(
    //     0,
    //     0,
    //     0,
    //     new Rotation3d(
    //     0,
    //     0,
    //     Math.PI/2
    //     )
    //     )) : Optional.empty();
    // }

    @Override
    public void periodic() {
        Pose2d dtPose = dtSim.getSimulatedDriveTrainPose();
        if (intakeSim.getGamePiecesAmount() == 1 && !hasPeice) {

            Pose3d rotatedCoralPose =
                    kCoralRelativePose.rotateBy(new Rotation3d(dtPose.getRotation()));
            coralPoseLogged =
                    new Pose3d[] {
                        new Pose3d(
                                rotatedCoralPose.getX() + dtPose.getX(),
                                rotatedCoralPose.getY() + dtPose.getY(),
                                rotatedCoralPose.getZ(),
                                rotatedCoralPose.getRotation())
                    };
        } else if (hasPeice) {
            Pose3d rotatedCoralPose;
            if(!preloaded){ rotatedCoralPose =
                    kManipZero.rotateBy(
                            new Rotation3d(
                                    0,
                                    -pivot.getAngleRads() - Math.PI / 2,
                                    dtPose.getRotation().getRadians())); 
                } else {
                        rotatedCoralPose = kPreloadManipZero.rotateBy(
                                new Rotation3d(
                                        0,
                                        -pivot.getAngleRads() - Math.PI / 2,
                                        dtPose.getRotation().getRadians()));
                }


            coralPoseLogged =
                    new Pose3d[] {
                        new Pose3d(
                                rotatedCoralPose.getX() + dtPose.getX(),
                                rotatedCoralPose.getY() + dtPose.getY(),
                                rotatedCoralPose.getZ()
                                        + PivotConstants.kZeroedPivotPose.getZ()
                                        + elevator.getHeight().abs(Meters),
                                rotatedCoralPose.getRotation())
                    };
        } else {
            coralPoseLogged = new Pose3d[] {};
        }
        Logger.recordOutput("Coral pose", coralPoseLogged);
        Logger.recordOutput("Corals", intakeSim.getGamePiecesAmount());
        Logger.recordOutput("Superstructure/in Manipulaton", hasPeice);
    }

    @Override
    public String getName() {
        return "sim superstructur";
    }
}
