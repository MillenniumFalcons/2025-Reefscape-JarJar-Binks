package team3647.frc2025.Util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.WristConstants;

public class InverseKinematics {

    public static Angle getPivotMax(Distance elevatorHeight) {
        // pivot relative to top bar position
        double pivotRelToTopbar =
                MathUtil.clamp(
                        elevatorHeight
                                .minus(ElevatorConstants.kStage2Threshold)
                                .minus(ElevatorConstants.kCarriageHeight)
                                .minus(Inches.of(1))
                                .plus(ElevatorConstants.annoyinAssCarriageToPivotPointOffest)
                                .in(Meters),
                        -ElevatorConstants.kStage2Threshold
                                .minus(ElevatorConstants.kCarriageHeight)
                                .minus(Inches.of(1))
                                .plus(ElevatorConstants.annoyinAssCarriageToPivotPointOffest)
                                .in(Meters),
                        ElevatorConstants.annoyinAssCarriageToPivotPointOffest
                                .minus(Inches.of(1))
                                .in(Meters));
        // System.out.println("pivot rel to : " + pivotRelToTopbar);
        if (pivotRelToTopbar
                <= -PivotConstants.kArmLength.minus(PivotConstants.weirdPolycarbClear).in(Meters)) {
            return Radian.of(0); // vertical
        } else if (pivotRelToTopbar > 1) {
            return PivotConstants.kMaxAngle;
        } else if (pivotRelToTopbar >= 0) {
            return Radian.of(
                    MathUtil.clamp(
                            Math.acos(1 / PivotConstants.kHighIntersectXOffset.in(Inches))
                                    + Degree.of(2).in(Radian),
                            PivotConstants.kMinAngle.in(Radian),
                            PivotConstants.kMaxAngle.in(Radian)));
        } // weird trancendental bisection stuff
        double low = Math.toRadians(0.1);
        double high = Math.toRadians(89.9);
        double theta = 0;
        while (high - low > 1e-4) {
            double mid = (low + high) / 2;
            double fMid =
                    1 / Math.sin(mid)
                            + pivotRelToTopbar * Math.tan(mid)
                            - PivotConstants.kHighIntersectXOffset.in(Meters);
            if (fMid == 0) theta = mid;

            if ((1 / Math.sin(low)
                                    + pivotRelToTopbar * Math.tan(low)
                                    - PivotConstants.kHighIntersectXOffset.in(Meters))
                            * fMid
                    < 0) {
                high = mid;
            } else {
                low = mid;
            }
        }

        return Radians.of(theta);

        // double angle = Math.atan
        // return Radian.of(
        //         MathUtil.clamp(
        //                 Math.PI / 2
        //                         - Math.atan(
        //                                 (topBarHeight)
        //                                         /
        // PivotConstants.kHighIntersectXOffset.abs(Meters)),
        //                 0.0,
        //                 PivotConstants.kMaxAngle.in(Radian)));
    }

    public static Angle getPivotMin(Distance elevatorHeight) {
        if (elevatorHeight.gte(ElevatorConstants.kClearHeight)) {
            return PivotConstants.kMinAngle;
        } else if (elevatorHeight.gte(ElevatorConstants.kHandoffHeight)) {
            return PivotConstants.kHandoffAngle.minus(Degree.of(0.5));
        } else {
            return Radian.of(
                    MathUtil.clamp(
                            Math.acos(
                                            (PivotConstants.kLowIntersectOffset.abs(Meters)
                                                            + elevatorHeight.in(Meters))
                                                    / PivotConstants.kArmLength.in(Meters))
                                    - Math.PI,
                            PivotConstants.kMinAngle.in(Radian),
                            PivotConstants.kMaxAngle.in(Radian)));
        }
    }

    public static Distance getElevMin(Angle pivotAngle) {
        double trig = -1;
        if (pivotAngle.lte(PivotConstants.kHandoffAngle)) {
            Logger.recordOutput("Inverse Kinematics/Elev Min", "less than Handoff");
            return ElevatorConstants.kStowHeight;
        } else if (pivotAngle.isEquivalent(PivotConstants.kHandoffAngle)) {
            Logger.recordOutput("Inverse Kinematics/Elev Min", "equal to handoff");
            return ElevatorConstants.kHandoffHeight;
        } else if (pivotAngle.gt(Radians.of(0))) {
            Distance max =
                    Meters.of(
                            MathUtil.clamp(
                                    (PivotConstants.kHighIntersectXOffset.abs(Inches)
                                                            - 1 / Math.sin(pivotAngle.abs(Radians)))
                                                    / Math.tan(pivotAngle.abs(Radians))
                                            + ElevatorConstants.kStage2Threshold
                                                    .plus(ElevatorConstants.kCarriageHeight)
                                                    .plus(Inches.of(1))
                                                    .minus(
                                                            ElevatorConstants
                                                                    .annoyinAssCarriageToPivotPointOffest)
                                                    .in(Meters),
                                    ElevatorConstants.kMaxHeight.in(Meters),
                                    ElevatorConstants.kMinHeight.in(Meters)));
            Logger.recordOutput("Inverse Kinematics/Elev Min", max.in(Inches));
            return max;
        } else if ((Math.acos(
                                PivotConstants.kLowIntersectOffset.in(Meters)
                                        / PivotConstants.kArmLength.in(Meters))
                        - Math.PI)
                < pivotAngle.plus(Degrees.of(5)).in(Radians)) {
            Logger.recordOutput("Inverse Kinematics/Elev Min", "Min height");
            return ElevatorConstants.kMinHeight;
        }
        ;
        try {
            trig = Math.cos(pivotAngle.in(Radian) + Math.PI);
        } catch (Exception e) {
            System.out.println(e);
        }
        if (trig < 0) {
            Logger.recordOutput("Inverse Kinematics/Elev Min", "trig < 0, Stow");
            return ElevatorConstants.kStowHeight;
        }
        Distance max =
                Meters.of(
                        MathUtil.clamp(
                                trig * PivotConstants.kArmLength.in(Meters),
                                ElevatorConstants.kMinHeight.in(Meters),
                                ElevatorConstants.kMaxHeight.in(Meters)));
        Logger.recordOutput("Inverse Kinematics/ELev Min", max.in(Inches));
        return max;
    }

    public static boolean isValidState(SuperstructureState state) {
        // lmao have a good time reading this
        if (state.equals(SuperstructureState.Stow)) {
            return true;
        }
        return (state.wristAngle.lte(WristConstants.kMaxAngle)
                && state.wristAngle.gte(WristConstants.kMinAngle)
                && state.elevatorHeight.lte(ElevatorConstants.kMaxHeight)
                && state.elevatorHeight.gte(ElevatorConstants.kMinHeight)
                && state.pivotAngle.gte(getPivotMin(state.elevatorHeight))
                && state.pivotAngle.lte(getPivotMax(state.elevatorHeight)));
    }

        public static SuperstructureState interpolate(
            Supplier<SuperstructureState> current, Supplier<SuperstructureState> finalState, Supplier<Pose2d> robotPose) {
        boolean elevatorDependent =
                current.get()
                                .pivotAngle
                                .gte(getPivotMin(finalState.get().elevatorHeight))
                        && current.get()
                                .pivotAngle
                                .lte(
                                        getPivotMax(
                                                finalState.get().elevatorHeight));
        boolean sequence =
                finalState
                                .get()
                                .pivotAngle
                                .gte(getPivotMin(current.get().elevatorHeight))
                        && finalState
                                .get()
                                .pivotAngle
                                .lte(getPivotMax(current.get().elevatorHeight));
        if (team3647.frc2025.Util.PoseUtils.inCircle(
            robotPose.get(),
            new Pose2d(team3647.frc2025.constants.FieldConstants.kBlueReefOrigin, Rotation2d.kZero),
            Inches.of((93.5 + 3.5)/2)) || 
        team3647.frc2025.Util.PoseUtils.inCircle(robotPose.get(),
            new Pose2d(AllianceFlip.flip(team3647.frc2025.constants.FieldConstants.kBlueReefOrigin), Rotation2d.kZero),
            Inches.of((93.5 + 3.5)/2.0))
        ){
            return current.get(); 
        } else if (elevatorDependent && sequence) {
            return finalState.get();
        } else if (elevatorDependent) {
            // rotating positive
            if (finalState.get().pivotAngle.gte(current.get().pivotAngle))
                return getPivotMax(current.get().elevatorHeight)
                                .lt(finalState.get().pivotAngle)
                        ? finalState
                                .get()
                                .withPivotAngle(
                                        getPivotMax(current.get().elevatorHeight)
                                                .minus(Degrees.of(5)))
                        : finalState.get();

            return getPivotMin(current.get().elevatorHeight)
                            .gt(finalState.get().pivotAngle)
                    ? finalState
                            .get()
                            .withPivotAngle(
                                    getPivotMin(current.get().elevatorHeight)
                                            .plus(Degrees.of(5)))
                    : finalState.get();
        } else if (!elevatorDependent) {
            // like stow to L1
            // going up
            if (finalState.get().elevatorHeight.gte(current.get().elevatorHeight)) {
                if (finalState.get().pivotAngle.gte(current.get().pivotAngle)) {
                    return getPivotMax(current.get().elevatorHeight)
                                    .gte(finalState.get().pivotAngle)
                            ? finalState.get()
                            : finalState
                                    .get()
                                    .withPivotAngle(
                                            getPivotMax(
                                                    current.get().elevatorHeight));
                } else {
                    return getPivotMin(current.get().elevatorHeight)
                                    .lt(finalState.get().pivotAngle)
                            ? finalState.get()
                            : finalState
                                    .get()
                                    .withPivotAngle(
                                            getPivotMin(
                                                    current.get().elevatorHeight));
                }
            }
            return getElevMin(current.get().pivotAngle)
                            .gt(finalState.get().elevatorHeight)
                    ? finalState
                            .get()
                            .withElevatorHeight(
                                    getElevMin(current.get().pivotAngle))
                    : finalState.get();
        }
        System.out.println("Stuck!");
        return current.get();
    }
}
