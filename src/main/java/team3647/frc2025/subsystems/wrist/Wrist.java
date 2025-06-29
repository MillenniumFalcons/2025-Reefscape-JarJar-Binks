package team3647.frc2025.subsystems.wrist;

import edu.wpi.first.units.measure.Angle;
import team3647.lib.PeriodicSubsystem;

public interface Wrist extends PeriodicSubsystem {

    public void setAngle(Angle angle);

    public void setEncoderAngle(Angle angle);

    public Angle getAngle();

    public double getAngleDegs();

    public boolean angleWithin(double lowBound, double highBound);

    public boolean angleReached(double angle, double tolerance);
}
