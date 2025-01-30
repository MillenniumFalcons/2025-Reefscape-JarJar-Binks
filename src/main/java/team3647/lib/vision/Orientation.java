package team3647.lib.vision;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Orientation {
    public Angle yaw;
    public Angle pitch;
    public Angle roll;

    public AngularVelocity yawRate;
    public AngularVelocity pitchRate;
    public AngularVelocity rollRate;

    public Orientation(
            Angle yaw,
            Angle pitch,
            Angle roll,
            AngularVelocity yawRate,
            AngularVelocity pitchRate,
            AngularVelocity rollRate) {
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
        this.yawRate = yawRate;
        this.pitchRate = pitchRate;
        this.rollRate = rollRate;
    }
}
