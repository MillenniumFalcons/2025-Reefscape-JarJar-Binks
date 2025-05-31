package team3647.lib.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;

public class NeuralDetectorPhotonVision extends PhotonCamera implements NeuralDetector {
    public NeuralDetectorPhotonVision(String camera) {
        super(NetworkTableInstance.getDefault(), camera);
    }

    public double getTX() {
        if (this.hasTarget()) {
            return this.getTX();
        } else {
            return 0;
        }
    }

    public double getTY() {
        if (this.hasTarget()) {
            return this.getTY();
        } else {
            return 0;
        }
    }

    public boolean hasTarget() {
        return this.hasTarget();
    }
}
