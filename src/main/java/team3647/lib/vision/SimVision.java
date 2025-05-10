package team3647.lib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.function.Supplier;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class SimVision {
    // A vision system sim labelled as "main" in NetworkTables
    VisionSystemSim visionSim = new VisionSystemSim("main");
    SimCameraProperties cameraProp = new SimCameraProperties();

    // The layout of AprilTags which we want to add to the vision system
    AprilTagFieldLayout tagLayout;

    PhotonCameraSim[] cameras;

    Supplier<Pose2d> simPose;

    public SimVision(
            AprilTagFieldLayout tags,
            Supplier<Pose2d> simPoseSupplier,
            AprilTagPhotonVision... cameras) {
        if (RobotBase.isReal()) {
            throw new IllegalStateException("TRYING TO RUN SIM VISION ON A REAL ROBOT");
        }
        this.tagLayout = tags;
        this.simPose = simPoseSupplier;
        visionSim.addAprilTags(tags);
        this.cameras = new PhotonCameraSim[cameras.length];

        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProp.setCalibError(0, 0);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProp.setFPS(200);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(25);
        cameraProp.setLatencyStdDevMs(5);

        for (int i = 0; i < cameras.length; i++) {
            this.cameras[i] = new PhotonCameraSim(cameras[i], cameraProp);
            visionSim.addCamera(this.cameras[i], cameras[i].robotToCam);
            this.cameras[i].setTargetSortMode(PhotonTargetSortMode.Largest);

            this.cameras[i].enableProcessedStream(true);
            this.cameras[i].enableDrawWireframe(true);
        }
    }

    public void periodic() {
        visionSim.update(simPose.get());
    }

    public Field2d getDebugField() {
        return visionSim.getDebugField();
    }
}
