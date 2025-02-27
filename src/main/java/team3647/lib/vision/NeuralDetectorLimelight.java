package team3647.lib.vision;

public class NeuralDetectorLimelight implements NeuralDetector {
    private final String name;

    public NeuralDetectorLimelight(String name) {
        this.name = name;
    }

    @Override
    public double getTX() {
        return LimelightHelpers.getTX(name);
    }

    @Override
    public double getTY() {
        return LimelightHelpers.getTY(name);
    }

    @Override
    public boolean hasTarget() {
        var detections = LimelightHelpers.getRawDetections(name);
        return detections.isPresent();
    }
}
