package team3647.lib.vision;



public class NeuralDetectorLimelight implements NeuralDetector{
	private final String name;

	public NeuralDetectorLimelight(String name){
		this.name = name;
	}



	@Override
	public double getTX() {
		var detections = LimelightHelpers.getTX(name);
		return detections;
		
	}

	@Override
	public double getTY() {
		var detections = LimelightHelpers.getTY(name);
		return detections;
	}

	@Override
	public boolean hasTarget() {
		var detections = LimelightHelpers.getRawDetections(name);
		return detections.isPresent();
	}
}
