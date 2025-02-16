package team3647.lib.vision;



public class NeuralDetectorLimelight implements NeuralDetector{
	private final String name;

	public NeuralDetectorLimelight(String name){
		this.name = name;
	}



	@Override
	public double getTX() {
		var detections = LimelightHelpers.getRawDetections(name);
		if (detections.isEmpty()) return 0;
		return detections.get()[0].txnc;
		
	}

	@Override
	public double getTY() {
		var detections = LimelightHelpers.getRawDetections(name);
		if (detections.isEmpty()) return 0;
		return detections.get()[0].tync;
	}

	@Override
	public boolean hasTarget() {
		var detections = LimelightHelpers.getRawDetections(name);
		return detections.isPresent();
	}
}
