import edu.wpi.first.math.geometry.Transform3d;
/** Fiducial Marker Identification Utility */

public class VisionPoint {
    public int id; // -2 if not detected
    public Transform3d cameraToTarget;
    public double weight;

    public VisionPoint() {
        id = 0;
        cameraToTarget = new Transform3d();
        weight = 0;
    }

    public String toString() {
        return "id: " + id + " camera to target: " + cameraToTarget;
    }
}
