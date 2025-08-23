package frc.robot.subsystems.swervedrive;

import java.util.Optional;
import java.util.function.Supplier;

import org.opencv.core.RotatedRect;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;

public class CustomVision {
    
    /**
     * April Tag Field Layout of the year.
     */
    public static final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    /**
     * Ambiguity defined as a value between (0,1). Used in {@link Vision#filterPose}.
     */
    private final double maximumAmbiguity = 0.25;
    /**
     * Photon Vision Simulation
     */
    public VisionSystemSim visionSim;
    /**
     * Count of times that the odom thinks we're more than 10meters away from the april tag.
     */
    private double longDistangePoseEstimationCount = 0;
    /**
     * Current pose from the pose estimator using wheel odometry.
     */
    private Supplier<Pose2d> currentPose;
    /**
     * Field from {@link swervelib.SwerveDrive#field}
     */
    private Field2d field2d = new Field2d();

    // Latest PhotonPipelineResult
    private PhotonPipelineResult latestResult;

    // Camera of Latest PhotonPipelineResult
    private PhotonCamera ActiveCam;
    
    // Swerve Subsystem
    private SwerveSubsystem driveBase;

    //Photon Cameras Stored List
    private ArrayList <PhotonCamera> Cameras;

    //Photon Cameras
    private final PhotonCamera CAMERA_LEFT;
    private final PhotonCamera CAMERA_RIGHT;

    public CustomVision(String LeftCamera, String RightCamera, SwerveSubsystem drivebase){
        //Initializing Cameras
        CAMERA_LEFT = new PhotonCamera(LeftCamera); 
        CAMERA_RIGHT = new PhotonCamera(RightCamera);
        
        //Adding both cameras to Global List
        Cameras.add(CAMERA_LEFT);
        Cameras.add(CAMERA_RIGHT);

        //Initializing drivebase
        drivebase = this.driveBase;

    }

    public Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
      Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
      if (aprilTagPose3d.isPresent()) {
        return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
      } else {
        throw new RuntimeException(
            "Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
      }
    }

    public double getDistanceFromAprilTag(int id) {
      Optional<Pose3d> tag = fieldLayout.getTagPose(id);
      return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d()))
          .orElse(-1.0);
    }

    public void UpdateToLatestResult(){
        PhotonCamera ActiveCamera = Cameras.get(0);
        List<PhotonPipelineResult> ListResults = ActiveCamera.getAllUnreadResults();
        var BestResult = ListResults.get(ListResults.size()-1);
        for (PhotonCamera cam : Cameras){
            ListResults = cam.getAllUnreadResults();
            var currResult = ListResults.get(ListResults.size()-1);
            if(BestResult.getBestTarget().getPoseAmbiguity() > currResult.getBestTarget().getPoseAmbiguity()){
                if(currResult.getBestTarget().getPoseAmbiguity() < maximumAmbiguity){
                    BestResult = currResult;
                    ActiveCamera = cam;
                }
            } 
        }

        if(BestResult.getBestTarget().getPoseAmbiguity() < maximumAmbiguity){
            latestResult = BestResult;
            ActiveCam = ActiveCamera;
        }
    }

    public PhotonTrackedTarget getLatestTarget() {
        return latestResult.getBestTarget();
    }

    public Pose2d getRobotPose(){

        PhotonTrackedTarget bestTrackedTarget = getLatestTarget();

        double kCameraHeight = 0.0;
        double kTargetHeight = 0.0;
        double kCameraPitch = Math.toRadians(0);
        double kTargetPitch = Math.toRadians(0);
        Rotation2d targetYaw = Rotation2d.fromDegrees(-bestTrackedTarget.getYaw());
        Transform2d cameraToRobot = new Transform2d(new Translation2d(0, 0), new Rotation2d(0, 0));
        Rotation2d gyroRotation2d = driveBase.getPose().getRotation();
        Pose2d targetPose = getAprilTagPose(bestTrackedTarget.getFiducialId(), new Transform2d());
        

        Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
            kCameraHeight,
            kTargetHeight,
            kCameraPitch,
            kTargetPitch,
            targetYaw,
            gyroRotation2d,
            targetPose,
            cameraToRobot
        );
        return robotPose;
    }

    
}
