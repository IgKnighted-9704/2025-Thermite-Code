package frc.robot.subsystems.swervedrive;

import java.util.List;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Optional;
import edu.wpi.first.math.util.Units;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CustomVision extends SubsystemBase {

    // List of all cameras used in this subsystem
    private final ArrayList<PhotonCamera> cameras = new ArrayList<>();
    
    // Load the AprilTag field layout for the 2025 Reefscape field
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    
    // Reference to the swerve drive subsystem
    private final SwerveSubsystem driveBase;
    
    // Store the latest results from all cameras
    private List<PhotonPipelineResult> results = new ArrayList<>(20);
    
    // Timestamp for the last read of camera results
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    // Individual references for two cameras
    private PhotonCamera CAMERA_LEFT;
    private PhotonCamera CAMERA_RIGHT;

    //Maximum Camera Ambiguity
    private double MAX_AMBIGUITY = 0.25;

    // Constructor: initializes cameras and stores reference to driveBase
    public CustomVision(SwerveSubsystem driveBase, String... args) {
        for (String cameraName : args) {
            cameras.add(new PhotonCamera(cameraName)); // Create a PhotonCamera for each name
        }
        this.driveBase = driveBase;

        // Assign left and right cameras for convenience
        CAMERA_LEFT = cameras.get(0);
        CAMERA_RIGHT = cameras.get(1);
    }

    // Returns the 3D pose of a specific AprilTag, applying a 3D transform offset (e.g., robot approach)
    public Pose3d getAprilTagPose(int aprilTag, Transform3d robotOffset) {
        // Look up the AprilTag pose from the field layout
        Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);

        // If the tag exists, apply the given offset and return the transformed pose
        if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().transformBy(robotOffset);
        } else {
            // Throw an exception if the tag cannot be found in the field layout
            throw new RuntimeException(
                "Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
        }
    }

    // Returns the 2D pose of a specific AprilTag, applying a 2D transform offset (for planar movement)
    public Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
        // Look up the AprilTag pose from the field layout
        Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);

        // If the tag exists, convert it to 2D and apply the given 2D offset
        if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        } else {
            // Throw an exception if the tag cannot be found in the field layout
            throw new RuntimeException(
                "Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
        }
    }

    // Updates the list of camera results while debouncing to avoid duplicate reads
    private void updateUnreadResults() {
        double mostRecentTimestamp = results.isEmpty() ? 0.0 : results.get(0).getTimestampSeconds();
        double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
        double debounceTime = Milliseconds.of(15).in(Seconds);

        // Find the most recent timestamp among stored results
        for (PhotonPipelineResult result : results) {
            mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
        }

        // If enough time has passed, read latest results from both cameras
        if ((results.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime))
            && (currentTimestamp - lastReadTimestamp) >= debounceTime) {
            
            if (results.size() < 20) {
                for(var left : CAMERA_LEFT.getAllUnreadResults()){
                    results.add(left);
                }
                for(var right : CAMERA_RIGHT.getAllUnreadResults()){
                    results.add(right);
                }
            } else {
                results.clear();
                for(var left : CAMERA_LEFT.getAllUnreadResults()){
                    results.add(left);
                }
                for(var right :  CAMERA_RIGHT.getAllUnreadResults()){
                    results.add(right);
                }
            }

            lastReadTimestamp = currentTimestamp;

            // Sort results by timestamp, newest first
            results.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
                return b.getTimestampSeconds() >= a.getTimestampSeconds() ? 1 : -1;
            });
        }
    }

    // Returns the best PhotonPipelineResult based on lowest pose ambiguity
    public Optional<PhotonPipelineResult> getBestResult() {
        if (results.isEmpty()) {
            return Optional.empty();
        }

        PhotonPipelineResult bestResult = results.get(0);
        double ambiguity = bestResult.getBestTarget().getPoseAmbiguity();
        double currentAmbiguity = 0;

        // Iterate through results to find the one with the lowest ambiguity
        for (PhotonPipelineResult result : results) {
            currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
            if (currentAmbiguity < ambiguity && currentAmbiguity > 0) {
                bestResult = result;
                ambiguity = currentAmbiguity;
            }
        }
        return Optional.of(bestResult);
    }


    // Returns the robot's pose relative to the field based on the best-seen AprilTag
    private Pose3d getTagRelativePose() {
        // Start with the first camera in the list as the default active camera
        PhotonCamera ActiveCamera = cameras.get(0);

        // Initialize variables to track the best target seen
        double lowestAmbiguity = Double.MAX_VALUE; // Track the lowest pose ambiguity seen
        int fid = -1; // Fiducial ID of the best target
        PhotonTrackedTarget bestTarget = null; // The best target object

        // Loop through all cameras to find the target with the lowest ambiguity
        for (PhotonCamera cam : cameras) {
            // Only process cameras with new (unread) results
            if (!cam.getAllUnreadResults().isEmpty()) {
                // Get the pose ambiguity of the latest target for this camera
                var currentAmbiguity = cam.getAllUnreadResults().get(cam.getAllUnreadResults().size() - 1)
                                        .getBestTarget().getPoseAmbiguity();

                // Update best target if this one has lower ambiguity and is below a threshold
                if (currentAmbiguity < lowestAmbiguity && currentAmbiguity < MAX_AMBIGUITY) { 
                    ActiveCamera = cam; // Use this camera as the active camera
                    lowestAmbiguity = currentAmbiguity; // Update lowest ambiguity
                    fid = cam.getAllUnreadResults().get(cam.getAllUnreadResults().size() - 1)
                            .getBestTarget().getFiducialId(); // Store fiducial ID
                    bestTarget = cam.getAllUnreadResults().get(cam.getAllUnreadResults().size() - 1)
                                    .getBestTarget(); // Store target
                }
            }
        }

        // Default robot pose if no valid target is found
        Pose3d robotPose3d = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

        // Only calculate the robot pose if a target was found
        if (fid != -1 && bestTarget != null) {
            // Transform from camera to target
            Transform3d cameraToTarget = new Transform3d(
                new Translation3d(
                    bestTarget.getBestCameraToTarget().getTranslation().getX(), 
                    bestTarget.getBestCameraToTarget().getTranslation().getY(),
                    bestTarget.getBestCameraToTarget().getTranslation().getZ()
                ), 
                new Rotation3d(
                    bestTarget.getBestCameraToTarget().getRotation().getX(),
                    bestTarget.getBestCameraToTarget().getRotation().getY(), 
                    bestTarget.getBestCameraToTarget().getRotation().getZ()
                )
            );

            // Get the known field pose of the AprilTag
            Pose3d fieldToTarget = getAprilTagPose(fid, new Transform3d());

            // Transform from camera to robot (camera offset from robot center)
            Transform3d cameraToRobot = new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(27), // x offset
                    Units.inchesToMeters(-23), // y offset
                    Units.inchesToMeters(8.75) // z offset
                ),
                ActiveCamera == CAMERA_LEFT ? 
                    new Rotation3d(0, Math.toRadians(25), 0) : // Rotation if left camera
                    new Rotation3d(0, Math.toRadians(155), 0)   // Rotation if right camera
            );

            // Estimate robot's pose on the field using the camera-to-target and camera-to-robot transforms
            robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, fieldToTarget, cameraToRobot);
        }

        // Return the calculated or default robot pose
        return robotPose3d;
    }

    // Returns the fiducial ID of the visible reef tag, or -1 if none found
    public int findVisibleReefTag(List<Integer> tagColors) {
        var bestResult = getBestResult();
        if (bestResult.isEmpty()) {
            return -1;
        }
        var bestTarget = bestResult.get().getBestTarget();
        if (bestTarget != null) {
            for(Integer match : tagColors) {
                if (bestTarget.getFiducialId() == match) {
                    return match;
                }

            }
        }
        return -1;
    }

    // Creates a command to drive the robot to a reef scoring position based on alliance and branch
    public Command createReefScoreCommand(boolean leftBranch) {

        // Determine current alliance color
        var driverAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        List<Integer> TagColors;
        double yOffSet;

        // Assign reef tags based on alliance
        if (driverAlliance == DriverStation.Alliance.Red) {
            TagColors = Constants.ReefConstants.REEF_RED_IDS;
        } else {
            TagColors = Constants.ReefConstants.REEF_BLUE_IDS;
        }

        // Find the visible reef tag ID
        int targetId = findVisibleReefTag(TagColors);

        // If no tag is visible, return a command that logs an error
        if (targetId == -1) {
            return Commands.runOnce(() -> System.out.println("No Visible Reef Tag"));
        }

        // Determine lateral offset based on left or right branch
        yOffSet = leftBranch ? Constants.ReefConstants.BRANCH_OFFSET_METERS
                             : -Constants.ReefConstants.BRANCH_OFFSET_METERS;

        // Create a Transform2d to adjust approach position
        Transform2d offsetTransform2d = new Transform2d(
                new Translation2d(Constants.ReefConstants.APPROACH_X_OFFSET_METERS, yOffSet),
                new Rotation2d());

        // Calculate the robot's target pose for scoring
        Pose2d reefContactPose = getAprilTagPose(targetId, offsetTransform2d);

        // Return a command to drive the robot to the reef contact pose
        return driveBase.twoStepApproach(reefContactPose, 0.1);
    }

    // Called periodically by the scheduler; updates camera results
    @Override
    public void periodic() {
        updateUnreadResults();
    }

}
