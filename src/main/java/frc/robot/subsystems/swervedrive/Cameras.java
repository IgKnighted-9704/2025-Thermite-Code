package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
    /**
     * Camera Enum to select each camera
     */
    public class Cameras {
      /**
       * Latency alert to use when high latency is detected.
       */
      public final Alert latencyAlert;
      /**
       * Camera instance for comms.
       */
      public final PhotonCamera camera;
      /**
       * Pose estimator for camera.
       */
      public final PhotonPoseEstimator poseEstimator;
      /**
       * Standard Deviation for single tag readings for pose estimation.
       */
      private final Matrix<N3, N1> singleTagStdDevs;
      /**
       * Standard deviation for multi-tag readings for pose estimation.
       */
      private final Matrix<N3, N1> multiTagStdDevs;
      /**
       * Transform of the camera rotation and translation relative to the center of the robot
       */
      private final Transform3d robotToCamTransform;
      /**
       * Current standard deviations used.
       */
      public Matrix<N3, N1> curStdDevs;
      /**
       * Estimated robot pose.
       */
      public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

      /**
       * Simulated camera instance which only exists during simulations.
       */
      public PhotonCameraSim cameraSim;
      /**
       * Results list to be updated periodically and cached to avoid unnecessary queries.
       */
      public List<PhotonPipelineResult> resultsList = new ArrayList<>();
      /**
       * Last read from the camera timestamp to prevent lag due to slow data fetches.
       */
      private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

      /**
       * Construct a Photon Camera class with help. Standard deviations are fake values, experiment
       * and determine estimation noise on an actual robot.
       *
       * @param name Name of the PhotonVision camera found in the PV UI.
       * @param robotToCamRotation {@link Rotation3d} of the camera.
       * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
       * @param singleTagStdDevs Single AprilTag standard deviations of estimated poses from the
       *        camera.
       * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the
       *        camera.
       */
      public Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
          Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
        latencyAlert =
            new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

        camera = new PhotonCamera(name);

        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

        poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamTransform);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        this.singleTagStdDevs = singleTagStdDevs;
        this.multiTagStdDevs = multiTagStdDevsMatrix;

        if (Robot.isSimulation()) {
          SimCameraProperties cameraProp = new SimCameraProperties();
          // A 640 x 480 camera with a 100 degree diagonal FOV.
          cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
          // Approximate detection noise with average and standard deviation error in pixels.
          cameraProp.setCalibError(0.25, 0.08);
          // Set the camera image capture framerate (Note: this is limited by robot loop rate).
          cameraProp.setFPS(30);
          // The average and standard deviation in milliseconds of image data latency.
          cameraProp.setAvgLatencyMs(35);
          cameraProp.setLatencyStdDevMs(5);

          cameraSim = new PhotonCameraSim(camera, cameraProp);
          cameraSim.enableDrawWireframe(true);
        }
      }

      /**
       * Add camera to {@link VisionSystemSim} for simulated photon vision.
       *
       * @param systemSim {@link VisionSystemSim} to use.
       */
      public void addToVisionSim(VisionSystemSim systemSim) {
        if (Robot.isSimulation()) {
          systemSim.addCamera(cameraSim, robotToCamTransform);
        }
      }

      /**
       * Get the result with the least ambiguity from the best tracked target within the Cache. This
       * may not be the most recent result!
       *
       * @return The result in the cache with the least ambiguous best tracked target. This is not the
       *         most recent result!
       */
      public Optional<PhotonPipelineResult> getBestResult() {
        if (resultsList.isEmpty()) {
          return Optional.empty();
        }

        PhotonPipelineResult bestResult = resultsList.get(0);
        double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
        double currentAmbiguity = 0;
        for (PhotonPipelineResult result : resultsList) {
          currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
          if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
            bestResult = result;
            amiguity = currentAmbiguity;
          }
        }
        return Optional.of(bestResult);
      }

      /**
       * Get the latest result from the current cache.
       *
       * @return Empty optional if nothing is found. Latest result if something is there.
       */
      public Optional<PhotonPipelineResult> getLatestResult() {
        return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
      }

      /**
       * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations,
       * and flushes the cache of results.
       *
       * @return Estimated pose.
       */
      public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        updateUnreadResults();
        return estimatedRobotPose;
      }

      /**
       * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by
       * timestamp.
       */
      private void updateUnreadResults() {
        // double mostRecentTimestamp =
        //     resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
        // double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
        // double debounceTime = Milliseconds.of(15).in(Seconds);
        // for (PhotonPipelineResult result : resultsList) {
        //   mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
        // }
        // if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime))
        //     && (currentTimestamp - lastReadTimestamp) >= debounceTime) {
        //   resultsList = Robot.isReal() ? camera.getAllUnreadResults()
        //       : cameraSim.getCamera().getAllUnreadResults();
        //   lastReadTimestamp = currentTimestamp;
        //   resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
        //     return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        //   });
        //   if (!resultsList.isEmpty()) {
        //     updateEstimatedGlobalPose();
        //   }
        // }
        System.out.println(camera.isConnected()); //TODO : Check is the camera is connected.
        System.out.println(camera.getAllUnreadResults()); //TODO : Check if the camera is updating at all.
        System.out.println(Robot.isReal()); //TODO : Check if the robot is in simulation or real.
      }

      /**
       * The latest estimated robot pose on the field from vision data. This may be empty. This should
       * only be called once per loop.
       *
       * <p>
       * Also includes updates for the standard deviations, which can (optionally) be retrieved with
       * {@link Cameras#updateEstimationStdDevs}
       *
       * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
       *         used for estimation.
       */
      private void updateEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : resultsList) {
          visionEst = poseEstimator.update(change);
          updateEstimationStdDevs(visionEst, change.getTargets());
        }
        estimatedRobotPose = visionEst;
      }

      /**
       * Calculates new standard deviations This algorithm is a heuristic that creates dynamic
       * standard deviations based on number of tags, estimation strategy, and distance from the tags.
       *
       * @param estimatedPose The estimated pose to guess standard deviations for.
       * @param targets All targets in this camera frame
       */
      private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
          List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
          // No pose input. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;

        } else {
          // Pose present. Start running Heuristic
          var estStdDevs = singleTagStdDevs;
          int numTags = 0;
          double avgDist = 0;

          // Precalculation - see how many tags we found, and calculate an average-distance metric
          for (var tgt : targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {
              continue;
            }
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
          }

          if (numTags == 0) {
            // No tags visible. Default to single-tag std devs
            curStdDevs = singleTagStdDevs;
          } else {
            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) {
              estStdDevs = multiTagStdDevs;
            }
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > 4) {
              estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
              estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }
            curStdDevs = estStdDevs;
          }
        }
      }
    }