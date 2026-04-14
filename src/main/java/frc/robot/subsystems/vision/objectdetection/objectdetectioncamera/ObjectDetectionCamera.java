package frc.robot.subsystems.vision.objectdetection.objectdetectioncamera;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MathHelpers;
import frc.robot.auto.AutoModeSelector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;
import frc.robot.subsystems.vision.objectdetection.objectdetectioncamera.ObjectDetectionCameraIO.ObjectDetectionCameraInputs;
import frc.robot.subsystems.vision.objectdetection.objectdetectioncamera.io.PhotonObjectDetectionCameraIO;
import frc.robot.subsystems.vision.objectdetection.objectdetectioncamera.io.SimulationObjectDetectionCameraIO;
import frc.robot.subsystems.vision.objectdetection.simulatedfield.SimulatedGamePiece;
import frc.robot.subsystems.vision.objectdetection.simulatedfield.SimulatedGamePieceConstants;

public class ObjectDetectionCamera extends SubsystemBase {
    private final ObjectDetectionCameraInputs objectDetectionCameraInputs = new ObjectDetectionCameraInputs();
    private final ObjectDetectionCameraIO objectDetectionCameraIO;
    private final String hostname;
    private final Transform3d cameraTransform;
    private final Drive drive;

    public ObjectDetectionCamera(Drive drive, String hostname, Transform3d robotCenterToCamera) {
        this.hostname = hostname;
        this.drive = drive;
        this.cameraTransform = robotCenterToCamera;
        this.objectDetectionCameraIO = generateIO(hostname, robotCenterToCamera);
    }

    @Override
    public void periodic() {
        objectDetectionCameraIO.updateInputs(objectDetectionCameraInputs);
    }

    /**
     * Calculates the position of the closest object on the field from its 3D rotation relative to the camera.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @return the closest object's 2D position on the field (z is assumed to be 0)
     */
    public Translation2d calculateClosestObjectPositionOnField(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        final Translation2d[] targetObjectsTranslation = getObjectPositionsOnField(targetGamePiece);
        final Translation2d currentRobotTranslation = drive.getPose().getTranslation();
        if (targetObjectsTranslation.length == 0)
            return null;
        Translation2d closestObjectTranslation = targetObjectsTranslation[0];
        double closestObjectDistanceToRobot = currentRobotTranslation.getDistance(closestObjectTranslation);

        for (Translation2d currentObjectTranslation : targetObjectsTranslation) {
            final double currentObjectDistanceToRobot = currentRobotTranslation.getDistance(currentObjectTranslation);
            if (currentObjectDistanceToRobot < closestObjectDistanceToRobot) {
                closestObjectTranslation = currentObjectTranslation;
                closestObjectDistanceToRobot = currentObjectDistanceToRobot;
            }
        }
        return closestObjectTranslation;
    }

    public boolean hasObject(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        return objectDetectionCameraInputs.hasObject[targetGamePiece.id];
    }

    
    public Translation2d[] getObjectPositionsOnField(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        final Rotation3d[] visibleObjectRotations = getObjectsRotations(targetGamePiece);
        final Translation2d[] objectPositionsOnField = new Translation2d[visibleObjectRotations.length];

        for (int i = 0; i < visibleObjectRotations.length; i++)
            objectPositionsOnField[i] = calculateObjectPositionFromRotation(visibleObjectRotations[i]);

        return objectPositionsOnField;
    }

    public Rotation3d[] getObjectsRotations(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        return objectDetectionCameraInputs.visibleObjectRotations[targetGamePiece.id];
    }

    /**
     * Calculates the position of an object on the field from its 3D rotation relative to the camera.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @param objectRotation the object's 3D rotation relative to the camera
     * @return the object's 2D position on the field (z is assumed to be 0)
     */
    private Translation2d calculateObjectPositionFromRotation(Rotation3d objectRotation) {
        //maybe dont work depending upon latency
        final Pose2d robotPoseAtResultTimestamp = drive.getPose();
        if (robotPoseAtResultTimestamp == null)
            return new Translation2d();
        final Pose3d cameraPose = new Pose3d(robotPoseAtResultTimestamp).plus(cameraTransform);
        final Rotation3d fieldRelativeObjectRotation =
                objectRotation.rotateBy(cameraPose.getRotation());
        final Translation3d rayDirection = new Translation3d(1.0, fieldRelativeObjectRotation);

        if (Math.abs(rayDirection.getZ()) < 1e-6)
            return cameraPose.getTranslation().toTranslation2d();

        final double distanceToGround = -cameraPose.getZ() / rayDirection.getZ();
        final Translation3d objectPosition =
                cameraPose.getTranslation().plus(rayDirection.times(distanceToGround));

        return objectPosition.toTranslation2d();
    }

    private ObjectDetectionCameraIO generateIO(String hostname, Transform3d cameraTransform) {
        if (RobotBase.isSimulation())
            return new SimulationObjectDetectionCameraIO(drive, hostname, cameraTransform);
        return new PhotonObjectDetectionCameraIO(hostname);
    }
}
