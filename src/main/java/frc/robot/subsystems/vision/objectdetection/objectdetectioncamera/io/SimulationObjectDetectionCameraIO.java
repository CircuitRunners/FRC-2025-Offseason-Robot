package frc.robot.subsystems.vision.objectdetection.objectdetectioncamera.io;


import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionConstants;
import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;
import frc.robot.subsystems.vision.objectdetection.objectdetectioncamera.ObjectDetectionCameraIO;
import frc.robot.subsystems.vision.objectdetection.simulatedfield.SimulatedGamePiece;
import frc.robot.subsystems.vision.objectdetection.simulatedfield.SimulatedGamePieceConstants;

public class SimulationObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private static final Rotation2d
            CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(70),
            CAMERA_VERTICAL_FOV = Rotation2d.fromDegrees(47.2);

    private final String hostname;
    private final Transform3d cameraTransform;
    private final Drive drive;
    

    public SimulationObjectDetectionCameraIO(Drive drive, String hostname, Transform3d cameraTransform) {
        this.hostname = hostname;
        this.drive = drive;
        this.cameraTransform = cameraTransform;
        
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputs inputs) {
        final Pose2d robotPose = drive.getPose();
        final double currentTimestamp = Timer.getFPGATimestamp();
        final Pose3d cameraPose = new Pose3d(robotPose).plus(cameraTransform);
        final ArrayList<Pair<SimulatedGamePiece, Rotation3d>>[] visibleGamePieces = calculateAllVisibleGamePieces(cameraPose);

        boolean hasAnyObject = false;
        for (int i = 0; i < ObjectDetectionConstants.NUMBER_OF_GAME_PIECE_TYPES; i++) {
            inputs.hasObject[i] = !visibleGamePieces[i].isEmpty();
            if (inputs.hasObject[i])
                hasAnyObject = true;
        }

        if (hasAnyObject) {
            updateHasNewResultInputs(inputs, visibleGamePieces, currentTimestamp);
            return;
        }

        updateNoNewResultInputs(inputs);
    }

    private ArrayList<Pair<SimulatedGamePiece, Rotation3d>>[] calculateAllVisibleGamePieces(Pose3d cameraPose) {
        final ArrayList<Pair<SimulatedGamePiece, Rotation3d>>[] visibleGamePieces = new ArrayList[ObjectDetectionConstants.NUMBER_OF_GAME_PIECE_TYPES];
        for (int i = 0; i < visibleGamePieces.length; i++)
            visibleGamePieces[i] = calculateVisibleGamePiecesRotations(cameraPose);
        return visibleGamePieces;
    }

    private void updateNoNewResultInputs(ObjectDetectionCameraInputs inputs) {
        inputs.hasObject = new boolean[ObjectDetectionConstants.NUMBER_OF_GAME_PIECE_TYPES];
        inputs.visibleObjectRotations = new Rotation3d[ObjectDetectionConstants.NUMBER_OF_GAME_PIECE_TYPES][0];

    }

    private void updateHasNewResultInputs(ObjectDetectionCameraInputs inputs, ArrayList<Pair<SimulatedGamePiece, Rotation3d>>[] visibleGamePieces, double currentTimestamp) {
        for (int i = 0; i < visibleGamePieces.length; i++) {
            inputs.visibleObjectRotations[i] = new Rotation3d[visibleGamePieces[i].size()];
            for (int j = 0; j < visibleGamePieces[i].size(); j++)
                inputs.visibleObjectRotations[i][j] = visibleGamePieces[i].get(j).getSecond();
        }

        inputs.latestResultTimestamp = currentTimestamp;

        logVisibleGamePieces(visibleGamePieces);
    }

    /**
     * Calculates the placements of all visible objects by checking if they are within range and within the horizontal FOV.
     *
     * @param cameraPose the position of the robot on the field
     * @return the placements of the visible objects, as a pair of the object and the rotation of the object relative to the camera
     */
    private ArrayList<Pair<SimulatedGamePiece, Rotation3d>> calculateVisibleGamePiecesRotations(Pose3d cameraPose) {

        final ArrayList<SimulatedGamePiece> gamePiecesOnField = SimulatedGamePiece.getSimulatedGamePieces();
        
        final ArrayList<Pair<SimulatedGamePiece, Rotation3d>> visibleObjects = new ArrayList<>();
        for (SimulatedGamePiece currentObject : gamePiecesOnField) {
            final Rotation3d cameraAngleToObject =
                    calculateCameraAngleToObject(getDetectionPoint(currentObject), cameraPose);

            if (isObjectWithinFOV(cameraAngleToObject))
                visibleObjects.add(new Pair<>(currentObject, cameraAngleToObject));
        }
        
        return visibleObjects;
        
    }

    

    private Rotation3d calculateCameraAngleToObject(Translation3d objectPosition, Pose3d cameraPose) {
        final Translation3d cameraPosition = cameraPose.getTranslation();

        final Translation3d difference = cameraPosition.minus(objectPosition);
        final Rotation3d differenceAsAngle = getAngle(difference);

        return differenceAsAngle.minus(cameraPose.getRotation());
    }

    private Translation3d getDetectionPoint(SimulatedGamePiece gamePiece) {
        final Translation3d objectPosition = gamePiece.getPosition();
        return new Translation3d(objectPosition.getX(), objectPosition.getY(), 0.0);
    }

    private Rotation3d getAngle(Translation3d translation) {
        return new Rotation3d(0, getPitch(translation).getRadians(), getYaw(translation).getRadians());
    }

    /**
     * Extracts the yaw off of a 3d vector.
     *
     * @param vector the vector to extract the yaw from
     * @return the yaw of the vector
     */
    private Rotation2d getYaw(Translation3d vector) {
        return new Rotation2d(Math.atan2(-vector.getY(), -vector.getX()));
    }

    /**
     * Extracts the pitch off of a 3d vector.
     *
     * @param vector the vector to extract the pitch from
     * @return the pitch of the vector
     */
    private Rotation2d getPitch(Translation3d vector) {
        return new Rotation2d(Math.atan2(vector.getZ(), Math.hypot(vector.getX(), vector.getY())));
    }

    /**
     * Checks if an object is within the field-of-view of the camera.
     *
     * @param objectRotation the rotation of the object relative to the camera
     * @return if the object is within the field-of-view of the camera
     */
    private boolean isObjectWithinFOV(Rotation3d objectRotation) {
        return Math.abs(objectRotation.getZ()) <= CAMERA_HORIZONTAL_FOV.getRadians() / 2 &&
                Math.abs(objectRotation.getY()) <= CAMERA_VERTICAL_FOV.getRadians() / 2;
    }

    

    private void logVisibleGamePieces(ArrayList<Pair<SimulatedGamePiece, Rotation3d>>[] visibleGamePieces) {
        ArrayList<SimulatedGamePiece> q = new ArrayList<>();
        for (ArrayList<Pair<SimulatedGamePiece, Rotation3d>> a:visibleGamePieces) {
            for (Pair<SimulatedGamePiece, Rotation3d> p : a) {
                q.add(p.getFirst());
            }
        }
    }


    public void displayFuel(ArrayList<SimulatedGamePiece> gamePiecesOnField) {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (SimulatedGamePiece s: gamePiecesOnField) {
            poses.add(new Pose2d(
                new Translation2d(
                    s.getPosition().getX(),
                    s.getPosition().getY()
                ),
                Rotation2d.kZero
            )
            );
        }
        // ObjectPoseEstimator.field.getObject("Fuel").setPoses(poses);
    }
    private Pose3d[] mapSimulatedGamePieceListToPoseArray(ArrayList<Pair<SimulatedGamePiece, Rotation3d>> gamePieces) {
        final Pose3d[] poses = new Pose3d[gamePieces.size()];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = new Pose3d(gamePieces.get(i).getFirst().getPosition(), new Rotation3d());
        }
    
        return poses;
        
    }
}
