package frc.robot.subsystems.vision.objectdetection.simulatedfield;

import java.util.concurrent.ThreadLocalRandom;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import frc.lib.util.FieldLayout;
import frc.robot.RobotConstants;

public class SimulatedGamePieceConstants {
    public static final double SCORE_TOLERANCE_METERS = 0.3;
    static final double
            INTAKE_TOLERANCE_METERS = 0.3,
            LOADER_TOLERANCE_METERS = 0.05;
    private static final Translation2d scoreCheckPositionTranslation = FieldLayout.handleAllianceFlip(new Translation2d(4.625594, FieldLayout.kFieldWidth.in(Units.Meters) / 2), RobotConstants.isRedAlliance);
    private static final Translation2d ejectFuelFromHubPositionTranslation = FieldLayout.handleAllianceFlip(new Translation2d(5.189474, FieldLayout.kFieldWidth.in(Units.Meters) / 2), RobotConstants.isRedAlliance);
    public static final Translation3d
            SCORE_CHECK_POSITION = new Translation3d(scoreCheckPositionTranslation.getX(), scoreCheckPositionTranslation.getY(), 1.4),
            EJECT_FUEL_FROM_HUB_POSITION = new Translation3d(ejectFuelFromHubPositionTranslation.getX(), ejectFuelFromHubPositionTranslation.getY(), 0.762);
    static final Translation3d
            COLLECTION_CHECK_POSITION = new Translation3d(0.4, 0, 0),
            LOADER_CHECK_POSITION = new Translation3d(0, 0.13455, 0.2);

    static final int MAXIMUM_HELD_FUEL = 40;
    

    private static final int
            STARTING_FUEL_ROWS = 12,
            STARTING_FUEL_COLUMNS = 30;
    private static final double
            FUEL_DIAMETER_METERS = 0.15,
            STARTING_FUEL_X_POSITION_METERS = 7.357364,
            STARTING_FUEL_Y_POSITION_METERS = 1.724406,
            STARTING_FUEL_SPACING_METERS = 0.16;

    private static final int
            DEPOT_FUEL_ROWS = 6,
            DEPOT_FUEL_COLUMNS = 4;
    private static final Translation2d DEPOT_CENTER_POSITION = new Translation2d(0.31, 5.96);

    public static final double
            EJECTION_FROM_HUB_MINIMUM_VELOCITY_METERS_PER_SECOND = 4,
            EJECTION_FROM_HUB_MAXIMUM_VELOCITY_METERS_PER_SECOND = 15;
    public static final Rotation2d EJECTION_FROM_HUB_MAXIMUM_ANGLE = Rotation2d.fromDegrees(35);

    static {
        initializeFuel();
    }

    public static void initializeFuel() {
        // for (int i = 0; i < STARTING_FUEL_ROWS; i++) {
        //     for (int j = 0; j < STARTING_FUEL_COLUMNS; j++) {
        //         new SimulatedGamePiece(
        //                 STARTING_FUEL_X_POSITION_METERS + (i * STARTING_FUEL_SPACING_METERS),
        //                 STARTING_FUEL_Y_POSITION_METERS + (j * STARTING_FUEL_SPACING_METERS)
        //         );
        //     }
        // }

        for (int i = 0; i < 15; i++) {
            new SimulatedGamePiece(
                ThreadLocalRandom.current().nextDouble(-FieldLayout.rightNeutralZone.getXWidth(), FieldLayout.rightNeutralZone.getXWidth())/2.0 + FieldLayout.rightNeutralZone.getCenter().getX(),
                ThreadLocalRandom.current().nextDouble(-FieldLayout.rightNeutralZone.getYWidth(), FieldLayout.rightNeutralZone.getYWidth())/2.0 + FieldLayout.rightNeutralZone.getCenter().getY()
            );
        }

        // initializeDepotFuel(DEPOT_CENTER_POSITION.getX(), DEPOT_CENTER_POSITION.getY());
        // initializeDepotFuel(FieldLayout.kFieldLength.in(Units.Meters) - DEPOT_CENTER_POSITION.getX(), FieldLayout.kFieldWidth.in(Units.Meters) - DEPOT_CENTER_POSITION.getY());
    }

    private static void initializeDepotFuel(double depotCenterX, double depotCenterY) {
        double startX = depotCenterX - (DEPOT_FUEL_COLUMNS - 1) * STARTING_FUEL_SPACING_METERS / 2.0;
        double startY = depotCenterY - (DEPOT_FUEL_ROWS - 1) * STARTING_FUEL_SPACING_METERS / 2.0;

        for (int row = 0; row < DEPOT_FUEL_ROWS; row++) {
            for (int col = 0; col < DEPOT_FUEL_COLUMNS; col++) {
                new SimulatedGamePiece(
                        startX + col * STARTING_FUEL_SPACING_METERS,
                        startY + row * STARTING_FUEL_SPACING_METERS
                );
            }
        }
    }

    public enum GamePieceType {
        FUEL(FUEL_DIAMETER_METERS / 2.0, 0);

        public final double originPointHeightOffGroundMeters;
        public final int id;

        GamePieceType(double originPointHeightOffGroundMeters, int id) {
            this.originPointHeightOffGroundMeters = originPointHeightOffGroundMeters;
            this.id = id;
        }

        public static String getNameFromID(int id) {
            for (int i = 0; i < values().length; i++)
                if (values()[i].id == id)
                    return values()[i].toString();
            return "";
        }
    }
}