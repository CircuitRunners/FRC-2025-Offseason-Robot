package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.io.BeamBreakIO;
import frc.lib.io.BeamBreakIOCANRange;
import frc.lib.io.BeamBreakIOSim;
import frc.lib.util.FieldLayout.Level;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.controlboard.ControlBoardConstants;
import frc.robot.subsystems.endeffector.EndEffector;

public class SuperstructureConstants {
	public static final CANrangeConfiguration CANRangeConfig = new CANrangeConfiguration();
	public static class BeamBreakConstants {
		public static BeamBreakIO getEndEffectorCoralBeamBreak() {
			if (Robot.isReal()) {
				try {
					return new BeamBreakIOCANRange(
							Ports.END_EFFECTOR_CORAL_BREAMBREAK,
							SuperstructureConstants.CANRangeConfig,
							SuperstructureConstants.kEndEffectorCoralDebounce,
							"Coral End Effector Break",
							2500);
				} catch (Exception e) {
					SmartDashboard.putString("End Effector Beam Break", "Failed");
					return new BeamBreakIOSim(
							() -> false, SuperstructureConstants.kEndEffectorCoralDebounce, "Coral End Effector Break");
				}
			} else {
				return new BeamBreakIOSim(
						ControlBoardConstants.mOperatorController.povDownRight().or(() -> Robot.isSimulation()),
						SuperstructureConstants.kEndEffectorCoralDebounce,
						"Coral End Effector Break");
			}
		}

		public static BeamBreakIO getEndEffectorVelocityDip(EndEffector endeffector) {
			if (Robot.isReal()) {
				return new BeamBreakIOSim(
						() -> endeffector.getVelocity().abs(Units.DegreesPerSecond)
								< kEndEffectorVelocityDip.in(Units.DegreesPerSecond),
						Units.Seconds.of(0.05),
						"End Effector Velocity Dip");
			} else {
				return new BeamBreakIOSim(() -> true, Units.Seconds.of(0.05), "End Effector Velocity Dip");
			}
		}
	}

	public static final Angle kReefHeadingGenerationDeadband = Units.Degrees.of(0.0);
	public static final Angle kProcessorAlgaeHeadingGenerationDeadband = Units.Degrees.of(0.0);
	public static final Angle kL1HeadingGenerationDeadband = Units.Degrees.of(0.0);
	public static final Angle kL2HeadingGenerationDeadband = Units.Degrees.of(0.0);
	public static final Angle kL3HeadingGenerationDeadband = Units.Degrees.of(0.0);
	public static final Angle kL4HeadingGenerationDeadband = Units.Degrees.of(0.0);

    public static final Distance kReefScoringDistanceEpsilon = Units.Centimeters.of(3.5);
	public static final Distance kProcessorAlgaeScoringDistanceEpsilon = Units.Centimeters.of(12.0);
    public static final Distance kL1ScoringDistanceEpsilon = Units.Centimeters.of(3.5);
	public static final Distance kL2ScoringDistanceEpsilon = Units.Centimeters.of(3.0);
	public static final Distance kL3ScoringDistanceEpsilon = Units.Centimeters.of(3.0);
	public static final Distance kL4ScoringDistanceEpsilon = Units.Centimeters.of(3.0);
	public static final Distance kNetScoringDistanceEpsilon = Units.Centimeters.of(8.0);

    public static final Angle kReefScoringAngleEpsilon = Units.Degrees.of(0.8);
	public static final Angle kProcessorAlgaeScoringAngleEpsilon = Units.Degrees.of(3.0);
	public static final Angle kL1ScoringAngleEpsilon = Units.Degrees.of(2.0);
	public static final Angle kL2ScoringAngleEpsilon = Units.Degrees.of(1.2);
	public static final Angle kL3ScoringAngleEpsilon = Units.Degrees.of(1.2);
	public static final Angle kL4ScoringAngleEpsilon = Units.Degrees.of(1.2);
	public static final Angle kNetScoringAngleEpsilon = Units.Degrees.of(4.2);

    public static final Time kReefScoringDelay = Units.Milliseconds.of(40);
	public static final Time kProcessorAlgaeScoringDelay = Units.Millisecond.of(200);
	public static final Time kL1ScoringDelay = Units.Milliseconds.of(40);
	public static final Time kL2ScoringDelay = Units.Milliseconds.of(40);
	public static final Time kL3ScoringDelay = Units.Milliseconds.of(40);
	public static final Time kL4ScoringDelay = Units.Milliseconds.of(40);
	public static final Time kNetScoringDelay = Units.Milliseconds.of(40);

	public static final Time kReefLookaheadTime = Units.Seconds.of(0.16);
	public static final Time kProcessorAlgaeLookaheadTime = Units.Seconds.of(0.16);
	public static final Time kL1LookaheadTime = Units.Seconds.of(0.16);
	public static final Time kL2LookaheadTime = Units.Seconds.of(0.16);
	public static final Time kL3LookaheadTime = Units.Seconds.of(0.16);
	public static final Time kL4LookaheadTime = Units.Seconds.of(0.16);

	public static final Distance kElevatorCenterOffset = Units.Inches.of(12.5); // NEEDS TO BE TUNED

	// ALL OF THIS NEEDS TO BE TUNED
	public static final Distance kAlgaeOffsetFactor = Units.Centimeters.of(10.0);
	public static final Distance kAlgaeReadyOffsetFactor = Units.Centimeters.of(20.0);
	public static final Distance kL4CoralOffsetFactor = Units.Centimeters.of(34.25);
	public static final Distance kL3CoralOffsetFactor = Units.Centimeters.of(30.25);
	public static final Distance kL2CoralOffsetFactor = Units.Centimeters.of(30.25);
	public static final Distance kL1CoralOffsetFactor = Units.Centimeters.of(60.0);

	public static final Time kEndEffectorCoralDebounce = Units.Seconds.of(0.04);
	public static final AngularVelocity kEndEffectorVelocityDip = Units.DegreesPerSecond.of(2000);

    public static Distance getAutoAlignScoringDistanceEpsilon(Level level) {
		return switch (level) {
			case L1 -> kL1ScoringDistanceEpsilon;
			case L2 -> kL2ScoringDistanceEpsilon;
			case L3 -> kL3ScoringDistanceEpsilon;
			case L4 -> kL4ScoringDistanceEpsilon;
			case PROCESSOR_ALGAE -> kProcessorAlgaeScoringDistanceEpsilon;
			case NET -> kNetScoringDistanceEpsilon;
			default -> kReefScoringDistanceEpsilon;
		};
	}

	public static Angle getAutoAlignScoringAngleEpsilon(Level level) {
		return switch (level) {
			case L1 -> kL1ScoringAngleEpsilon;
			case L2 -> kL2ScoringAngleEpsilon;
			case L3 -> kL3ScoringAngleEpsilon;
			case L4 -> kL4ScoringAngleEpsilon;
			case PROCESSOR_ALGAE -> kProcessorAlgaeScoringAngleEpsilon;
			case NET -> kNetScoringAngleEpsilon;
			default -> kReefScoringAngleEpsilon;
		};
	}

	public static Time getAutoAlignScoringDelay(Level level) {
		return switch (level) {
			case L1 -> kL1ScoringDelay;
			case L2 -> kL2ScoringDelay;
			case L3 -> kL3ScoringDelay;
			case L4 -> kL4ScoringDelay;
			case PROCESSOR_ALGAE -> kProcessorAlgaeScoringDelay;
			case NET -> kNetScoringDelay;
			default -> kReefScoringDelay;
		};
	}

	public static Time getAutoAlignLookaheadTime(Level level) {
		return switch (level) {
			case L2 -> kL2LookaheadTime;
			case L3 -> kL3LookaheadTime;
			case L4 -> kL4LookaheadTime;
			case PROCESSOR_ALGAE -> kProcessorAlgaeLookaheadTime;
			default -> kReefLookaheadTime;
		};
	}

	public static Angle getAutoAlignHeadingGenerationDeadband(Level level) {
		return switch (level) {
			case L2 -> kL2HeadingGenerationDeadband;
			case L3 -> kL3HeadingGenerationDeadband;
			case L4 -> kL4HeadingGenerationDeadband;
			case PROCESSOR_ALGAE -> kProcessorAlgaeHeadingGenerationDeadband;
			default -> kReefHeadingGenerationDeadband;
		};
	}

	/**
	 * Gets the distance to offset scoring from the center of the robot based on the level you're trying to score at.
	 * Because the pivot is at different angles, the gamepiece ends up at a slightly different spot, so you need to compensate through different offsets.
	 *
	 * @param level The wanted level (L1, L2, L3, L4).
	 * @return The distance offset to add to the elevator offset to get where your drivetrain should be relative to a pose.
	 */
	public static Distance getGamepieceOffsetFactor(Level level) {
		return switch (level) {
			case L4 -> SuperstructureConstants.kL4CoralOffsetFactor;
			case L3 -> SuperstructureConstants.kL3CoralOffsetFactor;
			case L2 -> SuperstructureConstants.kL2CoralOffsetFactor;
			case L1 -> SuperstructureConstants.kL1CoralOffsetFactor;
			case L3_ALGAE, L2_ALGAE -> SuperstructureConstants.kAlgaeOffsetFactor;
			case ALGAE_READY -> SuperstructureConstants.kAlgaeReadyOffsetFactor;
			default -> kL4CoralOffsetFactor;
		};
	}
}
