package frc.robot.subsystems.superstructure;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.FieldLayout;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoConstants;
import frc.robot.controlboard.ControlBoardConstants;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeployConstants;
import frc.robot.subsystems.intakeRollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.apriltag.Vision;

@Logged
public class Superstructure extends SubsystemBase {
    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Hood hood;
    private final IntakeDeploy intakeDeploy;
    private final IntakeRollers intakeRollers;
    private final Kicker kicker;
    private final Conveyor conveyor;

    public Superstructure(Drive drive, Vision vision, Shooter shooter, Hood hood, IntakeDeploy intakeDeploy, IntakeRollers intakeRollers, Kicker kicker, Conveyor conveyor) {
        this.drive = drive;
        this.vision = vision;
        this.shooter = shooter;
        this.hood = hood;
        this.intakeDeploy = intakeDeploy;
        this.intakeRollers = intakeRollers;
        this.kicker = kicker;
        this.conveyor = conveyor;
    }

    private boolean isPathFollowing = false;
    private boolean superstructureDone = false;
    private boolean driveReady = false;
    private boolean intakeDeployed = false;
    private boolean intakeBraked = false;
    public boolean shootOnTheMove = true;
    public boolean headingLockToggle = true;
    public boolean nearTrench = false;
    public boolean ignoreHubState = false;
    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeed, DriveConstants.kMaxAcceleration);


    public double maintainHeadingEpsilon = 0.25;

    private State state = State.TUCK;

    public Setpoint hoodSetpoint = Hood.ZERO;
    public Setpoint shooterSetpoint = Shooter.STOP;
    public Rotation2d headingSetpoint = new Rotation2d();


    public AngularVelocity shooterIncrement = Units.RPM.of(25.0);

    @Override
    public void periodic() {
        updateShooterSetpoint();
        updateHoodSetpoint();
        updateHeadingSetpoint();
    }

    public void updateShooterSetpoint() {
      //shooterSetpoint = Setpoint.withVelocitySetpoint(Units.RotationsPerSecond.of(Units.RPM.of(new TunableNumber("Shooter Vel", 1625.0, true).get()).in(Units.RotationsPerSecond)));
        shooterSetpoint = 
            Setpoint.withVelocitySetpoint(
              Units.RotationsPerSecond.of((Units.RPM.of(
              ShotCalculator.getInstance(drive)
              .getParameters()
              .flywheelSpeed()).plus(shooterIncrement)).in(Units.RotationsPerSecond)));
    }

    public void updateHoodSetpoint() {
    //hoodSetpoint = Setpoint.withPositionSetpoint(Units.Degrees.of(new TunableNumber("Hood Angle", 20.0, true).get()));
      nearTrench = FieldLayout.nearTrench(drive.getPose(), drive.getFieldRelativeChassisSpeeds());
      if (true /*&& !nearTrench*/) {
        hoodSetpoint = 
            Setpoint.withPositionSetpoint(
              Units.Degrees.of(
              ShotCalculator.getInstance(drive)
              .getParameters()
              .hoodAngle()));
      }
    }

    public void updateHeadingSetpoint() {
      boolean passing = !FieldLayout.distanceFromAllianceWall(Units.Meters.of(drive.getPose().getX()), RobotConstants.isRedAlliance).lte(FieldLayout.kAllianceZoneX.minus(Units.Inches.of(14)));
      if (!passing) {
        headingSetpoint = shootOnTheMove ? ShotCalculator.getInstance(drive).getParameters().heading() : ShotCalculator.getStationaryAimedPose(drive.getPose().getTranslation()).getRotation();
      }
      else {
        headingSetpoint = ShotCalculator.getInstance(drive).getParameters().heading();
      }
    }


    /**
     * stops intake rollers, conveyor, and kicker
     */
    public Command idleRollers() {
      return Commands.parallel(
                      intakeRollers.setpointCommand(IntakeRollers.IDLE),
                      conveyor.setpointCommand(Conveyor.IDLE),
                      kicker.setpointCommand(Kicker.IDLE)
              .withName("Idle Rollers")
      );
    }

    public Command zero() {
      return Commands.runOnce(() -> {
            intakeDeploy.setCurrentPosition(IntakeDeployConstants.kStowPosition);
            hood.setCurrentPosition(HoodConstants.kMinAngle);
          })
      .withName("Zero");
    }

    public Command spit() {
      return Commands.parallel(
            intakeDeploy.setpointCommand(IntakeDeploy.EXHAUST),
            intakeRollers.setpointCommand(IntakeRollers.EXHAUST),
            conveyor.setpointCommand(Conveyor.FEED_BACKWARDS),
            kicker.setpointCommand(Kicker.FEED_BACKWARDS),
            setState(State.SPIT),
            Commands.waitUntil(() -> false))
            .finallyDo(() -> {
                intakeRollers.applySetpoint(IntakeRollers.IDLE);
                conveyor.applySetpoint(Conveyor.IDLE);
                kicker.applySetpoint((Kicker.IDLE));
            })
            .withName("Spit");
    }

    public Command waitUntilSafeToShoot() {
      return Commands.waitUntil(() -> (shooter.spunUp() || Robot.isSimulation())
      && hood.nearPositionSetpoint()
      && (!headingLockToggle || atShotGoal() || RobotState.isAutonomous()));
    }

    public Command shoot() {
      return Commands.parallel(
        conveyor.setpointCommand(Conveyor.FEED_FORWARD),
        kicker.setpointCommand(Kicker.FEED_FORWARD)).
        withName("Shoot");
    }

    public Command turnToHubAuto() {
      return Commands.defer(() ->
      new PIDToPoseCommand(drive, this, new Pose2d(drive.getPose().getTranslation(), ShotCalculator.getInstance(drive).getParameters().heading()), Units.Inches.of(6.0), Units.Degrees.of(7)), Set.of(drive));
    }

    public Command idleIntake() {
      return intakeDeploy.setpointCommand(IntakeDeploy.IDLE);
    }

    public void brakeIntakeRollers(boolean wantsBrake) {
      if (intakeBraked != wantsBrake) {
        intakeBraked = !intakeBraked;
        intakeRollers.setNeutralBrake(wantsBrake);
      } else {return;}
    }

    public Command autoShootCommand() {
      return Commands.defer(() ->
        Commands.sequence(
          drive.stopDrivetrain(),
          turnToHubAuto().withTimeout(1.0),
          timeoutShootWhenReady()
        ), Set.of(drive, shooter)
      );
    }

    public Command juggle() {
      return Commands.parallel(
        shooter.setpointCommand(Shooter.JUGGLE),
        kicker.setpointCommand(Kicker.JUGGLE),
        conveyor.setpointCommand(Conveyor.JUGGLE));
    }

    public Command unjam() {
      return Commands.sequence(
        kicker.setpointCommand(Kicker.VELOCITY_BACKWARD),
        conveyor.setpointCommand(Conveyor.FEED_BACKWARDS),
        Commands.waitSeconds(0.2),
        kicker.setpointCommand(Kicker.IDLE),
        conveyor.setpointCommand(Conveyor.IDLE)
      );
    }

    public Command rampUpConveyorAndKicker() {
      return Commands.defer(() -> {
        final double feedRampSeconds = 0.00;
        final Timer timer = new Timer();
        final double rampSeconds = Math.max(feedRampSeconds, 0.02);

        return Commands.run(() -> {

          double rampPercent = Math.min(timer.get() / rampSeconds, 1.0);

          conveyor.applySetpoint(
              Setpoint.withVoltageSetpoint(Units.Volts.of(Conveyor.FEED_FORWARD.baseUnits * rampPercent)));
          kicker.applySetpoint(
              Setpoint.withVoltageSetpoint(Units.Volts.of(Kicker.FEED_FORWARD.baseUnits * rampPercent)));
        }, conveyor, kicker)
        .beforeStarting(timer::restart)
        .withTimeout(rampSeconds)
        .finallyDo(() -> timer.stop())
        .andThen(
            Commands.parallel(
                conveyor.setpointCommand(Conveyor.FEED_FORWARD),
                kicker.setpointCommand(Kicker.VELOCITY_FORWARD)));
      }, Set.of(conveyor, kicker)).withName("Ramp Up Conveyor + Kicker");
    }


    public Command shootWhenReadyPulse() {
      return Commands.sequence(
          Commands.runOnce(() -> maintainHeadingEpsilon = 0.00),
          Commands.parallel(
              shooter.followSetpointCommand(() -> shooterSetpoint),
              hood.followSetpointCommand(() -> hoodSetpoint),
              Commands.sequence(
              Commands.runOnce(() -> {
                  setStateInternal((state == State.INTAKING)
                          ? State.SHOOTINTAKE
                          : State.SHOOTING);
              }),
              waitUntilSafeToShoot(),
              Commands.runOnce(() -> setShootingGainProfile(true)),
              conveyor.setpointCommand(Conveyor.FEED_BACKWARDS),
              kicker.setpointCommandWithWait(Kicker.VELOCITY_FORWARD).withTimeout(0.2),
              Commands.parallel(
                conveyor.feedForwardOrPulseOnLowCurrent(),
                kicker.setpointCommand(Kicker.VELOCITY_FORWARD),
                intakeRollers.Pulse(),
                Commands.waitUntil(() -> false)))
      )).finallyDo(() -> {
          conveyor.applySetpoint(Conveyor.IDLE);
          kicker.applySetpoint(Kicker.FEED_BACKWARDS);
          shooter.applySetpoint(Shooter.IDLE);
          hood.applySetpoint(Hood.ZERO);
          maintainHeadingEpsilon = 0.25;
          setShootingGainProfile(false);
          setStateInternal((state == State.SHOOTINTAKE) ? State.INTAKING : State.DEPLOYED);
      });
    }

    public Command shootWhenReadyRise() {
      return Commands.sequence(
          Commands.runOnce(() -> maintainHeadingEpsilon = 0.00),
          Commands.parallel(
              shooter.followSetpointCommand(() -> shooterSetpoint),
              hood.followSetpointCommand(() -> hoodSetpoint),
              Commands.sequence(
              Commands.runOnce(() -> {
                  setStateInternal((state == State.INTAKING)
                          ? State.SHOOTINTAKE
                          : State.SHOOTING);
              }),
              waitUntilSafeToShoot(),
              Commands.runOnce(() -> setShootingGainProfile(true)),
              // conveyor.setpointCommand(Conveyor.FEED_BACKWARDS),
              // kicker.setpointCommandWithWait(Kicker.VELOCITY_FORWARD).withTimeout(0.2),
              Commands.parallel(
                conveyor.feedForwardOrPulseOnLowCurrent(),
                //conveyor.setpointCommand(Conveyor.FEED_FORWARD),
                kicker.setpointCommand(Kicker.VELOCITY_FORWARD),
                Commands.waitUntil(() -> isConveyorCurrentLowForRise()).withTimeout(1.0).andThen(intakeRise()),
                Commands.waitUntil(() -> false)))
      )).finallyDo(() -> {
          conveyor.applySetpoint(Conveyor.IDLE);
          kicker.applySetpoint(Kicker.FEED_BACKWARDS);
          shooter.applySetpoint(Shooter.IDLE);
          hood.applySetpoint(Hood.ZERO);
          intakeDeploy.applySetpoint(IntakeDeploy.DEPLOY);
          maintainHeadingEpsilon = 0.25;
          setShootingGainProfile(false);
          setStateInternal((state == State.SHOOTINTAKE) ? State.INTAKING : State.DEPLOYED);
      });
    }

    public Command shootWhenReadyPreset(AngularVelocity rpm, Angle angle) {
      return Commands.defer(() -> Commands.sequence(
          Commands.runOnce(() -> maintainHeadingEpsilon = 0.00),
          Commands.parallel(
              shooter.setpointCommand(Setpoint.withVelocitySetpoint(rpm.plus(shooterIncrement))),
              hood.setpointCommand(Setpoint.withPositionSetpoint(angle)),
              Commands.sequence(
              Commands.runOnce(() -> {
                  setStateInternal((state == State.INTAKING)
                          ? State.SHOOTINTAKE
                          : State.SHOOTING);
              }),
              waitUntilSafeToShoot(),
              Commands.runOnce(() -> setShootingGainProfile(true)),
              conveyor.setpointCommand(Conveyor.FEED_BACKWARDS),
              kicker.setpointCommandWithWait(Kicker.VELOCITY_FORWARD).withTimeout(0.2),
              Commands.parallel(
                conveyor.feedForwardOrPulseOnLowCurrent(),
                kicker.setpointCommand(Kicker.VELOCITY_FORWARD),
                intakeRollers.Pulse(),
                Commands.waitUntil(() -> false)))
      )).finallyDo(() -> {
          conveyor.applySetpoint(Conveyor.IDLE);
          kicker.applySetpoint(Kicker.FEED_BACKWARDS);
          shooter.applySetpoint(Shooter.IDLE);
          hood.applySetpoint(Hood.ZERO);
          maintainHeadingEpsilon = 0.25;
          setShootingGainProfile(false);
          setStateInternal((state == State.SHOOTINTAKE) ? State.INTAKING : State.DEPLOYED);
      }), Set.of(conveyor, kicker, shooter, hood, intakeRollers));
    }

    public Command shootAndIntake() {
      return Commands.sequence(
          Commands.runOnce(() -> maintainHeadingEpsilon = 0.00),
          Commands.parallel(
              shooter.followSetpointCommand(() -> shooterSetpoint),
              hood.followSetpointCommand(() -> hoodSetpoint),
              intakeRollers.setpointCommand(IntakeRollers.INTAKE),
              Commands.sequence(
              Commands.runOnce(() -> {
                  setStateInternal((state == State.INTAKING)
                          ? State.SHOOTINTAKE
                          : State.SHOOTING);
              }),
              waitUntilSafeToShoot(),
              Commands.runOnce(() -> setShootingGainProfile(true)),
              kicker.setpointCommandWithWait(Kicker.VELOCITY_FORWARD),
              Commands.parallel(
                conveyor.feedForwardOrPulseOnLowCurrent(),
                Commands.waitUntil(() -> false)))
      )).finallyDo(() -> {
          conveyor.applySetpoint(Conveyor.IDLE);
          kicker.applySetpoint(Kicker.FEED_BACKWARDS);
          shooter.applySetpoint(Shooter.IDLE);
          hood.applySetpoint(Hood.ZERO);
          intakeRollers.applySetpoint(IntakeRollers.IDLE);
          maintainHeadingEpsilon = 0.25;
          setShootingGainProfile(false);
          setStateInternal((state == State.SHOOTINTAKE) ? State.INTAKING : State.DEPLOYED);
      });
    }

    public Command deployIntake() {
      if (Robot.isReal()) {
        return Commands.sequence(intakeDeploy.setpointCommandWithWait(IntakeDeploy.DEPLOY), setIntakeStatus(true))
        .withName("Intake Deploy");
      }
      else
        return Commands.sequence(intakeDeploy.setpointCommand(IntakeDeploy.DEPLOY), setIntakeStatus(true))
        .withName("Intake Deploy");
    }

    public Command shakeIntake() {
      return Commands.sequence(
        intakeDeploy.setpointCommandWithWait(IntakeDeploy.SHAKE),
        Commands.waitSeconds(0.5),
        intakeDeploy.setpointCommand(IntakeDeploy.DEPLOY))
        .finallyDo(() -> intakeDeploy.applySetpoint(IntakeDeploy.IDLE));
    }

    public Command deployAndRunIntakeOtto() {
      return Commands.sequence(
        deployIntake(),
        Commands.runOnce(() -> brakeIntakeRollers(false)),
        runIntakeIfDeployed()
      );
    }

    public Command intakeRise() {
      return Commands.sequence(
        intakeDeploy.setMotionMagicConstraintsCommand(Units.RotationsPerSecond.of(0.8), IntakeDeployConstants.kDefaultAcceleration),
        intakeDeploy.setpointCommandWithWait(IntakeDeploy.SHAKE, Units.Amps.of(110)),
        intakeDeploy.setpointCommandWithWait(IntakeDeploy.DEPLOY, Units.Amps.of(110)),
        intakeDeploy.setMotionMagicConstraintsCommand(Units.RotationsPerSecond.of(0.2), IntakeDeployConstants.kDefaultAcceleration),
        intakeRollers.setpointCommand(IntakeRollers.INTAKE),
        intakeDeploy.setpointCommandWithWait(IntakeDeploy.RISE_UP, Units.Amps.of(110)),
        intakeRollers.setpointCommand(IntakeRollers.IDLE),
        intakeDeploy.setpointCommandWithWait(IntakeDeploy.FALL_DOWN, Units.Amps.of(110)),
        intakeRollers.setpointCommand(IntakeRollers.INTAKE),
        intakeDeploy.setpointCommandWithWait(IntakeDeploy.RISE_UP, Units.Amps.of(110)),
        intakeRollers.setpointCommand(IntakeRollers.IDLE),
        intakeDeploy.setpointCommandWithWait(IntakeDeploy.DEPLOY, Units.Amps.of(110)),
        intakeRollers.setpointCommand(IntakeRollers.INTAKE),
        Commands.waitSeconds(1.0)
      ).finallyDo(() -> {
        intakeDeploy.setMotionMagicConstraints(IntakeDeployConstants.kDefaultCruiseVelocity, IntakeDeployConstants.kDefaultAcceleration);
        intakeDeploy.applySetpoint(IntakeDeploy.DEPLOY);
        intakeRollers.applySetpoint(IntakeRollers.IDLE);});
    }

    public Command runIntakeIfDeployedJuggle() {
      return Commands.sequence(Commands.either(
          Commands.parallel(
            intakeRollers.setpointCommand(IntakeRollers.INTAKE),
            juggle()),
          Commands.sequence(
              deployIntake(),
              intakeRollers.setpointCommand(IntakeRollers.INTAKE)),
          () -> intakeDeployed),
          Commands.either(setState(State.SHOOTINTAKE), setState(State.INTAKING), () -> state == State.SHOOTING),
          Commands.waitUntil(() -> false))
          .withName("Intaking").finallyDo(() -> {
            intakeRollers.applySetpoint(IntakeRollers.IDLE);
            conveyor.applySetpoint(Conveyor.IDLE);
            kicker.applySetpoint(Kicker.IDLE);
            shooter.applySetpoint(Shooter.IDLE);
            setStateInternal((state == State.SHOOTINTAKE) ? State.SHOOTING : State.DEPLOYED);})
            .withName("End Intaking");
    }

    public Command runIntakeIfDeployed() {
      return Commands.sequence(Commands.either(
          Commands.parallel(
            intakeRollers.setpointCommand(IntakeRollers.INTAKE),
            conveyor.setpointCommand(Conveyor.FEED_BACKWARDS),
            kicker.setpointCommand(Kicker.FEED_BACKWARDS)),
          Commands.sequence(
              deployIntake(),
              intakeRollers.setpointCommand(IntakeRollers.INTAKE),
              conveyor.setpointCommand(Conveyor.FEED_BACKWARDS),
            kicker.setpointCommand(Kicker.FEED_BACKWARDS)),
          () -> intakeDeployed),
          Commands.either(setState(State.SHOOTINTAKE), setState(State.INTAKING), () -> state == State.SHOOTING),
          Commands.waitUntil(() -> false))
          .withName("Intaking").finallyDo(() -> {
            intakeRollers.applySetpoint(IntakeRollers.IDLE);
            conveyor.applySetpoint(Conveyor.IDLE);
            kicker.applySetpoint(Kicker.IDLE);
            setStateInternal((state == State.SHOOTINTAKE) ? State.SHOOTING : State.DEPLOYED);})
            .withName("End Intaking");
    }

    public Command tuck() {
      return Commands.sequence(
          intakeDeploy.setpointCommand(IntakeDeploy.STOW).onlyIf(() -> state != State.SHOOTING && state != State.SHOOTINTAKE),
          setIntakeStatus(false),
          setState(State.TUCK))
        .withName("Tuck");
    }

    public Command driveBrake() {
      return Commands.sequence(
        //Commands.waitUntil(() -> drive.getRotation().getMeasure().isNear(headingSetpoint.getMeasure(), Units.Degrees.of(5.0))),
        drive.brake()
      );
    }

    public boolean atShotGoal() {
      var passing = ShotCalculator.getInstance(drive).getParameters().passing();
      return DriverStation.isEnabled()
          && drive.getRotation().getMeasure().isNear(
            headingSetpoint.getMeasure(), 
            passing ? DriveConstants.driveYawPassToleranceDeg : DriveConstants.driveYawLaunchToleranceDeg);
    }

    public Command getAutoWaitCommand() {
      return Commands.defer(() ->
        Commands.waitSeconds(RobotContainer.autoDelay.getSelected() ? AutoConstants.delayTime : 0.0),
        Set.of(this)
      );
    }

    public DoubleSupplier getShootingTimeoutSeconds() {
        return () -> RobotContainer.autoShootAllFuelTime.getSelected();
    }

    public Command timeoutShootWhenReady() {
    return Commands.defer(() ->
        shootWhenReadyPulse()
            .raceWith(
                Commands.waitSeconds(getShootingTimeoutSeconds().getAsDouble())
            ),
			Set.of(this)
    	);
	}

    public Command timeoutShootWhenReadyRise() {
    return Commands.defer(() ->
        shootWhenReadyRise()
            .raceWith(
                Commands.waitSeconds(getShootingTimeoutSeconds().getAsDouble())
            ),
			Set.of(this)
    	);
	}

    public static enum State {
      TUCK,
      SHOOTING,
      INTAKING,
      SPIT,
      DEPLOYED,
      SHOOTINTAKE,
    }


    public boolean visionValid() {
      double time = vision.timeSinceLastTargetSeen();
      ChassisSpeeds speeds = drive.getRobotRelativeChassisSpeeds();
      double totalSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      
      if (totalSpeed < 1.0) {
        return time < 1.0;
      }
      else {
        return time < 0.5;
      }
    }

    private void setStateInternal(State nextState) {
      state = nextState;
    }

    private void setShootingGainProfile(boolean shooting) {
      //shooter.setShootingGains(shooting);
      kicker.setShootingGains(shooting);
    }
  
    public Command setState(State state) {
      return Commands.runOnce(() -> setStateInternal(state));
    }

    public Command setIntakeStatus(boolean status) {
      return Commands.runOnce(() -> intakeDeployed = status);
    }

    public Command toggleSOTM() {
      return Commands.runOnce(() -> shootOnTheMove = !shootOnTheMove);
    }
  
    public State getState() {
      return state;
    }

    public boolean isShootingState() {
      return state == State.SHOOTING || state == State.SHOOTINTAKE;
    }

    public boolean isConveyorCurrentLowForPulse() {
      return conveyor.isStatorCurrentLowForPulse();
    }

    public boolean isConveyorCurrentLowForRise() {
      return conveyor.isStatorCurrentLowForRise();
    }

    public boolean shouldHeadingLock() {
      return (headingLockToggle && (ControlBoardConstants.mDriverController.x().getAsBoolean() || ControlBoardConstants.mDriverController.b().getAsBoolean())  /*&& (!nearTrench|| state == State.SHOOTING). && (visionValid() || Robot.isSimulation())*/);
    }

    public void setPathFollowing(boolean isFollowing) {
		  isPathFollowing = isFollowing;
	  }

    public void setSuperstructureDone(boolean valToSet) {
		  superstructureDone = valToSet;
	  }

    public void setDriveReady(boolean valToSet) {
		  driveReady = valToSet;
	  }

    public boolean getSuperstructureDone() {
      return superstructureDone;
    }

    public Command commandToIntermediate(Drive drive, boolean isLeft) {
        Pose2d targetPose;
        
        if (isLeft) {
            targetPose = AutoConstants.leftIntermediate;
        } else {
            targetPose = AutoConstants.rightIntermediate;
        }
        
        PathConstraints constraints = new PathConstraints(
                DriveConstants.kMaxSpeed.in(Units.MetersPerSecond), DriveConstants.kMaxAcceleration.in(Units.MetersPerSecondPerSecond),
                DriveConstants.kMaxAngularRate.in(Units.RadiansPerSecond), DriveConstants.kMaxAngularAcceleration.in(Units.RadiansPerSecondPerSecond));

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                2
        );

        return pathfindingCommand;
    }

    public Command commandToShoot(Drive drive, boolean isLeft) {
        PathPlannerPath leftPathToShoot = null;
        try {
          if (isLeft)
            leftPathToShoot = PathPlannerPath.fromChoreoTrajectory("leftPathToShoot");
          else{
            leftPathToShoot = PathPlannerPath.fromChoreoTrajectory("rightPathToShoot");
          }
        } catch (Exception e) {
            e.printStackTrace();
        }

        PathConstraints constraints = new PathConstraints(
                DriveConstants.kMaxSpeed.in(Units.MetersPerSecond), DriveConstants.kMaxAcceleration.in(Units.MetersPerSecondPerSecond),
                DriveConstants.kMaxAngularRate.in(Units.RadiansPerSecond), DriveConstants.kMaxAngularAcceleration.in(Units.RadiansPerSecondPerSecond));

        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            leftPathToShoot,
            constraints);

        return pathfindingCommand;
    }

    public Command goToShootCommand(Drive drive) {
      if (FieldLayout.distanceFromAllianceWall(Units.Meters.of(drive.getPose().getX()), RobotConstants.isRedAlliance)
            .gte(FieldLayout.kAllianceZoneX.plus(Units.Inches.of(14)))) {
        boolean isLeft;
        if (drive.getLookaheadPose(Time.ofBaseUnits(.5, Units.Seconds)).getTranslation().getDistance(AutoConstants.leftShoot.getTranslation())
          < drive.getLookaheadPose(Time.ofBaseUnits(.5, Units.Seconds)).getTranslation().getDistance(AutoConstants.rightShoot.getTranslation())) {
            isLeft = true;
        } else {
            isLeft = false;
        }
        return Commands.sequence(
            commandToIntermediate(drive, isLeft),
            commandToShoot(drive, isLeft),
            drive.stopDrivetrain(),
            turnToHubAuto().withTimeout(1.0),
            timeoutShootWhenReady()
        );
    } else{
      return Commands.none();
    }
    }

}
