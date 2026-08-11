package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.autos.centerPreload.CenterPreload;
import frc.robot.auto.autos.centerPreload.CenterPreloadDepot;
import frc.robot.auto.autos.disruption.FullDisruptionOpp;
import frc.robot.auto.autos.disruption.FullDisruptionSame;
import frc.robot.auto.autos.doubleSwipe.Bean;
import frc.robot.auto.autos.doubleSwipe.DelayTestAuto;
import frc.robot.auto.autos.doubleSwipe.DoubleNeutral;
import frc.robot.auto.autos.doubleSwipe.DoubleNeutralRisky;
import frc.robot.auto.autos.doubleSwipe.DoubleNeutralSilly;
import frc.robot.auto.autos.doubleSwipe.DoubleNeutralStraight;
import frc.robot.auto.autos.doubleSwipe.Follower;
import frc.robot.auto.autos.doubleSwipe.GreedyBean;
import frc.robot.auto.autos.doubleSwipe.PIDTestAuto;
import frc.robot.auto.autos.singleSwipe.Neutral;
import frc.robot.auto.autos.singleSwipe.Popcorn;
import frc.robot.auto.autos.superSilly.DoubleSilly;
import frc.robot.auto.autos.superSilly.Fakeout;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
public class AutoModeSelector {
	private AutoChooser mAutoChooser = new AutoChooser();	

	public AutoModeSelector(Drive drive, Superstructure superstructure, AutoFactory factory) {
		mAutoChooser.addRoutine("[RIGHT] Silly", () -> new DoubleNeutralSilly(drive, superstructure, factory, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Silly", () -> new DoubleNeutralSilly(drive, superstructure, factory, false).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Risky", () -> new DoubleNeutralRisky(drive, superstructure, factory, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Risky", () -> new DoubleNeutralRisky(drive, superstructure, factory, false).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Close Single Neutral", () -> new Neutral(drive, superstructure, factory, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Close Single Neutral", () -> new Neutral(drive, superstructure, factory, false).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Double Silly", () -> new DoubleSilly(drive, superstructure, factory, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Double Silly", () -> new DoubleSilly(drive, superstructure, factory, false).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Straight, Not Angled", () -> new DoubleNeutralStraight(drive, superstructure, factory, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Straight, Not Angled", () -> new DoubleNeutralStraight(drive, superstructure, factory, false).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Double Neutral Linear", () -> new DoubleNeutral(drive, superstructure, factory, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Double Neutral Linear", () -> new DoubleNeutral(drive, superstructure, factory, false).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Fakeout", () -> new Fakeout(drive, superstructure, factory, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Fakeout", () -> new Fakeout(drive, superstructure, factory, false).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Opposite Disruption", () -> new FullDisruptionOpp(drive, superstructure, factory, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Opposite Disruption", () -> new FullDisruptionOpp(drive, superstructure, factory, false).getRoutine());
		mAutoChooser.addRoutine("[RIGHT] Same Disruption", () -> new FullDisruptionSame(drive, superstructure, factory, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Same Disruption", () -> new FullDisruptionSame(drive, superstructure, factory, false).getRoutine());

		mAutoChooser.addRoutine("[CENTER] Center Preload", () -> new CenterPreload(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[CENTER] Depot", () -> new CenterPreloadDepot(drive, superstructure, factory).getRoutine());


		mAutoChooser.addRoutine("[LEFT] BEAN", () -> new Bean(drive, superstructure, factory, false, false).getRoutine());
		mAutoChooser.addRoutine("[RIGHT] BEAN", () -> new Bean(drive, superstructure, factory, true, false).getRoutine());
		mAutoChooser.addRoutine("[LEFT] BEAN Greedy", () -> new GreedyBean(drive, superstructure, factory, false).getRoutine());
		mAutoChooser.addRoutine("[RIGHT] BEAN Greedy", () -> new GreedyBean(drive, superstructure, factory, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] BEAN Follower", () -> new Bean(drive, superstructure, factory, false, true).getRoutine());
		mAutoChooser.addRoutine("[LEFT] BEAN Follower", () -> new Bean(drive, superstructure, factory, true, true).getRoutine());

		mAutoChooser.addRoutine("[LEFT] Follower", () -> new Follower(drive, superstructure, factory, false).getRoutine());
		mAutoChooser.addRoutine("[RIGHT] Follower", () -> new Follower(drive, superstructure, factory, true).getRoutine());
    
		// mAutoChooser.addRoutine("Delay Test Auto", () -> new DelayTestAuto(drive, superstructure, factory, false).getRoutine());
		// mAutoChooser.addRoutine("PID Test Auto", () -> new PIDTestAuto(drive, superstructure, factory, false).getRoutine());

	}

	public Command getSelectedCommand() {
		return mAutoChooser.selectedCommandScheduler();
	}

	public AutoChooser getAutoChooser() {
		return mAutoChooser;
	}

}