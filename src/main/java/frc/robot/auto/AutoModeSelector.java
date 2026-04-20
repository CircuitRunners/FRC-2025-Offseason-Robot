package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.autos.centerPreload.CenterPreload;
import frc.robot.auto.autos.disruption.LeftFullDisruptionOpp;
import frc.robot.auto.autos.disruption.LeftFullDisruptionSame;
import frc.robot.auto.autos.disruption.RightFullDisruptionOpp;
import frc.robot.auto.autos.disruption.RightFullDisruptionSame;
import frc.robot.auto.autos.doubleSwipe.LeftDoubleNeutral;
import frc.robot.auto.autos.doubleSwipe.LeftDoubleNeutralRisky;
import frc.robot.auto.autos.doubleSwipe.LeftDoubleNeutralSilly;
import frc.robot.auto.autos.doubleSwipe.LeftDoubleNeutralStraight;
import frc.robot.auto.autos.doubleSwipe.RightDoubleNeutral;
import frc.robot.auto.autos.doubleSwipe.RightDoubleNeutralRisky;
import frc.robot.auto.autos.doubleSwipe.RightDoubleNeutralSilly;
import frc.robot.auto.autos.doubleSwipe.RightDoubleNeutralStraight;
import frc.robot.auto.autos.singleSwipe.LeftNeutral;
import frc.robot.auto.autos.singleSwipe.Popcorn;
import frc.robot.auto.autos.singleSwipe.RightNeutral;
import frc.robot.auto.autos.superSilly.LeftDoubleSilly;
import frc.robot.auto.autos.superSilly.LeftFakeout;
import frc.robot.auto.autos.superSilly.RightDoubleSilly;
import frc.robot.auto.autos.superSilly.RightFakeout;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
public class AutoModeSelector {
	private AutoChooser mAutoChooser = new AutoChooser();	

	public AutoModeSelector(Drive drive, Superstructure superstructure, AutoFactory factory) {
		mAutoChooser.addRoutine("[RIGHT] Silly", () -> new RightDoubleNeutralSilly(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Silly", () -> new LeftDoubleNeutralSilly(drive, superstructure, factory).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Risky", () -> new RightDoubleNeutralRisky(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Risky", () -> new LeftDoubleNeutralRisky(drive, superstructure, factory).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Close Single Neutral", () -> new RightNeutral(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Close Single Neutral", () -> new LeftNeutral(drive, superstructure, factory).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Double Silly", () -> new RightDoubleSilly(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Double Silly", () -> new LeftDoubleSilly(drive, superstructure, factory).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Straight, Not Angled", () -> new RightDoubleNeutralStraight(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Straight, Not Angled", () -> new LeftDoubleNeutralStraight(drive, superstructure, factory).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Double Neutral Linear", () -> new RightDoubleNeutral(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Double Neutral Linear", () -> new LeftDoubleNeutral(drive, superstructure, factory).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Fakeout", () -> new RightFakeout(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Fakeout", () -> new LeftFakeout(drive, superstructure, factory).getRoutine());

		mAutoChooser.addRoutine("[RIGHT] Opposite Disruption", () -> new RightFullDisruptionOpp(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Opposite Disruption", () -> new LeftFullDisruptionOpp(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[RIGHT] Same Disruption", () -> new RightFullDisruptionSame(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("[LEFT] Same Disruption", () -> new LeftFullDisruptionSame(drive, superstructure, factory).getRoutine());

		mAutoChooser.addRoutine("[CENTER] Center Preload", () -> new CenterPreload(drive, superstructure, factory).getRoutine());


		//mAutoChooser.addRoutine("Popcorn Penguins Steal", () -> new Popcorn(drive, superstructure, factory).getRoutine());
    }

	public Command getSelectedCommand() {
		return mAutoChooser.selectedCommandScheduler();
	}

	public AutoChooser getAutoChooser() {
		return mAutoChooser;
	}

}