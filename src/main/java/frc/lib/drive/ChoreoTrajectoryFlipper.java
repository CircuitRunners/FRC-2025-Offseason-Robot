package frc.lib.drive;

import java.util.ArrayList;
import java.util.List;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import frc.lib.util.FieldLayout;

public final class ChoreoTrajectoryFlipper {
    private ChoreoTrajectoryFlipper() {}

    public static Trajectory<SwerveSample> mirrorLeftRight(Trajectory<SwerveSample> trajectory) {
        List<SwerveSample> samples = trajectory.samples();
        List<SwerveSample> flipped = new ArrayList<>(samples.size());
        for (SwerveSample sample : samples) {
            flipped.add(mirrorLeftRight(sample));
        }
        return new Trajectory<>(trajectory.name(), flipped, trajectory.splits(), trajectory.events());
    }

    public static SwerveSample mirrorLeftRight(SwerveSample sample) {
        double fieldWidthMeters = FieldLayout.kFieldWidth.in(Units.Meters);
        double mirroredY = fieldWidthMeters - sample.y;
        double mirroredHeading = -sample.heading;

        double[] fx = sample.moduleForcesX();
        double[] fy = sample.moduleForcesY();
        double[] mirroredFy = new double[fy.length];
        for (int i = 0; i < fy.length; i++) {
            mirroredFy[i] = -fy[i];
        }

        return new SwerveSample(
            sample.t,
            sample.x,
            mirroredY,
            mirroredHeading,
            sample.vx,
            -sample.vy,
            -sample.omega,
            sample.ax,
            -sample.ay,
            -sample.alpha,
            fx,
            mirroredFy
        );
    }
}
