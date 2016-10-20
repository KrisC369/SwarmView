/**
 *
 */
package applications.trajectory;

import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import choreo.Choreography;
import control.FiniteTrajectory4d;
import control.dto.Pose;

import java.util.ArrayList;

/**
 * @author tom
 */
public class NerveTrajectoryIntroduction implements FiniteTrajectory4d {
    private static final double ORIENTATION = -Math.PI / 2;
    private ArrayList<FiniteTrajectory4d> lineTrajectories = new ArrayList<>();
    private double duration;
    private FiniteTrajectory4d trajectory;

    public NerveTrajectoryIntroduction() throws Exception {
        double[][] path = {
                { 7, 6, 0.0, 4 },    // start position, with start time
                { 7, 6, 0.5, 1 },    // go into position
                { 7, 6, 0.5, 1 },
                { 2, 4, 1.5, 2.5 },
                { 5, 3, 4, 2 },
                { 3.5, 7, 2, 2 },
                { 1.5, 4.5, 2.5, 1.6 },
                { 6.5, 2, 1.5, 2.4 },
                { 5, 6, 1.5, 2.4 },
                { 2, 2, 4, 2.5 },
                { 2, 2, 4, 1 },
                { 2, 2, 1, 1.5 },
                { 2, 2, 4, 1.5 },
                { 2, 2, 4, 1 },
                { 2, 2, 2.5, 0.8 },
                { 2, 2, 4, 1 },
                { 2, 2, 4, 2 },
        };

        double startTime = path[0][3];
        Point3D endPosition = Point3D.create(path[0][0], path[0][1], path[0][2]);
        Point3D startPosition;
        double duration = 0;

        Choreography.Builder choreoBuilder = Choreography.builder();

        for (double[] lineInfo : path) {
            startTime += duration;
            duration = lineInfo[3];
            startPosition = endPosition;
            endPosition = Point3D.create(lineInfo[0], lineInfo[1], lineInfo[2]);

            if (startPosition.equals(endPosition)) {
                choreoBuilder.withTrajectory(Trajectories
                        .newHoldPositionTrajectory(Point4D.from(startPosition, ORIENTATION)))
                        .forTime(duration);
            } else {
                choreoBuilder.withTrajectory(Trajectories
                        .newStraightLineTrajectoryWithDuration(
                                Point4D.from(startPosition, ORIENTATION),
                                Point4D.from(endPosition, ORIENTATION), duration));
            }

        }
        this.duration = startTime + duration;
        this.trajectory = choreoBuilder.build();

    }

    @Override
    public double getTrajectoryDuration() {
        return duration;
    }

    @Override
    public Pose getDesiredPosition(double timeInSeconds) {
        return trajectory.getDesiredPosition(timeInSeconds);
    }
}
