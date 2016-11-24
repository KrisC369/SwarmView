/**
 *
 */
package io.github.agentwise.swarmview.trajectory.rats.acts.introduction;

import io.github.agentwise.swarmview.trajectory.applications.trajectory.Hover;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.StraightLineTrajectory4D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.composites
        .TrajectoryComposite;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import io.github.agentwise.swarmview.trajectory.control.FiniteTrajectory4d;
import io.github.agentwise.swarmview.trajectory.control.dto.Pose;

/**
 * @author tom
 */
public class NerveTrajectoryIntroduction implements FiniteTrajectory4d {

    private double duration;
    private FiniteTrajectory4d trajectory;

    public NerveTrajectoryIntroduction(Pose initialPosition, Pose finalPosition, double start) {
        double[][] path = {
                { initialPosition.x(), initialPosition.y(), initialPosition.z(), 0 },
                // start position, start time is ignored
                { 7, 5, 1.5, 0.3 },
                { 4, 5, 2, 1 },
                { 2, 5, 2, 1 },
                { 3, 6, 1.5, 1 },
                { 2, 6, 1.5, 0.3 },
                { 3, 6, 1.5, 1 },
                { 2, 5, 2.5, 0.9 },
                { 4, 5, 1.5, 1 },
                { 3, 5, 2.5, 0.5 },
                { 4, 5, 1.5, 1 },
                { 3.5, 5, 1.5, 1 },
                { 4.5, 5, 1.5, 1 },
                { 4, 5, 1.5, 0.6 },
                { 3.5, 5, 1.5, 0.5 },
                { 4.5, 5, 1.5, 1 },
                { 2, 2, 4, 0.8 },
                { 2, 2, 2.5, 0.5 },
                { 2, 2, 4, 1 },
                { 2, 2, 2.5, 1 },
                { finalPosition.x(), finalPosition.y(), finalPosition.z(), 1 },
        };

        double startTime = 0;
        Point4D endPosition = Point4D.create(path[0][0], path[0][1], path[0][2], 0);
        Point4D startPosition;
        double duration = start;

        TrajectoryComposite.Builder trajectoryBuilder = TrajectoryComposite.builder();

        if (start > 0) {
            Hover hoverBeforeBegin = new Hover(initialPosition, start);
            trajectoryBuilder.addTrajectory(hoverBeforeBegin);
        }

        StraightLineTrajectory4D line;

        for (int i = 1; i < path.length; i++) {
            double[] lineInfo = path[i];
            startTime += duration;
            duration = lineInfo[3];
            startPosition = endPosition;
            endPosition = Point4D.create(lineInfo[0], lineInfo[1], lineInfo[2], 0);
            line = StraightLineTrajectory4D
                    .createWithPercentageVelocity(startPosition, endPosition, duration);
            trajectoryBuilder.addTrajectory(line);
        }

        this.duration = startTime;
        this.trajectory = trajectoryBuilder.build();

    }

    @Override
    public double getTrajectoryDuration() {
        return this.duration;
    }

    @Override
    public Pose getDesiredPosition(double timeInSeconds) {
        return this.trajectory.getDesiredPosition(timeInSeconds);
    }

}

