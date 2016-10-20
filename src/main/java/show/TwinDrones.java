package show;

import applications.trajectory.Trajectories;
import applications.trajectory.Trajectory4d;
import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.Logger;
import choreo.Choreography;
import control.FiniteTrajectory4d;
import org.slf4j.LoggerFactory;

/**
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public abstract class TwinDrones {
    protected static final double orientation = -Math.PI / 2;
    protected final double startIntroAt = 30;
    protected final double operatingHeight = 1.5;
    protected final double endHeight = 1.5;
    protected final Point3D circleCenterPoint = Point3D.create(4, 4, operatingHeight);
    protected final double waitAtStation = 1.5;
    protected final double frequency = 1 / 2.32d; // ~max velocity achieved with f ~= 1/2.12 ~= 0.47
    protected final double revolutions = 3.5;
    protected final double circleTiming = (1 / frequency) * revolutions;
    protected final double circleRadius = 1;
    protected final double enterVelocity = 1;
    protected final double exitVelocity = 2;
    protected final double introEndTime = 15;

    TwinDrones() {
        //TODO remove this when logback config file is available.
        Logger root = (Logger) LoggerFactory.getLogger(Logger.ROOT_LOGGER_NAME);
        root.setLevel(Level.INFO);
    }

    protected abstract FiniteTrajectory4d getTrajectory();

    public static FiniteTrajectory4d createRomeoTrajectory() {
        return new Romeo().getTrajectory();
    }

    public static FiniteTrajectory4d createJulietTrajectory() {
        return new Juliet().getTrajectory();
    }

    public static class Romeo extends TwinDrones {
        private final Point4D takeOff = Point4D.create(7, 5, operatingHeight, orientation);
        private final Point4D endPoint = Point4D.create(1.1, 5, endHeight, orientation);
        private final Point4D wp1 = Point4D.create(5, 4, operatingHeight, orientation);
        private final Point4D wp2 = Point4D.create(3, 4, operatingHeight, orientation);
        private final double phaseToConnectStart = 0;

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            FiniteTrajectory4d firstLeg = Trajectories
                    .newStraightLineTrajectory(takeOff, wp1, enterVelocity);
            FiniteTrajectory4d lastLeg = Trajectories
                    .newStraightLineTrajectory(wp2, endPoint, exitVelocity);
            Trajectory4d circleTraj = Trajectories.circleTrajectoryBuilder().fixYawAt(orientation)
                    .setFrequency(frequency).setPhase(phaseToConnectStart).setRadius(circleRadius)
                    .setLocation(circleCenterPoint).build();

            Choreography choreo = Choreography.builder()
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(takeOff))
                    .forTime(startIntroAt).withTrajectory(firstLeg)
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(wp1))
                    .forTime(waitAtStation).withTrajectory(circleTraj).forTime(circleTiming)
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(wp2))
                    .forTime(waitAtStation).withTrajectory(lastLeg)
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(endPoint))
                    .untillTime(introEndTime + startIntroAt)
                    .build();

            LoggerFactory.getLogger(TwinDrones.class)
                    .info("Trajecory for Romeo takes " + choreo.getTrajectoryDuration()
                            + " seconds to perform fully.");
            return choreo;
        }
    }

    public static class Juliet extends TwinDrones {
        private final Point4D takeOff = Point4D.create(1, 5, operatingHeight, orientation);
        private final Point4D endPoint = Point4D.create(4.9, 5, endHeight, orientation);
        private final Point4D wp1 = Point4D.create(3, 4, operatingHeight, orientation);
        private final Point4D wp2 = Point4D.create(5, 4, operatingHeight, orientation);
        private final double phaseToConnectStart = Math.PI;

        @Override
        protected FiniteTrajectory4d getTrajectory() {
            FiniteTrajectory4d firstLeg = Trajectories
                    .newStraightLineTrajectory(takeOff, wp1, enterVelocity);
            FiniteTrajectory4d lastLeg = Trajectories
                    .newStraightLineTrajectory(wp2, endPoint, exitVelocity);
            Trajectory4d circleTraj = Trajectories.circleTrajectoryBuilder().fixYawAt(orientation)
                    .setFrequency(frequency).setPhase(phaseToConnectStart).setRadius(circleRadius)
                    .setLocation(circleCenterPoint).build();

            Choreography choreo = Choreography.builder()
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(takeOff))
                    .forTime(startIntroAt).withTrajectory(firstLeg)
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(wp1))
                    .forTime(waitAtStation).withTrajectory(circleTraj).forTime(circleTiming)
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(wp2))
                    .forTime(waitAtStation).withTrajectory(lastLeg)
                    .withTrajectory(Trajectories.newHoldPositionTrajectory(endPoint))
                    .untillTime(introEndTime + startIntroAt)
                    .build();

            LoggerFactory.getLogger(TwinDrones.class)
                    .info("Trajecory for Juliet takes " + choreo.getTrajectoryDuration()
                            + " seconds to perform fully.");
            return choreo;
        }
    }
}
