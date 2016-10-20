package choreo;

import applications.trajectory.BasicTrajectory;
import applications.trajectory.TrajectoryUtils;
import com.google.auto.value.AutoValue;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;
import com.google.common.collect.Queues;
import control.FiniteTrajectory4d;
import applications.trajectory.Trajectory4d;
import control.dto.Pose;

import java.util.List;
import java.util.Queue;

import static com.google.common.base.Preconditions.checkArgument;
import static org.slf4j.LoggerFactory.getLogger;

/**
 * A choreography represents a sequence of different trajectories to be executed for set durations.
 * The choreography is in itself a trajectory4d instance so it can be used as a single trajectory
 * from the lower level control point-of-view. Using the builder, one can create choreography
 * instances and configure them with different trajectories to be executed in sequence.
 * <p>A choreography can and should only be consumed once and cannot be reused.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public final class Choreography extends BasicTrajectory implements FiniteTrajectory4d {
    private final ImmutableList<ChoreoSegment> initialSegments;
    private final Queue<ChoreoSegment> segments;
    private double timeWindowShift;
    private boolean hasRun;

    private Choreography(List<ChoreoSegment> segmentsArg) {
        super();
        initialSegments = ImmutableList.copyOf(segmentsArg);
        segments = Queues.newArrayDeque(segmentsArg);
        timeWindowShift = 0d;
    }

    /**
     * @return A choreography builder instance.
     */
    public static Builder builder() {
        return new Builder();
    }

    private void checkChoreoSegments(double timeInSeconds) {
        logFirstTime();
        double normTime = normalize(timeInSeconds);
        if (normTime >= getCurrentSegment().getDuration()) {
            shiftSegments();
        }
    }

    private void logFirstTime() {
        if (!hasRun && getLogger(Choreography.class).isDebugEnabled()) {
            hasRun = true;
            getLogger(Choreography.class)
                    .debug("Executing first choreo segment: " + segments.peek());
        }
    }

    private void shiftSegments() {
        if (this.segments.size() > 1) {
            this.timeWindowShift += segments.poll().getDuration();
            if (getLogger(Choreography.class).isDebugEnabled()) {
                getLogger(Choreography.class)
                        .debug("Executing next choreo segment: " + segments.peek());
            }
        }
    }

    private double normalize(double timeInSeconds) {
        return timeInSeconds - timeWindowShift;
    }

    private ChoreoSegment getCurrentSegment() {
        return segments.peek();
    }

    @Override
    public Pose getDesiredPosition(double timeInSeconds) {
        setStartTime(timeInSeconds);
        final double currentTime = timeInSeconds - getStartTime();
        checkChoreoSegments(currentTime);
        return Pose.create(getCurrentSegment().getTarget().getDesiredPositionX(currentTime),
                getCurrentSegment().getTarget().getDesiredPositionY(currentTime),
                getCurrentSegment().getTarget().getDesiredPositionZ(currentTime),
                getCurrentSegment().getTarget().getDesiredAngleZ(currentTime));
    }

    @Override
    public String toString() {
        return "Choreography{" + "Choreo segments=" + initialSegments + '}';
    }

    @Override
    public double getTrajectoryDuration() {
        double totalDuration = 0;
        for (ChoreoSegment s : initialSegments) {
            totalDuration += s.getDuration();
        }
        return totalDuration;
    }

    /**
     * Step builder instance that can be built or further configured with trajectories.
     */
    public interface BuildableStepBuilder extends TimingRequiredStepBuilder {
        /**
         * @param trajectory The trajectory to add.
         * @return this builder instance.
         */
        BuildableStepBuilder withTrajectory(FiniteTrajectory4d trajectory);

        /**
         * @param trajectory The trajectory to add to the choreography.
         * @return A builder instance to specify the duration to execute given trajectory for.
         */
        TimingRequiredStepBuilder withTrajectory(Trajectory4d trajectory);

        /**
         * @return A fully built choreography instance.
         */
        Choreography build();
    }

    /**
     * Step builder instance for adding timing to specified trajectory.
     */
    public interface TimingRequiredStepBuilder {
        /**
         * @param duration the duration of the previously added trajectory.
         * @return A Builder instance.
         */
        BuildableStepBuilder forTime(double duration);

        /**
         * @param timeMark The timing mark in the future untill when to execute the current segment.
         * @return A Builder instance.
         */
        BuildableStepBuilder untillTime(double timeMark);
    }

    /**
     * A segment in the choreography specified by a target trajectory and a duration for which to
     * execute this trajectory.
     */
    @AutoValue
    abstract static class ChoreoSegment {
        /**
         * @return The trajectory to be executed in this segment.
         */
        public abstract Trajectory4d getTarget();

        /**
         * @return The duration this trajectory should be executed for.
         */
        public abstract double getDuration();
    }

    /**
     * Builder class for building choreography instances.
     */
    public static final class Builder implements BuildableStepBuilder {

        private final List<ChoreoSegment> segments;
        private Trajectory4d tempTarget;
        private double tempDuration;
        private boolean initial = true;

        private Builder() {
            this.segments = Lists.newArrayList();
        }

        @Override
        public TimingRequiredStepBuilder withTrajectory(Trajectory4d trajectory) {
            addSegmentWithDuration(tempTarget, tempDuration);
            this.tempTarget = trajectory;
            return this;
        }

        @Override
        public BuildableStepBuilder withTrajectory(FiniteTrajectory4d trajectory) {
            addSegmentWithDuration(tempTarget, tempDuration);
            this.tempTarget = TrajectoryUtils.createFrom(trajectory);
            this.tempDuration = trajectory.getTrajectoryDuration();
            return this;
        }

        @Override
        public Choreography build() {
            addSegmentWithDuration(tempTarget, tempDuration);
            return new Choreography(segments);
        }

        @Override
        public BuildableStepBuilder forTime(double duration) {
            checkArgument(duration > 0, "Duration should be > 0");
            this.tempDuration = duration;
            return this;
        }

        @Override
        public BuildableStepBuilder untillTime(double timeMark) {
            double diffUntillMark = calcDiff(timeMark);
            if (diffUntillMark < 0) {
                throw new IllegalArgumentException(
                        "Current choreo would already last longer than the given time to mark the"
                                + " end.");
            }
            return forTime(diffUntillMark);
        }

        private double calcDiff(double timeMark) {
            return timeMark - segments.stream().mapToDouble(cS -> cS.getDuration()).sum();
        }

        private boolean hasTempState() {
            return !initial;
        }

        private void reset() {
            this.initial = false;
        }

        private void addSegmentWithDuration(Trajectory4d target, double duration) {
            if (hasTempState()) {
                segments.add(new AutoValue_Choreography_ChoreoSegment(target, duration));
            }
            reset();
        }
    }
}