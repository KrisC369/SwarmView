package io.github.agentwise.swarmview.trajectory.applications.trajectory;

import com.google.common.collect.Lists;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point3D;
import io.github.agentwise.swarmview.trajectory.applications.trajectory.geom.point.Point4D;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static io.github.agentwise.swarmview.trajectory.applications.trajectory.TestUtils.assertBounds;
import static java.lang.StrictMath.max;
import static java.lang.StrictMath.min;
import static org.junit.Assert.assertEquals;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class ShortSwingTrajectory4DTest {

  private Trajectory4d target;
  private Point4D begin = Point4D.create(2, -2, 4, -3);
  private Point3D end = Point3D.create(2.5, -4, 4);
  private double height = 0.5;
  private double frequency = 1 / 3d;

  @Before
  public void setUp() {
      //All of these construction calls are equivalent. Pick what's most convenient for you.
    target = ShortSwingTrajectory4D.create(begin, end, height, frequency);
    target = Trajectories.newShortPendulumSwingTrajectory(begin, end, height, frequency);
    target = Trajectories.shortSwingTrajectoryBuilder()
            .setBeginPoint(begin)
            .setEndPoint(end)
            .setDropHeight(height)
            .setFrequency(frequency)
            .build();
  }

  @Test
  public void getTrajectoryLinearXTestBounds() throws Exception {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < 1000; i++) {
      l.add(target.getDesiredPositionX(i / 10d));
    }
    assertBounds(l, min(end.getX(), begin.getX()), max(end.getX(), begin.getX()));
  }

  @Test
  public void getTrajectoryLinearYTestBounds() throws Exception {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < 1000; i++) {
      l.add(target.getDesiredPositionY(i / 10d));
    }
    assertBounds(l, min(begin.getY(), end.getY()), max(begin.getY(), end.getY()));
  }

  @Test
  public void getTrajectoryLinearZTestBounds() throws Exception {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < 1000; i++) {
      l.add(target.getDesiredPositionZ(i / 10d));
    }
    assertBounds(l, begin.getZ() - height, end.getZ());
  }

  @Test
  public void getTrajectoryAngularZTestBounds() throws Exception {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < 1000; i++) {
      l.add(target.getDesiredPositionZ(i / 10d));
    }
    assertBounds(l, 0, Math.PI * 2);
  }

  @Test
  public void getSimpleSwingOriginFromPointsAndHeight() throws Exception {
    Point4D begin = Point4D.create(2, 2, 4, -3);
    Point3D end = Point3D.create(2, 4, 4);
    double height = 0.5;
    double expectedR = 1.25d;
    Point4D origin =
        ShortSwingTrajectory4D.getSimpleSwingOriginFromPointsAndHeight(begin, end, height);
    assertEquals(2, origin.getX(), 0);
    assertEquals(3, origin.getY(), 0);
    assertEquals(4.3, origin.getZ(), 0);
    assertEquals(-3, origin.getAngle(), 0);
  }
}
