package control;

import java.util.ArrayList;
import java.util.List;

import control.dto.Pose;

/**
 * Defines the choreography (movements) of all drones in a complete dance show
 * 
 * @author Mario h.c.t.
 *
 */
public class Choreography {
	private double durationInSec;
	private int numberDrones;
	private final List<Act> acts = new ArrayList<>();;

	private Choreography() {};
	
	private Choreography(double durationInSec, int numberDrones) {
		this.durationInSec = durationInSec;
		this.numberDrones = numberDrones;
	}

	public static Choreography create(double durationInSec, int numberDrones) {
		Choreography choreo = new Choreography(durationInSec, numberDrones);
		
		return choreo;
	}
	
	public double getDurationInSec() {
		return durationInSec;
	}
	
	public void setDurationInSec(double durationInSec) {
		this.durationInSec = durationInSec;
	}
	
	public int getNumberDrones() {
		return numberDrones;
	}
	
	public void setNumberDrones(int numberDrones) {
		this.numberDrones = numberDrones;
	}
	
	public FiniteTrajectory4d getFullTrajectory(DroneName name) {
		//iterate over all acts, 
		//retrieve the trajectory per dronename
		
		double accumulatedTime = 0;
		for (Act act: acts) {
			accumulatedTime += act.getDuration();
		}
		final double choreographyDuration = accumulatedTime;
		final DroneName droneName = name;
		
		return new FiniteTrajectory4d() {
			private List<Act> show = acts;
			
			@Override
			public double getTrajectoryDuration() {
				return choreographyDuration;
			}
			
			@Override
			public Pose getDesiredPosition(double timeInSeconds) {
				double initialTimeAct = 0;
				for (Act act: show) {
					double finalTimeAct = act.getDuration() + initialTimeAct;
					
					if (initialTimeAct <= timeInSeconds && timeInSeconds <= finalTimeAct) {
						return act.getTrajectory(droneName).getDesiredPosition(timeInSeconds-initialTimeAct);
					}
					
					initialTimeAct = finalTimeAct;
				}
				
				// if could not find any trajectory, returns null
				// TODO raise RuntimeException, indicating the show does not last the ammount of timeInSeconds
				System.out.println("NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNOOOOOO\n\n\n\n");
				return null;
			}
		}; // end FiniteTrajectory4d implementation
	}
	
	
	public void addAct(Act act) {
		acts.add(act);
	}
}
