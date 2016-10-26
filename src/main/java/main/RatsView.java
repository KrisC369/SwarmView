package main;

import java.awt.MouseInfo;
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;

import control.Act;
import control.ActConfiguration;
import control.Choreography;
import static control.DroneName.*;
import control.DronePositionConfiguration;
import control.dto.Pose;
import processing.core.PApplet;
import processing.event.KeyEvent;
import processing.event.MouseEvent;
import rats.acts.attack.AttackAct;
import rats.acts.chaos.ChaosAct;
import rats.acts.introduction.IntroductionAct;
import rats.acts.taming.TamingAct;

public class RatsView extends PApplet {
	private static final float MAX_ZOOM = 4.0f;
	private static final float MIN_ZOOM = 0.3f;
	DroneView[] drones;
	int rotzfactor = 0;
	float zoom = 1.0f;
	final int displayDimensionX = 1024;
	final int displayDimensionY = 800;
	private boolean mouseActive = true;
	private boolean timerActive = true;
	private boolean simulationActive = true;
	
	private int lastMouseX;
	private int lastMouseY;
	private float lastZoom;
	private int initialTime = 0;
	private float rotz;
	private int lastTimeStep;
	
	Choreography choreo;
	
	public static void main(String[] args) {
		PApplet.main(RatsView.class);
	}
	
	@Override
	public void settings() {
//		fullScreen();
		size(displayDimensionX, displayDimensionY, P3D);
	}
	
	@Override
	public void setup() {
		fill(255);
		
		initializeTrajectories();
	}
	
	private void initializeTrajectories() {
		initialTime = 0; //TODO add separate method to reset the view parameters

		//
		//Specification of initial drone positions for Introduction
		//
		List<DronePositionConfiguration> introPositions = new ArrayList<>();
		introPositions.add(DronePositionConfiguration.create(Nerve, Pose.create(7.0, 6.0, 1.0, 0.0),  Pose.create(2.0, 2.0, 4.0, 0.0)));
		introPositions.add(DronePositionConfiguration.create(Romeo, Pose.create(7.0, 5.0, 1.0, 0.0),  Pose.create(1.1, 5.0, 1.5, 0.0)));
		introPositions.add(DronePositionConfiguration.create(Juliet, Pose.create(1.0, 5.0, 1.0, 0.0), Pose.create(4.9, 5.0, 1.5, 0.0)));
		introPositions.add(DronePositionConfiguration.create(Fievel, Pose.create(1.0, 6.0, 1.0, 0.0), Pose.create(5.0, 2.5, 1.0, 0.0)));
		introPositions.add(DronePositionConfiguration.create(Dumbo, Pose.create(2.0, 3.0, 1.0, 0.0),  Pose.create(4.0, 3.5, 2.5, 0.0)));
		ActConfiguration introConfiguration = ActConfiguration.create(50, introPositions);
		Act introduction = IntroductionAct.create(introConfiguration);

		//
		//Specification of initial drone positions for Chaos
		//
		List<DronePositionConfiguration> chaosPositions = new ArrayList<>();
		chaosPositions.add(DronePositionConfiguration.create(Nerve,  introduction.finalPosition(Nerve),  Pose.create(6.0, 6.0, 2.0, 0.0)));
		chaosPositions.add(DronePositionConfiguration.create(Romeo,  introduction.finalPosition(Romeo),  Pose.create(3.5, 4.0, 1.0, 0.0)));
		chaosPositions.add(DronePositionConfiguration.create(Juliet, introduction.finalPosition(Juliet), Pose.create(1.0, 1.0, 2.5, 0.0)));
		chaosPositions.add(DronePositionConfiguration.create(Fievel, introduction.finalPosition(Fievel), Pose.create(2.0, 5.0, 2.0, 0.0)));
		chaosPositions.add(DronePositionConfiguration.create(Dumbo,  introduction.finalPosition(Dumbo),  Pose.create(1.5, 3.0, 1.0, 0.0)));
		ActConfiguration chaosConfiguration = ActConfiguration.create(5, chaosPositions);
		Act chaos = ChaosAct.create(chaosConfiguration);

		//
		//Specification of initial drone positions for Attack
		//
		List<DronePositionConfiguration> attackPositions = new ArrayList<>();
		attackPositions.add(DronePositionConfiguration.create(Nerve,  chaos.finalPosition(Nerve),  Pose.create(4.5, 3.0, 2.0, 0.0)));
		attackPositions.add(DronePositionConfiguration.create(Romeo,  chaos.finalPosition(Romeo),  Pose.create(3.5, 3.0, 2.5, 0.0)));
		attackPositions.add(DronePositionConfiguration.create(Juliet, chaos.finalPosition(Juliet), Pose.create(2.0, 6.0, 2.0, 0.0)));
		attackPositions.add(DronePositionConfiguration.create(Fievel, chaos.finalPosition(Fievel), Pose.create(5.0, 5.5, 2.5, 0.0)));
		attackPositions.add(DronePositionConfiguration.create(Dumbo,  chaos.finalPosition(Dumbo),  Pose.create(3.0, 6.1, 1.0, 0.0)));
		ActConfiguration attackConfiguration = ActConfiguration.create(5, attackPositions);
		Act attack = AttackAct.create(attackConfiguration);

		//
		//Specification of initial drone positions for Taming
		//
		List<DronePositionConfiguration> tamingPositions = new ArrayList<>();
		tamingPositions.add(DronePositionConfiguration.create(Nerve,  attack.finalPosition(Nerve),  Pose.create(2.0, 2.0, 1.5, 0.0)));
		tamingPositions.add(DronePositionConfiguration.create(Romeo,  attack.finalPosition(Romeo),  Pose.create(3.0, 3.0, 1.5, 0.0)));
		tamingPositions.add(DronePositionConfiguration.create(Juliet, attack.finalPosition(Juliet), Pose.create(4.0, 4.0, 1.5, 0.0)));
		tamingPositions.add(DronePositionConfiguration.create(Fievel, attack.finalPosition(Fievel), Pose.create(5.0, 5.0, 1.5, 0.0)));
		tamingPositions.add(DronePositionConfiguration.create(Dumbo,  attack.finalPosition(Dumbo),  Pose.create(6.0, 6.0, 1.5, 0.0)));
		ActConfiguration tamingConfiguration = ActConfiguration.create(5, tamingPositions);
		Act taming = TamingAct.create(tamingConfiguration);		
		
		//
		// Configures the whole Choreography
		//
		choreo = Choreography.create(70, 5);
		choreo.addAct(introduction);
		choreo.addAct(chaos);
		choreo.addAct(attack);
		choreo.addAct(taming);
		
		//
		//Configures the view
		//
		drones = new DroneView[choreo.getNumberDrones()];

		drones[0] = new DroneView(this, choreo.getFullTrajectory(Nerve), color(0, 244, 200), 1);
        drones[1] = new DroneView(this, choreo.getFullTrajectory(Romeo), color(200, 100, 10), 50);
        drones[2] = new DroneView(this, choreo.getFullTrajectory(Juliet), color(200, 0, 200), 50);
        drones[3] = new DroneView(this, choreo.getFullTrajectory(Fievel), color(255, 255, 200), 50);
        drones[4] = new DroneView(this, choreo.getFullTrajectory(Dumbo), color(120, 255, 250), 50);
		
        /**
         * Safety checks for collision between drones
         */
        //		OfflineMinimumDistanceCheckers.checkMinimum3dDistanceConstraint(trajectories, 1.0);
    }
	
	@Override
	public void draw() {
		background(0);
		
		Point mouse = MouseInfo.getPointerInfo().getLocation();
		
		if (mouseIsActive()) {
			lastMouseX = mouse.x;
			lastMouseY = mouse.y;
			lastZoom = zoom;
		}
		
		positionView(lastMouseX, lastMouseY, lastZoom);
		
		
		pushMatrix();
		strokeWeight(2.0f);
		translate(0, 0, 200);
		box(800, 800, 400);
		popMatrix();

		drawStage();
		
		pushMatrix();
		translate(-400, -400, 0);
		text("Origin", 0.0f, 0.0f, 0.0f);
		
		int timeStep;
		if (simulationIsActive()) {
			int currentTime = millis();
			if (initialTime  == 0) {
				initialTime = millis();
			}
			
			timeStep = currentTime-initialTime;
			lastTimeStep = timeStep;
		} else {
			timeStep = lastTimeStep;
		}
		
		if (timerIsActive()) {
			drawTimer(timeStep);
		}
		for (int i=0; i<choreo.getNumberDrones(); i++) {
			drones[i].displayNext((timeStep)/1000.0f);
		}
		popMatrix();
		
		popMatrix();
	}

	/**
	 * Positions the Scene view according to the last mouse movement
	 * 
	 */
	private void positionView(float x, float y, float zoom) {
		zoom = constrain(zoom, MIN_ZOOM, MAX_ZOOM);
		scale(zoom);
	
		/**
		 * Rotation of the screen
		 */
		float rotx = (2*x/(float) displayDimensionX)*-2*PI+PI;
		float roty = (2*y/(float) displayDimensionY)*-2*PI-PI;
		rotz = rotzfactor*PI/36;
	
		pushMatrix();
		translate(width/(2*zoom), height/(2*zoom), rotz/zoom);
		rotateX(roty);  
		rotateZ(rotx);  
	}
	
	public void mouseWheel(MouseEvent event) {
		if(event.getCount() >= 0 && zoom <= MAX_ZOOM) { 
		    zoom += 0.01;
		  } 
		  else if (zoom >= MIN_ZOOM) {
		    zoom -= 0.01; 
		  }
	}
	
	/**
	 * Handles user input via the keyboard
	 */
	@Override
	public void keyPressed(KeyEvent event) {
		if (event.getKey() == 'z') {
			 mouseToggle();
		}
		
		if (event.getKey() == 't') {
			timerToggle();
		}
		
		if (event.getKey() == ' ') {
			pauseToggle();
		}
		
		if (event.getKey() == 'r') {
			initializeTrajectories();
		}
	}
	
	/**
	 * Activates or deactivates listening to mouse events
	 */
	private void mouseToggle() {
		if (mouseActive == true) {
			mouseActive = false;
		} else {
			mouseActive = true;
		}
	}
	
	/**
	 * Activates/deactivates the timer
	 */
	private void timerToggle() {
		if (timerActive  == true) {
			timerActive = false;
		} else {
			timerActive = true;
		}
	}
	
	/**
	 * Activates/deactivates the simulation
	 */
	private void pauseToggle() {
		if (simulationActive == true) {
			simulationActive = false;
		} else {
			simulationActive = true;
		}
	}
	
	private boolean mouseIsActive() {
		return mouseActive;
	}
	
	private boolean timerIsActive() {
		return timerActive;
	}
	
	private boolean simulationIsActive() {
		return simulationActive;
	}
	/**
	 * Draws timer
	 * @param time in milliseconds
	 */
	public void drawTimer(double time) {
		pushMatrix();
		rotateX(-PI/2);
		fill(255);
		textSize(72);
		
		int seconds = (int) (time / 1000) % 60 ;
		int minutes = (int) ((time / (1000*60)) % 60);
		int milliseconds = (int) (time % 1000);
		text(String.format("%02d' %02d\" %03d", minutes, seconds, milliseconds), -100.0f, -450.0f, 0.0f);
		popMatrix();
	}
	
	/**
	 * Draws the stage on the screen
	 */
	public void drawStage() {
		pushMatrix();
		scale(700, 700, 700);
		
		// Room floor
		noStroke();
		beginShape(QUADS);
		fill(255, 255, 0, 100);
		vertex(-1, -1,  0);
		vertex( 1, -1,  0);
		vertex( 1,  1,  0);
		vertex(-1,  1,  0);
		endShape();
		
		// Room back (x,z) plane
		noStroke();
		beginShape(TRIANGLE);
		fill(255, 0, 255, 100);
		vertex(-1, -1,  0);
		vertex( 0, -1,  1);
		vertex( 1,  -1,  0);
		endShape();
		
		// TODO draw markers for the x,y,z coordinates on the corners
		
		popMatrix();
	}
}
