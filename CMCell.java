/**
 *  Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * 
 * Package: cellModel
 * File: CMCell.java
 * Apr 10, 2013 1:44:54 PM
 */
package cellModel;

import java.util.Random;
import java.util.Arrays;

import javax.vecmath.Vector3f;

import org.lwjgl.util.glu.Sphere;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;


public class CMCell implements CMBioObj{
	final private static int NO_RESPONSE = 1, UNIFORM_RESPONSE = 2, DIFFERENTIAL_RESPONSE = 3;
	private static float radius = 5.0f;
	private static float density = 1.2f;
	private static float volume = (float)(4.0/3.0 * Math.PI * radius * radius * radius);
	private static float mass = density * volume;
	private static float maxVelChange = 0.3f;
	private static int cell_ids = 0;
	private int molResponse = NO_RESPONSE;
	private int id;
	private Vector3f origin;
	private static SphereShape cellShape = new SphereShape(radius);
	private CMRigidBody body;
	private Transform trans;
	private float cellFriction = .3f;
	private static float[] baseCellColor = {1.0f, 0.7f, 0.7f};
	private float[] cellColor;
	protected float cameraDistance = 20f;
	private boolean visible = true;
	private boolean toRemove = false;
	private CMSimulation sim;
	private static float responseDeltaVel = 0.2f;
	private String objectType = "Cell";
	
	private int verSegments = 3; //Number of vertical segments the sphere will be divided into for response to molecules
	private int horSegments = verSegments * 2;
	private int numSegments = horSegments * verSegments;
	private int[] diffProbs;
	private int baseProb = 40; //% probability that molecule will bind
	private int currentProb = baseProb; //For Uniform response, the probability that molecule will bind to the whole cell
	private int deltaProb = 10;
	private float[] cumProbs; //Will hold cumulative probabilities for direction of motion
	private float verDegPerSeg = (float)(180.0/verSegments), horDegPerSeg=(float)(360.0/horSegments);
	
	public CMCell(CMSimulation s, Vector3f o){
		this.origin = new Vector3f(o);
		this.sim = s;
		
		trans = new Transform();
		trans.setIdentity();
		trans.origin.set(origin);
		
		Vector3f localInertia = new Vector3f(0, 0, 0);
		cellShape.calculateLocalInertia(mass, localInertia);
		//System.out.println(localInertia);

		DefaultMotionState motionState = new DefaultMotionState(trans);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, cellShape, localInertia);
		rbInfo.friction = cellFriction;
		body = new CMRigidBody(rbInfo, this);
		
		this.id = cell_ids;
		cell_ids++;
		
		cellColor = new float[3];
		for (int i = 0; i < 3; i++){
			cellColor[i] = (float)(baseProb/100.0) * baseCellColor[i];
		}
		
		diffProbs = new int[numSegments];
		cumProbs = new float[numSegments];
		for (int i = 0; i < numSegments; i++){
			diffProbs[i] = baseProb;
		}
		
		molResponse = UNIFORM_RESPONSE;
	}
	
	public void updateObject(){
		//probability of binding moves towards the baseline
		if (!body.isActive()){
			//System.out.println("Cell " + this.id + " has been deactivated.");
			body.activate();
		}
		if (molResponse == DIFFERENTIAL_RESPONSE){
			//Move probabilities towards baseline
			for (int i = 0; i < numSegments; i++){
				if (diffProbs[i] > baseProb){
					diffProbs[i]--;
				}
				else if(diffProbs[i] < baseProb){
					diffProbs[i]++;
				}
			}
		}
		else if (molResponse == UNIFORM_RESPONSE){
			if (currentProb < baseProb){
				currentProb++;
			}
			else if (currentProb > baseProb){
				currentProb--;
			}
			for (int i = 0; i < 3; i++){
				cellColor[i] = (float)(currentProb/100.0) * baseCellColor[i];
			}
		}
		//randomly change the velocity of the cell
		Vector3f oldVel = new Vector3f(0f, 0f, 0f);
		body.getLinearVelocity(oldVel);
		Vector3f deltaVel = getRandomDeltaVel();
		//System.out.print("Cell " + this.id + ": Old Vel" + oldVel + " DeltaVel " + deltaVel);
		//System.out.print(" is Active? " + body.isActive());
		deltaVel.add(oldVel);
		//System.out.println(deltaVel);
		body.setLinearVelocity(deltaVel);
		//System.out.println(" newVel: " + deltaVel + " gravity: " + body.getGravity(grav));
		
		//set the current origin
		body.getMotionState().getWorldTransform(trans);
		this.origin.set(trans.origin);
	}
	
	public Vector3f getOrigin(){
		return this.origin;
	}
	
	public float getRadius(){
		return radius;
	}
	
	private Vector3f getRandomDeltaVel(){
		//System.out.println("Getting Random Delta Velocity");
		Vector3f deltaVel = new Vector3f(0f, 0f, 0f);
		float magnitude = (sim.nextRandomF() * maxVelChange);
		float horAngle, verAngle, yMag, xMag, zMag;
		double h;
		switch(molResponse){
			case UNIFORM_RESPONSE:
			case NO_RESPONSE:
				//For these responses, simply get a random velocity vector
				//Get random horizontal angle between 0 and 2 * PI
				horAngle = (float)(sim.nextRandomF() * 2 * Math.PI);
				//Get random vertical angle between -PI/2 to +PI/2
				verAngle = (float)(sim.nextRandomF() * Math.PI - (Math.PI/2));
				
				yMag = (float)(magnitude * Math.sin(verAngle));
				h = magnitude * Math.cos(verAngle);
				xMag = (float)(Math.cos(horAngle)* h);
				zMag = (float)(Math.sin(horAngle) * h);
				
				deltaVel.set(xMag, yMag, zMag);
				
				/*
				System.out.println("  Straight random velocity");
				System.out.print("  horAngle: " + horAngle + "(" + Math.toDegrees(horAngle) + "), ");
				System.out.println("  verAngle: " + verAngle + "(" + Math.toDegrees(verAngle) + ")");
				System.out.print("  Magnitude: " + magnitude + " Delta V: " + xMag + ", " + yMag + ", " + zMag);
				System.out.println(" deltaVel " + deltaVel);*/
				break;
			case DIFFERENTIAL_RESPONSE:
				//Use the probabilities of the horizontal and vertical segments to choose a direction
				//1. Get the cumulative distributions of the horizontal and vertical probabilities
				int totalProbs = 0;
				
				for (int i = 0; i < numSegments; i++){
					totalProbs += diffProbs[i];
				}
				
				cumProbs[0] = diffProbs[0]/(float)totalProbs;
				
				for (int i = 1; i < numSegments; i++){
					cumProbs[i] = (float)(cumProbs[i-1] + diffProbs[i]/(float)totalProbs);
				}
				
				//Now get a random value for direction
				float randomProb = sim.nextRandomF();

				//Find the actual angles
				int randSegment = -1;
				for (int i = 0; i < numSegments; i++){
					if (cumProbs[i] >= randomProb){
						randSegment = i;
						break;
					}
				}
				if (randSegment < 0){
					randSegment = 0;
				}
				
				//Now determine the vertical segment and the horizontal segment
				int verSegment = randSegment / horSegments;
				int horSegment = randSegment % horSegments;
				
				//We have probabilistically chosen a random horizontal and vertical segment to bias the cell's motion toward
				//Now we find the actual angle at the center of that segment
				verAngle = (float)(verDegPerSeg * verSegment - 90 + (verDegPerSeg/2.0));
				horAngle = (float)(horDegPerSeg * horSegment + (horDegPerSeg/2.0));
				
				double verAngleRads = Math.toRadians(verAngle);
				double horAngleRads = Math.toRadians(horAngle);
				yMag = (float)(magnitude * Math.sin(verAngleRads));
				h = magnitude * Math.cos(verAngleRads);
				xMag = (float)(Math.cos(horAngleRads)* h);
				zMag = (float)(Math.sin(horAngleRads) * h);
				deltaVel.set(xMag, yMag, zMag);
				
				/*
				System.out.println("  Differential Response");
				System.out.println("  diffProbs:" + Arrays.toString(diffProbs));
				System.out.println("  Total Probs: " + totalProbs);
				System.out.println("  cumProbs:" + Arrays.toString(cumProbs));
				System.out.println("  random Prob: " + randomProb + "  random Segment: " + randSegment);
				System.out.println("  HorSegment: " + horSegment + " VerSegment: " + verSegment);
				System.out.print("  horAngle: " + horAngle);
				System.out.println("  verAngle: " + verAngle);
				*/
				break;
			default: break;
				
		}
		//System.out.println(deltaVel);
		return (deltaVel);
	}
	
	public static CMBioObjGroup fillSpace(CMSimulation sim, int numCell, Vector3f minP, Vector3f maxP, String name){
		//Will evenly spread the cells throughout the space
		//If the space is not big enough for the cells, it will evenly spread out the maximum
		//number of cells
		
		float interCell = (float)(radius * .01); //distance between cells is 1% of the radius
		float width = maxP.x - minP.x - (2 * interCell); //Make sure there is space on the edges
		float height = maxP.y - minP.y - (2 * interCell);
		float depth = maxP.z - minP.z - (2 * interCell);
		
		
		//Divide the space up into Rows, Columns and Pages
		//RCP >= numCells and C/R approx w/h and P/R approx d/h
		//So R^3 >= numCells * h^2 / w * d
		int numRows = (int)(Math.ceil(Math.pow(numCell * height * height / (width * depth), 1.0/3.0)));
		//System.out.println("Num rows: " + numRows);
		float rowHeight = height/numRows;
		//System.out.println("Row Height: " + rowHeight);
		int numCols = (int)(Math.ceil(numRows * width / height));
		//System.out.println("Num cols: " + numCols);
		float colWidth = width/numCols;
		//System.out.println("colWidth: " + colWidth);
		int numPages = (int)(Math.ceil(numCell / ((float)numRows * numCols)));
		//System.out.println("numPages: " + numPages);
		float pageDepth = depth/numPages;
		//System.out.println("pageDepth: " + pageDepth);
		
		//TODO:  If rowHeight or colWidth or pageDepth are too small to fit the cell
		//Adjust the number of rows or columns until
		CMBioObjGroup theCells = new CMBioObjGroup(sim, name);
		int numSquares = numRows * numCols;
		for (int i = 0; i < numCell; i++){
			int col = i % numCols;
			int row = i / numCols % numRows;
			int page = i / numSquares;
			
			float x = minP.x + (interCell/2) + (col * colWidth) + (colWidth/2);
			float y = minP.y + (interCell/2) + (row * rowHeight) + (rowHeight/2);
			float z = minP.z + (interCell/2) + (page * pageDepth) + (pageDepth/2);
			//System.out.println("New Cell: " + x + ", " + y + ", " + z);
			CMCell newCell = new CMCell(sim, new Vector3f(x,y,z));
			theCells.addObject(newCell);
			newCell.setCellGravity();
			newCell.setInitialVel();
		}
		
		return theCells;
	}
	
	public void setCellGravity(){
		//set the gravity for the cell - it is modified by boyancy
		//a = g(rho_water * volume - mass)/mass + rho_water * Volume
		//This assumes rho_water = 1
		float acceleration = (float)(9.8) * (volume - mass)/(mass + volume);
		body.setGravity(new Vector3f(0,acceleration,0));
	}
	
	private void setInitialVel(){
		float magnitude = sim.nextRandomF() * maxVelChange;
		float hor_angle = (float)(sim.nextRandomF() * 2 * Math.PI);
		float ver_angle = (float)((sim.nextRandomF() * Math.PI) - Math.PI/2.0);
		float y_mag = (float)(magnitude * Math.sin(ver_angle));
		double h = magnitude * Math.cos(ver_angle);
		float x_mag = (float)(Math.cos(hor_angle)* h);
		float z_mag = (float)(Math.sin(hor_angle) * h);
		body.setLinearVelocity(new Vector3f(x_mag, y_mag, z_mag));
	}
	
	
	public CollisionShape getCollisionShape(){
		return cellShape;
	}
	
	public CMRigidBody getRigidBody(){
		return body;
	}
	
	public Vector3f getColor3Vector(){
		return new Vector3f(cellColor[0], cellColor[1], cellColor[2]);
	}
	
	public void setVisible(boolean v){
		visible = v;
	}
	
	public boolean isVisible(){
		return visible;
	}
	
	public void collided(CMBioObj c, Vector3f localPoint, Vector3f otherPoint, long collId){
		//Find the vector to the collision point
		Vector3f newVel = new Vector3f();
		newVel.set(localPoint);
		if (c instanceof CMMolecule){
			switch (molResponse){
				case NO_RESPONSE:
					break;
				case UNIFORM_RESPONSE:
					//See if molecule binds
					if (sim.nextRandomF() > (float)currentProb/100.0){
						//molecule does not bind
						return;
					}
					//See if molecule has already been marked for removal
					if (c.isMarked()){
						return;
					}
					//System.out.println("Molecule bound");
					c.markForRemoval();
					currentProb += deltaProb;
					if (currentProb > 100){
						//if molecule binds, the cell becomes more receptive. This value could be 
						//different depending on the speed of response
						currentProb = 100;
					}
					
					for (int i = 0; i < 3; i++){
						cellColor[i] = (float)(currentProb/100.0) * baseCellColor[i];
					}
					//Bias velocity towards molecule
					//Find the current speed (magitude of velocity)
					Vector3f oldVel = new Vector3f(0, 0, 0);
					body.getLinearVelocity(oldVel);
			
					//Scale the new velocity vctor to response length
					newVel.scale(responseDeltaVel);
					//Add the response velocity to the old velocity
					newVel.add(oldVel);
					body.setLinearVelocity(newVel);
					break;
				case DIFFERENTIAL_RESPONSE:
					//The probabilities at the horizontal and vertical angles change
					//Find the vertical segment
					float verAngle = (float)(Math.asin(newVel.y/newVel.length()));
					double h = Math.cos(verAngle) * newVel.length();
					verAngle = (float)Math.toDegrees(verAngle);
					int verSegment = (int)((verAngle + 90) / verDegPerSeg);
					assert verSegment < verSegments : "Vertical Segment > verSegments";
					
					float horAngle = (float)(Math.acos(newVel.x/h));
					horAngle = (float)Math.toDegrees(horAngle);
					//Adjust to an angle between 0 and 2 * PI
					if (newVel.z < 0){
						horAngle = 360 - horAngle;
					}
					int horSegment = (int)(horAngle / horDegPerSeg);
					
					int index = horSegments * verSegment + horSegment;
					
					//See if molecule binds
					if (sim.nextRandomF() > (float)diffProbs[index]/100.0){
						//molecule does not bind
						return;
					}
					//See if molecule has already been marked for removal
					if (c.isMarked()){
						return;
					}
					//System.out.println("Molecule bound");
					c.markForRemoval();
					
					diffProbs[index] += deltaProb;
					if (diffProbs[index] > 100){
						diffProbs[index] = 100;
					}
					
					
					//System.out.println("Vector: " + newVel + " verAngle: " + verAngle + " verSegment: " + verSegment);
					//System.out.println("horAngle: " + horAngle + " horSegment: " + horSegment);
					
					break;
				default:
					break;
			}
		}
		else if ((c instanceof CMCell) || (c instanceof CMWall)){
			if (sim.nextRandomF() > (float)currentProb/100.0){
				//molecule does not bind
				return;
			}
			System.out.println("Cell " + id + " is binding to " + c.getID());
			if (sim.constraintExists(collId)){
				sim.getConstraint(collId).checkIn();
				System.out.println("Cell " + id + " is checking in.");
			}
			else{
				CMGenericConstraint con = new CMGenericConstraint(sim, body, c.getRigidBody(), localPoint, otherPoint, 1000, 20, collId);
				con.checkIn();
				System.out.println("Cell " + id + " has made a constraint.");
			}
		}
	}
	
	public boolean specialRender(IGL gl, Transform t){
		return false;
	}
	
	public String getCsvData(){
		//Find position
		this.body.getMotionState().getWorldTransform(trans);
		
		String s = "Cell," + this.id + "," + trans.origin.x + ","  + trans.origin.y + "," + trans.origin.z + "\n";
		return s;
	}
	
	public String toString(){
		String s = "I am cell " + this.id;
		return s;
	}
	
	public int getID(){
		return this.id;
	}
	
	public void setType(String s){
		objectType = s;
	}
	
	public String getType(){
		return objectType;
	}
	
	public float getMass(){
		return mass;
	}
	
	public CMSimulation getSim(){
		return sim;
	}
	
	public void addConstraint(CMGenericConstraint c){
		
	}
	
	public void removeConstraint(CMGenericConstraint c){
		
	}
	
	public void destroy(){
		body.destroy();
	}
	
	public void markForRemoval(){
		toRemove = true;
	}
	
	public boolean isMarked(){
		return toRemove;
	}
}
