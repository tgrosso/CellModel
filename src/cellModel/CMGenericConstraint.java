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

import javax.vecmath.Vector3f;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.linearmath.Transform;

public class CMGenericConstraint implements CMConstraint{ // implements CMBioObj {
	private int MAX_CONSTRAINTS = 100;
	private boolean isActive; //the constraint is active.. either a current constraint or a constraint that needs to be added to the world
	private boolean neverActive = true;
	private int checked; //checks in number of constraints (can keep track of number for destroy method)
	private long initialTime; //need to find a way to subtract Date values
	private long lifeSpan = 60 * 20 * 1000 * 1000; // longest lifespan in microseconds (10 minutes)
	private float a = 3f/(lifeSpan * lifeSpan);
	private long collisionId;
	private int constraintId;
	private int segment = -1;
	private int proteinId = -1;
	CMSegmentedCell cell = null;
	CMSimulation sim;
	Generic6DofConstraint constraint;
	
	public CMGenericConstraint(CMSimulation s, CMRigidBody rbA, CMRigidBody rbB, Transform localA, Transform localB, boolean useLinearReferenceFrameA, long collId, int conId, int seg, int pro){
		constraint= new Generic6DofConstraint(rbA, rbB, localA, localB, true);
		sim = s;
		isActive = false;
		checked = 0;
		collisionId = collId;
		constraintId = conId; 
		initialTime = sim.getCurrentTimeMicroseconds();
		sim.addConstraint(this);
		segment = seg;
		proteinId = pro;
		if (rbA.getParent() instanceof CMSegmentedCell){
			cell = (CMSegmentedCell)rbA.getParent();
		}
	}
	
	public void checkIn(){
		if (checked < 2){
			checked++; //should we inform the objects here that they have been checked in for a constraint?
		}
		
		if (checked >= 2){
			isActive = true;// if is active is = true, do we want the cells to stick to more than one other cell?
			neverActive = false;
		}
		
		
	}
	
	//updating time to see if constraint continues or is broken in next time step
	public void updateTime(){
		long age = sim.getCurrentTimeMicroseconds() - initialTime;
		long time = sim.getCurrentTimeMicroseconds();
		float deg = getDegraded(time);
		float fa = getFocalAdhesionDevelopment(time);
		float force = getForceFactor(time);
		
		float prob = deg * fa * force;
		
		if (sim.nextRandomF() < prob){
			isActive = false;
		}
		
		//System.out.println("Constraint CollId: " + collisionID + " age: " + age + " deg: " + deg + " fa: " + fa);
		//System.out.println("     prob " + prob + " alive: " + isActive);
	}
	
	public float getDegraded(long age){
		//This is the probability of breakage due to kinase degredation. It increases linearly with time.
		float x = (float)age;
		float degradation = (float)(x / lifeSpan);
		return degradation;
	}
	
	public float getFocalAdhesionDevelopment(long age){
		//This is the probability of breakage due to focal adhesions develop and then degrade over time
		long value = age - (lifeSpan/2);
		float fad = a * value * value + .25f;
		return (.5f * fad);
	}
	
	public float getForceFactor(long currentTime){
		return .1f;
	}
	
	//need better way to clean this up
	public boolean isActive(){
		return isActive;
	}
	
	
	public void destroy(){
		sim.writeToLog("Destroying Constraint: Collision Id " + collisionId + " constraint Id " + constraintId + " lifespan " + (sim.getCurrentTimeMicroseconds() - initialTime));
		if (isActive != true){
			//objects informed that the constraint is removed... this might have been informed in isCAlive
			//remove checks
			
			//constraint removed in CMSimulation by removeConstraint method in DynamicsWorld
			if (neverActive){
				//if never activated, return all unused bound proteins
				if (cell != null){
					cell.reclaimMembraneProteins(segment, proteinId);
				}
			}
		}
	}
	
	public long getCollId(){
		return (collisionId);
	}
	
	public int getConId(){
		return (constraintId);
	}
	
	public boolean hasBeenActive(){
		return !neverActive;
	}
	
	public Generic6DofConstraint getConstraint(){
		return constraint;
	}
	
	//public static CMGenericConstraint createConstraint(){
	//	
	//}
}