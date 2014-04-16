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
	private boolean isActive; //the constraint is active.. either a current constraint or a constraint that needs to be added to the world
	private int checked; //checks in number of constraints (can keep track of number for destroy method)
	private long initialTime; //need to find a way to subtract Date values
	private long life; // amount of time the constraint will last
	private long collisionID;
	private float bonds, maxBonds;
	private int segment = -1;
	CMSegmentedCell cell = null;
	CMSimulation sim;
	Generic6DofConstraint constraint;
	
	public CMGenericConstraint(CMSimulation s, CMRigidBody rbA, CMRigidBody rbB, Transform localA, Transform localB, boolean useLinearReferenceFrameA, long mu, long sd, long ID, int bds, int maxBds, int seg){
		constraint= new Generic6DofConstraint(rbA, rbB, localA, localB, true);
		sim = s;
		isActive = false;
		checked = 0;
		collisionID = ID;
		initialTime = sim.getCurrentTimeMicroseconds();
		life = (long) ((mu) + (sim.nextGaussianF() * sd));
		if (life < 0){
			life = 0;
		}
		bonds = bds;
		maxBonds = maxBds;
		sim.addConstraint(this);
		segment = seg;
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
		}
		
		
	}
	
	//updating time to see if constraint continues or is broken in next time step
	public void updateTime(){
		if (bonds < 10){
			isActive = false;
			return;
		}
		float strength = (bonds/(float)maxBonds);
		float maxBondLength = 1.5f;
		float bondLength = (1.0f - strength)*maxBondLength;
		float angularLimitX = strength * BulletGlobals.SIMD_PI;
		float angularLimitYZ = strength * BulletGlobals.SIMD_HALF_PI;
		constraint.setLinearLowerLimit(new Vector3f(0, 0, 0));
		constraint.setLinearUpperLimit(new Vector3f(0f, bondLength, 0f));
		constraint.setAngularLowerLimit(new Vector3f(-angularLimitX, -angularLimitYZ, -angularLimitYZ));
		constraint.setAngularUpperLimit(new Vector3f(angularLimitX, angularLimitYZ, angularLimitYZ)); //can rotate around z axis
		int brokenBonds = Math.round(bonds * .1f);
		bonds = bonds - brokenBonds; //Have to make this based on probability
		cell.breakBonds(brokenBonds, segment);
		//System.out.println("Constraint " + collisionID + " Strength: " + strength);
		
	}
	
	//need better way to clean this up
	public boolean isActive(){
		return isActive;
	}
	
	
	public void destroy(){
		if (isActive != true){
			//objects informed that the constraint is removed... this might have been informed in isCAlive
			//remove checks
			
			//constraint removed in CMSimulation by removeConstraint method in DynamicsWorld
		}
	}
	
	public long getID(){
		return (collisionID);
	}
	
	public Generic6DofConstraint getConstraint(){
		return constraint;
	}
}
