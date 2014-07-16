/**
 * Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 * Package: cellModel
 * File: CMPoint2PointConstraint.java
 * Aug 31, 2013 1:09:20 PM
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * 
 */
package cellModel;

import javax.vecmath.Vector3f;
import com.bulletphysics.dynamics.constraintsolver.Point2PointConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;

/**
 * @author tagsit
 *
 */
public class CMPoint2PointConstraint implements CMConstraint{
	private boolean isActive; //the constraint is active.. either a current constraint or a constraint that needs to be added to the world
	private boolean neverActive = true;
	private int checked; //checks in number of constraints (can keep track of number for destroy method)
	private long initialTime; 
	private long life; // amount of time the constraint will last
	private long collisionID;
	private int constraintID;
	Point2PointConstraint constraint;
	private CMSimulation sim;
	
	CMPoint2PointConstraint(CMSimulation s, CMRigidBody rbA, CMRigidBody rbB, Vector3f pivotInA, Vector3f pivotInB, long mu, long sd, long collId, int conId){
		constraint = new Point2PointConstraint(rbA, rbB, pivotInA, pivotInB);
		sim = s;
		isActive = false;
		checked = 0;
		collisionID = collId;
		constraintID = conId;
		initialTime = sim.getCurrentTimeMicroseconds()/1000;
		life = (long) ((mu) + (sim.nextGaussianF() * sd));
		sim.addConstraint(this);
	}

	public void checkIn(){
		if (checked < 2){
			checked ++;
		}
		if (checked >= 2){
			isActive = true;
			neverActive = false;
		}
	}
	
	public void updateTime(){
		long currentTime = sim.getCurrentTimeMicroseconds()/1000;
		if (currentTime - initialTime > life){
			isActive = false;
		}
	}
	
	public boolean isActive(){
		return isActive;
	}
	
	public void destroy(){
		
	}
	
	public long getID(){
		return collisionID;
	}
	
	public Point2PointConstraint getConstraint(){
		return constraint;
	}
	
	public boolean hasBeenActive(){
		return !neverActive;
	}
	
	public long getCollId(){
		return collisionID;
	}
	public int getConId(){
		return constraintID;
	}
	
}
