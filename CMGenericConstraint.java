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

import java.util.Date;
import java.util.Random;
import javax.vecmath.Vector3f;
import java.util.Random;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;


public class CMGenericConstraint extends Generic6DofConstraint{ // implements CMBioObj {
	private boolean isActive; //the constraint is active.. either a current constraint or a constraint that needs to be added to the world
	private int checked; //checks in number of constraints (can keep track of number for destroy method)
	private Date initialTime; //need to find a way to subtract Date values
	private long life; // amount of time the constraint will last
	
	
	
	public CMGenericConstraint(long mu, long sd){
		isActive = false;
		checked = 0;
		//using Gaussian (normal) distribution
		Random lifeGenerator = new Random();
		life = (long) ((mu) + (lifeGenerator.nextDouble()) * sd);
		}
	
	
	//have to add collision detection to see if the two objects collided to perform this function
	//have to check if collided with another cell... should collided function have two parameters? (CMBioObj c, CMBioObj d)**
	public void checkIn(){
		if (checked < 2){
			checked++; //should we inform the objects here that they have been checked in for a constraint?
		}
		
		if (checked >= 2){
			isActive = true;// if is active is = true, do we want the cells to stick to more than one other cell?
			//if is active = true, will generate a constraint
		}
		
		
	}
	
	//updating time to see if constraint continues or is broken in next time step
	public void updateTime(Date currentTime){
	
		//if (currentTime - initialTime > life){//how to subtract date.util time?**
			//isActive = false;
		//}
		
	}
	
	//need better way to clean this up
	public boolean isAlive;
	
	public void isCAlive(){
		if (isActive = true){
			isAlive = true;
		}
		else {
			isAlive =false;
		}
	}
	
	
	public void destroy(){
		if (isAlive != true){
			//objects informed that the constraint is removed... this might have been informed in isCAlive
			//remove checks
			
			//constraint removed in CMSimulation by removeConstraint method in DynamicsWorld
		}
		
	
		
	}

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	


}
