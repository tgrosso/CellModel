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
import javax.vecmath.Matrix3f;

import org.lwjgl.util.glu.Sphere;

import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;


public class CMCell implements CMBioObj{
	protected static float radius;
	protected static float density;
	protected static float volume;
	protected static float mass;
	protected static float maxVelChange = 0.3f;
	protected static int cell_ids = 0;
	protected int id;
	protected Vector3f origin;
	private static SphereShape cellShape;
	protected CMRigidBody body;
	protected Transform trans;
	protected float cellFriction = .3f;
	protected static float[] baseCellColor = {1.0f, 1.0f, 1.0f};
	private float[] cellColor;
	protected float cameraDistance = 20f;
	protected boolean visible = true;
	protected boolean toRemove = false;
	protected CMSimulation sim;
	protected String objectType = "Cell";
	protected boolean bound = false;
	
	protected int baseProb = 40; //% probability that molecule will bind
	protected int currentProb = baseProb; //For Uniform response, the probability that molecule will bind to the whole cell
	protected int deltaProb = 10;
	
	public CMCell(CMSimulation s, Vector3f o){
		this.origin = new Vector3f(o);
		this.sim = s;
		
		radius = 5.0f;
		density = 1.2f;
		volume = (float)(4.0/3.0 * Math.PI * radius * radius * radius);
		mass = density * volume;
		cellShape = new SphereShape(radius);
		
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
	}
	
	public void updateObject(){
		//probability of binding moves towards the baseline
		if (!body.isActive()){
			//System.out.println("Cell " + this.id + " has been deactivated.");
			body.activate();
		}
		Vector3f oldVel = new Vector3f();
		body.getLinearVelocity(oldVel);
		oldVel.add(getRandomVector(maxVelChange));
		//Apply a random force to the cell
		body.setLinearVelocity(oldVel);

		//set the current origin
		body.getMotionState().getWorldTransform(trans);
		this.origin.set(trans.origin);
		//System.out.println("Cell " + this.id + " Origin: " + this.origin + " and visible?" + isVisible());
		//System.out.println("   and cell color is " + getColor3Vector());
	}
	
	public Vector3f getOrigin(){
		return this.origin;
	}
	
	public float getRadius(){
		return this.radius;
	}
	
	protected Vector3f getRandomVector(float mag){
		//System.out.println("Getting Random Delta Velocity");
		Vector3f deltaVel = new Vector3f(0f, 0f, 0f);
		float magnitude = (sim.nextRandomF() * mag);
		float horAngle, verAngle, yMag, xMag, zMag;
		double h;
		//Simply get a random velocity vector
		//Get random horizontal angle between 0 and 2 * PI
		horAngle = (float)(sim.nextRandomF() * 2 * Math.PI);
		//Get random vertical angle between -PI/2 to +PI/2
		verAngle = (float)(sim.nextRandomF() * Math.PI - (Math.PI/2));
				
		yMag = (float)(magnitude * Math.sin(verAngle));
		h = magnitude * Math.cos(verAngle);
		xMag = (float)(Math.cos(horAngle)* h);
		zMag = (float)(Math.sin(horAngle) * h);
				
		deltaVel.set(xMag, yMag, zMag);
		
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
	
	public void collided(CMBioObj c, ManifoldPoint pt, long collId){
		//System.out.println("I am " + getType() + ";" + getID() + " and I have collided");
		if (c instanceof CMWall){
			if (sim.constraintExists(collId)){
				sim.getConstraint(collId).checkIn();
			}
			else{
				float[] mat = {1f, 0f, 0f, 
						0f, 1f, 0f,
						0f, 0f, 1f};
		Matrix3f world = new Matrix3f(mat);
		Transform worldTrans = new Transform(world);
		Transform myTrans = new Transform();
		Transform otherTrans = new Transform();

		
		//worldTrans.inverse();
		
		body.getMotionState().getWorldTransform(myTrans);
		c.getRigidBody().getMotionState().getWorldTransform(otherTrans);
		//myTrans.inverse();
		//otherTrans.inverse();
		
		myTrans.inverse();
		myTrans.mul(worldTrans);
		otherTrans.inverse();
		otherTrans.mul(worldTrans);
		
		CMGenericConstraint con = new CMGenericConstraint(sim, body, c.getRigidBody(), myTrans, otherTrans, true, 5000, 50, collId, 0, 0, -1);
		con.getConstraint().setLinearLowerLimit(new Vector3f(0, 0, 0));
		con.getConstraint().setLinearUpperLimit(new Vector3f(3f, 3f, 3f));
		con.checkIn();
		sim.addConstraint(con);
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
	
	public void destroy(){
		body.destroy();
	}
	
	public void markForRemoval(){
		toRemove = true;
	}
	
	public boolean isMarked(){
		return toRemove;
	}
	
	public boolean isBound(){
		return bound;
	}
	public void clearBound(){
		bound = false;
	}
	
	public void bind(){
		bound = true;
	}
}
