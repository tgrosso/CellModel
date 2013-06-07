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

import javax.vecmath.Vector3f;

import org.lwjgl.util.glu.Sphere;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;


public class CMCell implements CMBioObj{
	private static float radius = 5.0f;
	private static float density = 1.2f;
	private static float volume = (float)(4.0/3.0 * Math.PI * radius * radius * radius);
	private static float mass = density * volume;
	private static float maxVelChange = 0.5f;
	private static int cell_ids = 0;
	private int id;
	private Vector3f origin;
	private static SphereShape cellShape = new SphereShape(radius);
	private CMRigidBody body;
	private Transform trans;
	private float cellFriction = .2f;
	private static float[] cellColor = {1.0f, 0.5f, 0.5f, 1.0f};
	protected float cameraDistance = 20f;
	private boolean visible = true;
	private CMSimulation sim;
	private String objectType = "Cell";
	
	public CMCell(CMSimulation s, Vector3f o){
		this.origin = o;
		this.sim = s;
		
		trans = new Transform();
		trans.setIdentity();
		trans.origin.set(origin);
		
		Vector3f localInertia = new Vector3f(0, 0, 0);
		cellShape.calculateLocalInertia(mass, localInertia);

		DefaultMotionState motionState = new DefaultMotionState(trans);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, cellShape, localInertia);
		rbInfo.friction = cellFriction;
		body = new CMRigidBody(rbInfo, this);
		float magnitude = sim.nextRandomF() * maxVelChange;
		float hor_angle = sim.nextRandomF() * 360;
		float ver_angle = sim.nextRandomF() * 360;
		float y_mag = (float)(magnitude * Math.sin(ver_angle));
		double h = magnitude * Math.cos(ver_angle);
		float x_mag = (float)(Math.cos(hor_angle)* h);
		float z_mag = (float)(Math.sin(hor_angle) * h);
		body.setLinearVelocity(new Vector3f(x_mag, y_mag, z_mag));
		//body.setLinearVelocity(new Vector3f(3, 3, 3));
		
		//set the gravity for the cell - it is modified by boyancy
		//a = g(rho_water * volume - mass)/mass + rho_water * Volume
		//This assumes rho_water = 1
		float acceleration = (float)(9.8) * (volume - mass)/(mass + volume);
		body.setGravity(new Vector3f(0,acceleration,0));
		
		this.id = cell_ids;
		cell_ids++;
	}
	
	public void updateObject(Random r){
		//randomly change the velocity of the cell
		float magnitude = r.nextFloat() * maxVelChange;
		float hor_angle = r.nextFloat() * 360;
		float ver_angle = r.nextFloat() * 360;
		float y_mag = (float)(magnitude * Math.sin(ver_angle));
		double h = magnitude * Math.cos(ver_angle);
		float x_mag = (float)(Math.cos(hor_angle)* h);
		float z_mag = (float)(Math.sin(hor_angle) * h);
		Vector3f oldVel = new Vector3f(0, 0, 0);
		body.getLinearVelocity(oldVel);
		body.setLinearVelocity(new Vector3f(oldVel.x + x_mag, oldVel.y + y_mag, oldVel.z + z_mag));
		//System.out.println(this.id + "-Velocity Changes:" + magnitude + " Direction:" + x_mag + "," + y_mag + ", " + z_mag );
		float acceleration = (float)(9.8) * (volume - mass)/(mass + volume);
		body.setGravity(new Vector3f(0,acceleration,0));
		//System.out.print(mass + " " + acceleration + " ");
		//Vector3f out = new Vector3f(0,0,0);
		//out = body.getGravity(out);
		//System.out.println("gravity: " + out);
	}
	
	public static CMBioObjGroup fillSpace(CMSimulation sim, int numCell, Vector3f minP, Vector3f maxP, String name){
		//Will evenly spread the cells throughout the space
		//If the space is not big enough for the cells, it will evenly spread out the maximum
		//number of cells
		
		float width = maxP.x - minP.x - (2 * radius);
		float height = maxP.y - minP.y - (2 * radius);
		float depth = maxP.z - minP.z - (2 * radius);
		float interCell = (float)(radius * .01); //distance between cells is 1% of the radius
		
		//Divide the space up into Rows, Columns and Pages
		//RCP >= numCells and C/R approx w/h and P/R approx d/h
		//So R^3 >= numCells * h^2 / w * d
		int numRows = (int)(Math.ceil(Math.pow(numCell * height * height / (width * depth), 1.0/3.0)));
		float rowHeight = height/numRows;
		int numCols = (int)(Math.ceil(numRows * width / height));
		float colWidth = width/numCols;
		int numPages = (int)(Math.ceil(numCell / (numRows * numCols)));
		float pageDepth = depth/numPages;
		
		//TODO:  If rowHeight or colWidth or pageDepth are too small to fit the cell
		//Adjust the number of rows or columns until
		CMBioObjGroup theCells = new CMBioObjGroup(sim, name);
		int numSquares = numRows * numCols;
		for (int i = 0; i < numCell; i++){
			int col = i % numCols;
			int row = i / numCols % numRows;
			int page = i / numSquares;
			
			float x = minP.x + (col * colWidth) + (colWidth/2 + interCell);
			float y = minP.y + (row * rowHeight) + (rowHeight/2 + interCell);
			float z = minP.z + (page * pageDepth) + (pageDepth/2 + interCell);
			CMCell newCell = new CMCell(sim, new Vector3f(x,y,z));
			theCells.addObject(newCell);
		}
		
		//System.out.println(numRows);
	    //System.out.println(numCols);
		//System.out.println(numPages);
		return theCells;
	}
	
	
	public CollisionShape getCollisionShape(){
		return cellShape;
	}
	
	public RigidBody getRigidBody(){
		return body;
	}
	
	public Vector3f getColor3Vector(){
		return new Vector3f(1.0f, 1.0f, 0.0f);
	}
	
	public void setVisible(boolean v){
		visible = v;
	}
	
	public boolean isVisible(){
		return visible;
	}
	
	public void collided(CMBioObj c, Vector3f p){
		if (c instanceof CMMolecule){
			//Bias direction (not velocity!) towards molecule
			//Find the current speed (magitude of velocity)
			Vector3f oldVel = new Vector3f(0, 0, 0);
			body.getLinearVelocity(oldVel);
			float oldMagnitude = oldVel.length();
			
			//Find the normal to the collision point
			Transform t = new Transform();
			t = this.body.getMotionState().getWorldTransform(t);
			origin = t.origin;
			Vector3f newVel = p;
			newVel.sub(origin);
			
			//scale the normal to the current speed
			newVel.scale(oldMagnitude/newVel.length());
			body.setLinearVelocity(newVel);
		}
		else if (c instanceof CMWall){
			//System.out.println("Collided with Wall");
		}
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
	
	public void addConstraint(CMGenericConstraint c){
		
	}
	
	public void removeConstraint(CMGenericConstraint c){
		
	}
}
