/**
 *  Copyright (C) 2013 Terri A. Grosso
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * 
 * Terri A. Grosso
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


public class CMCell implements CMBioObj{
	private static float radius = 5.0f;
	private static float mass = (float)(4.0/3.0 * Math.PI * radius * radius * radius);  //assume density of 1 g/cm^3
	private static float maxVelChange = 0.5f;
	private static int mol_ids = 0;
	private int id;
	private Vector3f origin;
	private static SphereShape cellShape = new SphereShape(radius);
	private RigidBody body;
	private static float[] cellColor = {1.0f, 0.5f, 0.5f, 1.0f};
	protected float cameraDistance = 20f;
	private boolean visible = true;
	
	public CMCell(CMSimulation sim, Vector3f o){
		this.origin = o;
		
		Transform t = new Transform();
		t.setIdentity();
		t.origin.set(origin);
		
		Vector3f localInertia = new Vector3f(0, 0, 0);
		cellShape.calculateLocalInertia(mass, localInertia);

		DefaultMotionState motionState = new DefaultMotionState(t);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, cellShape, localInertia);
		body = new RigidBody(rbInfo);
		float magnitude = sim.nextRandomF() * maxVelChange;
		float hor_angle = sim.nextRandomF() * 360;
		float ver_angle = sim.nextRandomF() * 360;
		float y_mag = (float)(magnitude * Math.sin(ver_angle));
		double h = magnitude * Math.cos(ver_angle);
		float x_mag = (float)(Math.cos(hor_angle)* h);
		float z_mag = (float)(Math.sin(hor_angle) * h);
		body.setLinearVelocity(new Vector3f(x_mag, y_mag, z_mag));
		//body.setLinearVelocity(new Vector3f(3, 3, 3));
		
		this.id = mol_ids;
		mol_ids++;
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
	}
	
	public static void fillSpace(CMSimulation sim, int numCell, Vector3f minP, Vector3f maxP){
		//Will evenly spread the cells throughout the space
		
		float x_len = maxP.x - minP.x;
		float y_len = maxP.y - minP.y;
		float z_len = maxP.z - minP.z;
		
		for (int i = 0; i < numCell; i++){
			float randX = x_len * sim.nextRandomF();
			float randY = y_len * sim.nextRandomF();
			float randZ = z_len * sim.nextRandomF();
			sim.addBioObject(new CMMolecule(sim, new Vector3f(randX+minP.x, randY+minP.y, randZ+minP.z)));
		}
		
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
	
	public String toString(){
		String s = "I am cell " + this.id;
		return s;
	}
}
