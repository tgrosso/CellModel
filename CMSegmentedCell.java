/**
 * Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 * Package: cellModel
 * File: CMSegmentedCell.java
 * Jul 28, 2013 9:59:03 AM
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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Random;

import javax.vecmath.Vector3f;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.extras.gimpact.GImpactMeshShape;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.collision.shapes.TriangleIndexVertexArray;
import com.bulletphysics.collision.shapes.TriangleShape;

import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;

import cellModel.shapes.CMGImpactMeshSphere;
import cellModel.shapes.CMBvhTriangularMeshSphere;

import static com.bulletphysics.demos.opengl.IGL.*;

/**
 * @author tagsit
 *
 */
public class CMSegmentedCell implements CMBioObj{
	
	private static int cell_ids = 0;
	private static String type = "Segmented Cell";
	
	
	private CMGImpactMeshSphere shape;
	private CMRigidBody body;
	private CMSimulation sim;
	
	private boolean isVisible = true;
	private boolean toRemove = false;
	private int myId;
	private int numSegments;
	
	private float density = 1.10f;
	private float mass;
	private float radius;
	private float volume;
	private float[] baseColor = {1.0f, 0f, 0f};
	private float[][] color;
	private float maxVel = 0.1f;
	
	private Transform trans;
	private Vector3f origin;
	
	private static Vector3f aabbMax = new Vector3f(1e30f, 1e30f, 1e30f);
	private static Vector3f aabbMin = new Vector3f(-1e30f, -1e30f, -1e30f);
	
	private static float[] glMat = new float[16];
	
	public CMSegmentedCell(CMSimulation s, float r, Vector3f o, int dl){
		//TODO - should this be a subclass of CMCell? Think about that.
		myId = cell_ids;
		cell_ids++;
		sim = s;
		radius = r;
		origin = o;
		volume = (float)(4.0 / 3.0 * Math.PI * radius * radius * radius);
		mass = density * volume;
		int detail_level = dl;
		if (detail_level < 0){
			detail_level = 0;
		}
		if (detail_level > 3){
			detail_level = 3;
		}
		shape = new CMGImpactMeshSphere(detail_level);
		shape.setLocalScaling(new Vector3f(radius, radius, radius));
		shape.updateBound();
		
		trans = new Transform();
		trans.setIdentity();
		trans.origin.set(origin);
		
		Vector3f localInertia = new Vector3f(0, 0, 0);
		shape.calculateLocalInertia(mass, localInertia);
		DefaultMotionState motionState = new DefaultMotionState(trans);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, shape, localInertia);
		body = new CMRigidBody(rbInfo, this);
		
		numSegments = shape.getNumTriangles();
		
		sim.setNeedsGImpact(true);
		
		color = new float[numSegments][];
		for (int i = 0; i < numSegments; i++){
			color[i] = new float[3];
			for (int j = 0; j < 3; j++){
				color[i][j] = (float)Math.random();
			}
		}
		
		//System.out.println("Segmented Cell Created");
	}
	
	public void setCellGravity(){
		//set the gravity for the cell - it is modified by boyancy
		//a = g(rho_water * volume - mass)/mass + rho_water * Volume
		//This assumes rho_water = 1
		float acceleration = (float)(9.8) * (volume - mass)/(mass + volume);
		body.setGravity(new Vector3f(0,acceleration,0));
	}
	
	
	public void collided(CMBioObj c, Vector3f point, long collID){
		//System.out.println("I've collided!");
	}
	
	public boolean specialRender(IGL gl, Transform t){
		gl.glPushMatrix();
		t.getOpenGLMatrix(glMat);
		gl.glMultMatrix(glMat);
		
		drawSegmentsCallback drawCallback = new drawSegmentsCallback(gl, this);
		shape.processAllTriangles(drawCallback, aabbMin, aabbMax);
		gl.glPopMatrix();
		
		return true;
	}
	
	public float[] getSegmentColor(int segment){
		return color[segment];
	}
	
	public CollisionShape getCollisionShape(){
		return shape;
	}
	
	public RigidBody getRigidBody(){
		return body;
	}
	public void updateObject(Random r){
		if (!body.isActive()){
			//System.out.println("Cell " + this.myId + " has been deactivated.");
			body.activate();
		}
		
		Vector3f oldVel = new Vector3f(0f, 0f, 0f);
		body.getLinearVelocity(oldVel); 
		float magnitude = (sim.nextRandomF() * maxVel);
		Vector3f deltaVel = new Vector3f(0f, 0f, 0f);
		
		//Get random horizontal angle between 0 and 2 * PI
		float horAngle = (float)(sim.nextRandomF() * 2 * Math.PI);
		
		//Get random vertical angle between -PI/2 to +PI/2
		float verAngle = (float)(sim.nextRandomF() * Math.PI - (Math.PI/2));
		
		float yMag = (float)(magnitude * Math.sin(verAngle));
		double h = magnitude * Math.cos(verAngle);
		float xMag = (float)(Math.cos(horAngle)* h);
		float zMag = (float)(Math.sin(horAngle) * h);
		
		deltaVel.set(xMag, yMag, zMag);
		deltaVel.add(oldVel);
		body.setLinearVelocity(deltaVel);
	}
	
	public Vector3f getColor3Vector(){
		return new Vector3f();
	}
	
	public void setVisible(boolean v){
		isVisible = v;
	}
	
	public boolean isVisible(){
		return isVisible;
	}
	
	public void addConstraint(CMGenericConstraint c){
		
	}
	public void removeConstraint(CMGenericConstraint c){
		
	}
	
	public int getID(){
		return myId;
	}
	
	public float getMass(){
		return mass;
	}
	
	public String getType(){
		return type;
	}
	
	public void destroy(){
		
	}
	
	public void markForRemoval(){
		toRemove = true;
	}
	
	public boolean isMarked(){
		return toRemove;
	}
	
	private static class drawSegmentsCallback extends TriangleCallback {
		private IGL gl;
		CMSegmentedCell parent;

		public drawSegmentsCallback(IGL gl, CMSegmentedCell p) {
			this.gl = gl;
			this.parent = p;
		}
		
		public void processTriangle(Vector3f[] triangle, int partId, int triangleIndex) {
			float[] color = parent.getSegmentColor(triangleIndex);
			gl.glBegin(GL_TRIANGLES);
			gl.glColor3f(color[0], color[1], color[2]);
			gl.glVertex3f(triangle[0].x, triangle[0].y, triangle[0].z);
			//gl.glColor3f(color[0]);
			gl.glVertex3f(triangle[1].x, triangle[1].y, triangle[1].z);
			//gl.glColor3f(1, 0, 0);
			gl.glVertex3f(triangle[2].x, triangle[2].y, triangle[2].z);
			gl.glEnd();
			
		}
	}

}
