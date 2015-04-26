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
 * File: CMWall.java
 * Apr 2, 2013 4:43:30 PM
 */
package cellModel;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor;
import com.bulletphysics.util.ObjectArrayList;

import javax.vecmath.Matrix3f;

import java.nio.FloatBuffer;
import java.util.Random;

import javax.vecmath.Vector3f;

import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;


public class CMWall implements CMBioObj{
	private static int wall_ids = 0;
	protected CMRigidBody body;
	private BoxShape wallShape;
	private Vector3f origin;
	private float[] wallColor = {.4f, 0.2f, 0.2f, 1f};
	protected float width, height, depth;
	private static FloatBuffer buffer = BufferUtils.createFloatBuffer(16);
	private boolean visible = true;
	private boolean toRemove = false;
	private int id;
	private CMSimulation sim;
	private boolean bound = false;
	private float lamininReactionRate = (float)(6.42 * Math.pow(10, -9));
	private float lamininSurfaceDensity; //molecules per micron^2
	private long lastTimeMilliseconds, currentTimeMilliseconds;
	private boolean hasLaminin;
	private static boolean finalWritten = false;
	
	public CMWall(CMSimulation s, float w, float h, float d, Vector3f o){
		float mass = 0;
		sim = s;
		width = w;
		height = h;
		depth = d;
		origin = o;
		Transform t = new Transform();
		t.setIdentity();
		t.origin.set(origin);
		lamininSurfaceDensity = 700;
		currentTimeMilliseconds = sim.getCurrentTimeMicroseconds()/1000;
		hasLaminin = false;
		
		Vector3f localInertia = new Vector3f(0, 0, 0);
		wallShape = new BoxShape(new Vector3f(width/2, height/2, depth/2));
		DefaultMotionState motionState = new DefaultMotionState(t);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, wallShape, localInertia);
		body = new CMRigidBody(rbInfo, this);
		
		this.id = wall_ids;
		wall_ids++;
		//Vector3f minAABB = new Vector3f(0,0,0);
		//Vector3f maxAABB = new Vector3f(0,0,0);
		//body.getAabb(minAABB, maxAABB);
		//System.out.println("Aabb Bounds: " + minAABB.toString() + " " + maxAABB.toString());
		
	}
	
	public void addLaminin(){
		hasLaminin = true;
	}
	
	public boolean isLamininCoated(){
		return hasLaminin;
	}
	
	public float getLamininDensity(){
		return lamininSurfaceDensity;
	}
	
	public CollisionShape getCollisionShape(){
		return wallShape;
	}
	
	public CMRigidBody getRigidBody(){
		return body;
	}
	
	public void updateObject(){
		//update laminin surface concentration
		lastTimeMilliseconds = currentTimeMilliseconds;
		currentTimeMilliseconds = sim.getCurrentTimeMicroseconds()/1000;
		long deltaTime = currentTimeMilliseconds - lastTimeMilliseconds;
		float deltaLaminin = lamininSurfaceDensity * lamininReactionRate;
		lamininSurfaceDensity = lamininSurfaceDensity - deltaLaminin;
	}
	
	public Vector3f getColor3Vector(){
		return new Vector3f(wallColor[0], wallColor[1], wallColor[2]);
	}
	
	public void setColor(float red, float green, float blue){
		setColor(red, green, blue, 1.0f);
	}
	
	public void setColor(float red, float green, float blue, float alpha){
		wallColor[0] = red;
		wallColor[1] = green; 
		wallColor[2] = blue;
		wallColor[3] = alpha;
	}
	
	public String toString(){
		return ("I am a wall!");
	}
	
	public void setVisible(boolean v){
		visible = v;
	}
	
	public boolean isVisible(){
		return visible;
	}
	
	public void collided(CMBioObj c, ManifoldPoint pt, long collId){
		
		if (c instanceof CMCell){
			if (sim.constraintExists(collId)){
				sim.checkInConstraints(collId);
			}
			else{
				//Let the cell handle the collision
				c.collided(this, pt, collId);
			}
		}
	}
	
	public boolean specialRender(IGL gl, Transform t){
		return false;
	}
	
	public int getID(){
		return this.id;
	}
	
	public String getType(){
		String s = "Wall";
		return s;
	}
	
	public float getMass(){
		return (0.0f);
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
	
	public String finalOutput(){
		if (finalWritten){
			return "";
		}
		finalWritten = true;
		return "Need to write final output from CMWall";
	}
}
