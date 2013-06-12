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
 * File: CMReceptorRaft.java
 * June 9, 2013 7:44:54 PM
 */

package cellModel;

import java.util.Random;
import javax.vecmath.Vector3f;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;


public class CMReceptorRaft implements CMBioObj{
	private static float radius = .5f;
	private static float height = .1f;
	private static float mass = (float)(height * Math.PI * radius * radius);
	private static float forceMagnitude = 30f;
	private static CylinderShape raftShape = new CylinderShape(new Vector3f(radius, height/2, 0));
	private CMRigidBody body;
	private float[] raftColor = {0.0f, 0.0f, 1.0f, 1.0f};
	private boolean visible;
	private static int raft_ids = 0;
	private int id;
	private String type = "Raft";
	private CMCell parent;
	
	public CMReceptorRaft(CMCell p, Vector3f o){
		parent = p;
		
		Transform trans = new Transform();
		trans.setIdentity();
		trans.origin.set(o);
		
		Vector3f localInertia = new Vector3f(0, 0, 0);
		raftShape.calculateLocalInertia(mass, localInertia);

		DefaultMotionState motionState = new DefaultMotionState(trans);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, raftShape, localInertia);
		body = new CMRigidBody(rbInfo, this);
	}

	
	
	public CollisionShape getCollisionShape(){
		return raftShape;
	}
	
	public RigidBody getRigidBody(){
		return body;
	}
	
	public void updateObject(Random r){
		
	}
	
	public Vector3f getColor3Vector(){
		return new Vector3f(raftColor[0], raftColor[1], raftColor[2]);
	}
	
	public void setVisible(boolean v){
		visible = v;
	}
	
	public boolean isVisible(){
		return visible;
	}
	
	public void collided(CMBioObj c, Vector3f point){
		
	}
	
	public void addConstraint(CMGenericConstraint c){
		
	}
	
	public void removeConstraint(CMGenericConstraint c){
		
	}
	
	public int getID(){
		return id;
	}
	
	public float getMass(){
		return mass;
	}
	
	public String getType(){
		return type;
	}
}
