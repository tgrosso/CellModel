/**
 * Terri A. Grosso
 * Package: cellModel
 * File: CMWall.java
 * Apr 2, 2013 4:43:30 PM
 */
package cellModel;

/**
 * @author tagsit
 *
 */

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;

import java.nio.FloatBuffer;
import java.util.Random;
import javax.vecmath.Vector3f;

import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;


public class CMWall implements CMBioObj{
	private RigidBody body;
	private BoxShape wallShape;
	private Vector3f origin;
	private float[] wallColor = {.4f, 0.2f, 0.2f, 1f};
	private float width, height, depth;
	private static FloatBuffer buffer = BufferUtils.createFloatBuffer(16);
	private boolean visible = true;
	
	public CMWall(float w, float h, float d, Vector3f o){
		float mass = 0;
		width = w;
		height = h;
		depth = d;
		origin = o;
		Transform t = new Transform();
		t.setIdentity();
		t.origin.set(origin);
		
		Vector3f localInertia = new Vector3f(0, 0, 0);
		wallShape = new BoxShape(new Vector3f(width/2, height/2, depth/2));
		DefaultMotionState motionState = new DefaultMotionState(t);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, wallShape, localInertia);
		body = new RigidBody(rbInfo);
		
		//Vector3f minAABB = new Vector3f(0,0,0);
		//Vector3f maxAABB = new Vector3f(0,0,0);
		//body.getAabb(minAABB, maxAABB);
		//System.out.println("Aabb Bounds: " + minAABB.toString() + " " + maxAABB.toString());
		
	}
	
	public CollisionShape getCollisionShape(){
		return wallShape;
	}
	
	public RigidBody getRigidBody(){
		return body;
	}
	
	public void updateObject(Random r){
		//Object doesn't move.  Nothing to update
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
	
	public void setVisible(boolean v){
		visible = v;
	}
	
	public boolean isVisible(){
		return visible;
	}
}
