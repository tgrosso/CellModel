package cellModel;

import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import javax.vecmath.Vector3f;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import org.lwjgl.BufferUtils;
import java.nio.FloatBuffer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Random;

import org.lwjgl.opengl.GL11;
import org.lwjgl.util.glu.Sphere;

public class CMMolecule implements CMBioObj{
	private static float radius = .1f;
	private static float mass = .1f;
	private static float maxVelChange = .5f;
	private static int mol_ids = 0;
	private int id;
	private Vector3f origin;
	private static SphereShape molShape = new SphereShape(radius);
	private RigidBody body;
	private static Sphere drawSphere = new Sphere();
	private static float[] molColor = {1.0f, 1.0f, 0.0f, 1.0f};
	protected float cameraDistance = 20f;
	private boolean visible = true;
	
	public CMMolecule(CMSimulation sim, Vector3f o){
		this.origin = o;
		
		Transform t = new Transform();
		t.setIdentity();
		t.origin.set(origin);
		
		Vector3f localInertia = new Vector3f(0, 0, 0);
		molShape.calculateLocalInertia(mass, localInertia);

		DefaultMotionState motionState = new DefaultMotionState(t);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, molShape, localInertia);
		body = new RigidBody(rbInfo);
		float magnitude = sim.nextRandomF() * maxVelChange;
		float hor_angle = sim.nextRandomF() * 360;
		float ver_angle = sim.nextRandomF() * 360;
		float y_mag = (float)(magnitude * Math.sin(ver_angle));
		double h = magnitude * Math.cos(ver_angle);
		float x_mag = (float)(Math.cos(hor_angle)* h);
		float z_mag = (float)(Math.sin(hor_angle) * h);
		//body.setLinearVelocity(new Vector3f(x_mag, y_mag, z_mag));
		body.setLinearVelocity(new Vector3f(3, 3, 3));
		
		this.id = mol_ids;
		mol_ids++;
	}
	
	
	public static void fillSpace(CMSimulation sim, int numMol, Vector3f minP, Vector3f maxP){
		//Will randomly spread the molecules throughout the space
		//System.out.println(minP.x + ", " + minP.y + ", " + minP.z);
		//System.out.println(maxP.x + ", " + maxP.y + ", " + maxP.z);
		
		float x_len = maxP.x - minP.x;
		float y_len = maxP.y - minP.y;
		float z_len = maxP.z - minP.z;
		
		for (int i = 0; i < numMol; i++){
			float randX = x_len * sim.nextRandomF();
			float randY = y_len * sim.nextRandomF();
			float randZ = z_len * sim.nextRandomF();
			sim.addBioObject(new CMMolecule(sim, new Vector3f(randX+minP.x, randY+minP.y, randZ+minP.z)));
		}
		
	}
	
	public CollisionShape getCollisionShape(){
		return molShape;
	}
	
	public RigidBody getRigidBody(){
		return body;
	}
	
	public void updateObject(Random r){
		//apply a random force to the molecule
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
		String s = "I am molecule " + this.id;
		return s;
	}
}
