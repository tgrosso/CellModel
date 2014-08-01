package cellModel;

import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.demos.opengl.DemoApplication;
import com.bulletphysics.demos.opengl.GLDebugDrawer;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.demos.opengl.LWJGL;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor;
import com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor;
import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Vector3f;

import org.lwjgl.LWJGLException;

import static com.bulletphysics.demos.opengl.IGL.*;

/**
 *
 * @author jezek2
 */
public class GenericConstraintTest extends DemoApplication {

	Generic6DofConstraint gen6dof;
	RigidBody boxA;

	public GenericConstraintTest(IGL gl) {
		super(gl);
	}

	public void initPhysics() {
		// Setup the basic world
		DefaultCollisionConfiguration collision_config = new DefaultCollisionConfiguration();

		CollisionDispatcher dispatcher = new CollisionDispatcher(collision_config);

		Vector3f worldAabbMin = new Vector3f(-10000, -10000, -10000);
		Vector3f worldAabbMax = new Vector3f(10000, 10000, 10000);
		//BroadphaseInterface overlappingPairCache = new AxisSweep3(worldAabbMin, worldAabbMax);
		//BroadphaseInterface overlappingPairCache = new SimpleBroadphase();
		BroadphaseInterface overlappingPairCache = new DbvtBroadphase();

		//#ifdef USE_ODE_QUICKSTEP
		//btConstraintSolver* constraintSolver = new OdeConstraintSolver();
		//#else
		ConstraintSolver constraintSolver = new SequentialImpulseConstraintSolver();
		//#endif

		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, overlappingPairCache, constraintSolver, collision_config);

		dynamicsWorld.setGravity(new Vector3f(0f, -30f, 0f));

		dynamicsWorld.setDebugDrawer(new GLDebugDrawer(gl));

		RigidBody ground, boxB;
		Transform groundTransform, transformA, transformB;
		// Setup a big ground box
		{
			CollisionShape groundShape = new BoxShape(new Vector3f(200f, 10f, 200f));
			groundTransform = new Transform();
			groundTransform.setIdentity();
			groundTransform.origin.set(0f, -10f, 0f);
			ground = localCreateRigidBody(0f, groundTransform, groundShape);
		}
		
		//Make a box to bind to the ground
		{
			CollisionShape boxAShape = new BoxShape(new Vector3f(2f, 2f, 2f));
			transformA= new Transform();
			transformA.setIdentity();
			transformA.origin.set(-3f, 9f, 0f);
			boxA = localCreateRigidBody(0.1f, transformA, boxAShape);
			boxA.setFriction(1.0f);
		}
		
		//Make a second box to compare
		
		{
			CollisionShape boxBShape = new BoxShape(new Vector3f(2f, 2f, 2f));
			transformB= new Transform();
			transformB.setIdentity();
			transformB.origin.set(3f, 9f, 0f);
			boxB = localCreateRigidBody(0.1f, transformB, boxBShape);
		}
		
		Transform aTrans = new Transform();
		Transform groundTrans = new Transform();
		aTrans.setIdentity();
		groundTrans.setIdentity();
		aTrans.origin.set(new Vector3f(0, -2, 0));
		groundTrans.origin.set(new Vector3f(-3, 10, 0));
		
		gen6dof = new Generic6DofConstraint(ground, boxA, groundTrans, aTrans, true);
		dynamicsWorld.addConstraint(gen6dof);
		
		//Play with these to understand the way the constraint works
		gen6dof.setLimit(0, 0f, 0f);
		gen6dof.setLimit(1, 0f, 1f);
		gen6dof.setLimit(2, 0f, 0f);
		gen6dof.setLimit(3, 0f, 0f);
		gen6dof.setLimit(4, 0f, 0f);
		gen6dof.setLimit(5, 0f, 0f);
		
		TranslationalLimitMotor tlm = gen6dof.getTranslationalLimitMotor();
		tlm.restitution = 1.3f;
		clientResetScene();
	}
	
	@Override
	public void clientMoveAndDisplay() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// simple dynamics world doesn't handle fixed-time-stepping
		float ms = getDeltaTimeMicroseconds();
		float minFPS = 1000000f / 60f;
		if (ms > minFPS) {
			ms = minFPS;
		}

		if (dynamicsWorld != null) {
			dynamicsWorld.stepSimulation(ms / 1000000.f);
			// optional but useful: debug drawing
			dynamicsWorld.debugDrawWorld();
		}

		TranslationalLimitMotor tlm = gen6dof.getTranslationalLimitMotor();
		//System.out.println(tlm.accumulatedImpulse);
		renderme();

		//glFlush();
		//glutSwapBuffers();
	}
	
	@Override
	public void renderme(){
		super.renderme();
		Transform ta = new Transform(), tb = new Transform();
		gen6dof.getCalculatedTransformA(ta);
		gen6dof.getCalculatedTransformB(tb);
		gl.glBegin(GL_LINES);
		gl.glColor3f(0f, 0f, 0f);
		gl.glVertex3f(ta.origin.x, ta.origin.y, ta.origin.z);
		gl.glVertex3f(tb.origin.x, tb.origin.y, tb.origin.z);
		gl.glEnd();
	}

	@Override
	public void displayCallback() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if (dynamicsWorld != null) {
			dynamicsWorld.debugDrawWorld();
		}

		renderme();

		//glFlush();
		//glutSwapBuffers();
		
	}
	
	public void outputData(){
		int x = 1;
		System.out.println("Restitution: " + boxA.getRestitution());
		RotationalLimitMotor rlm = gen6dof.getRotationalLimitMotor(x);
		System.out.println("Rotational Limit Motor" + x);
		System.out.println("accumulatedImpulse: " + rlm.accumulatedImpulse);
		System.out.println("bounce: " + rlm.bounce);
		System.out.println("currentLimit: " + rlm.currentLimit);
		System.out.println("currentLimitError: " + rlm.currentLimitError);
		System.out.println("damping: " + rlm.damping);
		System.out.println("enableMotor: " + rlm.enableMotor);
		System.out.println("ERP: " + rlm.ERP);
		System.out.println("hiLimit: " + rlm.hiLimit);
		System.out.println("limitSoftness: " + rlm.limitSoftness);
		System.out.println("loLimit: " + rlm.loLimit);
		System.out.println("maxLimitForce: " + rlm.maxLimitForce);
		System.out.println("maxMotorForce: " + rlm.maxMotorForce);
		System.out.println("targetVelocity: " + rlm.targetVelocity);
		System.out.println("");
		TranslationalLimitMotor tlm = gen6dof.getTranslationalLimitMotor();
		System.out.println("Translational Limit Motor");
		System.out.println("accumulatedImpulse: " + tlm.accumulatedImpulse);
		System.out.println("damping: " + tlm.damping);
		System.out.println("limitSoftness: " + tlm.limitSoftness);
		System.out.println("lowerLimit: " + tlm.lowerLimit);
		System.out.println("restitution: " + tlm.restitution);
		System.out.println("upperLimit: " + tlm.upperLimit);
	}

	@Override
	public void keyboardCallback(char key, int x, int y, int modifiers) {
		switch(key){
			case 'e':
				outputData();
				break;
			default:
				super.keyboardCallback(key, x, y, modifiers);
				break;
		}
	}

	public static void main(String[] args) throws LWJGLException {
		GenericConstraintTest demoApp = new GenericConstraintTest(LWJGL.getGL());
		demoApp.initPhysics();
		demoApp.setCameraDistance(10f);

		LWJGL.main(args, 800, 600, "Joint 6DOF - Sequencial Impulse Solver", demoApp);
	}
	
}