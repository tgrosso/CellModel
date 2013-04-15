/** This is an extention of the jbullet basic demo
 * designed to model cellular migration of retinal progenitor cells
 * 
 * 
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 
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
 * File: CMSimulation.java
 * Apr 10, 2013 1:44:54 PM
 */

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
import com.bulletphysics.demos.opengl.GLShapeDrawer;
import com.bulletphysics.demos.opengl.FastFormat;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.DebugDrawModes;
import javax.vecmath.Vector3f;
import org.lwjgl.LWJGLException;
import static com.bulletphysics.demos.opengl.IGL.*;
import java.util.Random;

/**
 * BasicDemo is good starting point for learning the code base and porting.
 * 
 * @author jezek2
 */
public class CMSimulation extends DemoApplication {


	// maximum number of objects (test tube walls and apparatus, molecules and cells)
	private static final int NUM_WALLS = 6;
	private static final int NUM_MESH_BOXES = 100;
	private static final int NUM_MOLECULES = 1000;
	private static final int NUM_CELLS = 100;
	private static final int MAX_PROXIES = NUM_WALLS + NUM_MESH_BOXES + NUM_MOLECULES + NUM_CELLS;
	
	
	private static float wallThick = 2f;
	private static float meshThick = 6f;
	private static float pore_density = 100000;
	private static float pore_diameter = 12; //Actually 8 but need room for 10micrometer cells
	private static float well_depth = 40;
	private static float inset_depth = 60;
	private static Vector3f testTubeSize = new Vector3f(80f, well_depth + inset_depth + meshThick, 80f);
	
	private StringBuilder buf;
	
	
	// keep the collision shapes, for deletion/cleanup
	private ObjectArrayList<CollisionShape> collisionShapes = new ObjectArrayList<CollisionShape>();
	private ObjectArrayList<CMBioObj> modelObjects = new ObjectArrayList<CMBioObj>();
	private BroadphaseInterface broadphase;
	private CollisionDispatcher dispatcher;
	private ConstraintSolver solver;
	private DefaultCollisionConfiguration collisionConfiguration;
	private Random random;
	
	public CMSimulation(IGL gl, long seed){
		super(gl);
		random = new Random(seed);
		buf = new StringBuilder();
	}
	
	public CMSimulation(IGL gl) {
		this(gl, System.currentTimeMillis());
	}
	
	@Override
	public void clientMoveAndDisplay() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		int numObjects = modelObjects.size();
		for (int i = 0; i < numObjects; i++){
			CMBioObj bioObj = modelObjects.getQuick(i);
			bioObj.updateObject(random);
		}

		// simple dynamics world doesn't handle fixed-time-stepping
		float ms = getDeltaTimeMicroseconds();

		// step the simulation
		if (dynamicsWorld != null) {
			dynamicsWorld.stepSimulation(ms / 1000000f);
			// optional but useful: debug drawing
			dynamicsWorld.debugDrawWorld();
		}

		renderme();

		//glFlush();
		//glutSwapBuffers();
	}

	@Override
	public void displayCallback() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		renderme();

		// optional but useful: debug drawing to detect problems
		if (dynamicsWorld != null) {
			dynamicsWorld.debugDrawWorld();
		}

		//glFlush();
		//glutSwapBuffers();
	}

	public void initPhysics() {
		setCameraDistance(50f);

		// collision configuration contains default setup for memory, collision setup
		collisionConfiguration = new DefaultCollisionConfiguration();

		// use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new CollisionDispatcher(collisionConfiguration);

		broadphase = new DbvtBroadphase();

		// the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
		solver = sol;
		
		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

		//Simulation takes place in fluid.  For now, no gravity
		dynamicsWorld.setGravity(new Vector3f(0f, 0f, 0f));

		addBioObject(new CMWall(testTubeSize.x, wallThick, testTubeSize.z, new Vector3f(0f, -testTubeSize.y/2, 0f)));
		addBioObject(new CMWall(testTubeSize.x, wallThick, testTubeSize.z, new Vector3f(0f, testTubeSize.y/2, 0f)));
		addBioObject(new CMWall(wallThick, testTubeSize.y, testTubeSize.z, new Vector3f(-testTubeSize.x/2, 0f, 0f)));
		addBioObject(new CMWall(wallThick, testTubeSize.y, testTubeSize.z, new Vector3f(testTubeSize.x/2, 0f, 0f)));
		addBioObject(new CMWall(testTubeSize.x, testTubeSize.y, wallThick, new Vector3f(0f, 0f, testTubeSize.z/2)));
		
		CMWall front = new CMWall(testTubeSize.x, testTubeSize.y, wallThick, new Vector3f(0f, 0f, -testTubeSize.z/2));
		front.setVisible(false);
		addBioObject(front);

		addMesh(testTubeSize.x, testTubeSize.z, meshThick, pore_density, pore_diameter, well_depth);
		addBioObject(new CMCell(this, new Vector3f(0f, 0f, 0f)));
		
		CMMolecule.fillSpace(this, NUM_MOLECULES, new Vector3f(-testTubeSize.x/2, -testTubeSize.y/2, -testTubeSize.z/2), new Vector3f(testTubeSize.x/2, well_depth-testTubeSize.y/2, testTubeSize.z/2));

		clientResetScene();
	}
	
	@Override
	public void myinit(){
		super.myinit();
		//gl.glClearColor(0f, 0f, 0f, 1f);
		setCameraDistance(90);
		updateCamera();
	}
	
	
	@Override
	public void renderme() {
		updateCamera();

		Transform m = new Transform();
		if (dynamicsWorld != null) {
			int numObjects = modelObjects.size();
			for (int i = 0; i < numObjects; i++){
				CMBioObj bioObj = modelObjects.getQuick(i);
				if (bioObj.isVisible()){
					RigidBody rb = bioObj.getRigidBody();
					DefaultMotionState motionState = (DefaultMotionState)rb.getMotionState();
					m.set(motionState.graphicsWorldTrans);
					GLShapeDrawer.drawOpenGL(gl, m, bioObj.getCollisionShape(), bioObj.getColor3Vector(), getDebugMode());
				}
			}
				

			float xOffset = 10f;
			float yStart = 20f;
			float yIncr = 20f;

			gl.glDisable(GL_LIGHTING);
			gl.glColor3f(1f, 1f, 1f);

			if ((debugMode & DebugDrawModes.NO_HELP_TEXT) == 0) {
				setOrthographicProjection();

				yStart = showProfileInfo(xOffset, yStart, yIncr);

				String s = "mouse to interact";
				drawString(s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				// JAVA NOTE: added
				s = "LMB=drag"; // RMB=shoot box, MIDDLE=apply impulse";
				drawString(s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;
				
				s = "space to reset";
				drawString(s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = "cursor keys and z,x to navigate";
				drawString(s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = "i to toggle simulation, s single step";
				drawString(s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = "q to quit";
				drawString(s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = "h to toggle help text";
				drawString(s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;
				yStart += yIncr;
				

				buf.setLength(0);
				buf.append("+- shooting speed = ");
				FastFormat.append(buf, ShootBoxInitialSpeed);
				drawString(buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				if (getDynamicsWorld() != null) {
					buf.setLength(0);
					buf.append("# objects = ");
					FastFormat.append(buf, getDynamicsWorld().getNumCollisionObjects());
					drawString(buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
					yStart += yIncr;
					
					buf.setLength(0);
					buf.append("# pairs = ");
					FastFormat.append(buf, getDynamicsWorld().getBroadphase().getOverlappingPairCache().getNumOverlappingPairs());
					drawString(buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
					yStart += yIncr;

				}

				// JAVA NOTE: added
				int free = (int)Runtime.getRuntime().freeMemory();
				int total = (int)Runtime.getRuntime().totalMemory();
				buf.setLength(0);
				buf.append("heap = ");
				FastFormat.append(buf, (float)(total - free) / (1024*1024));
				buf.append(" / ");
				FastFormat.append(buf, (float)(total) / (1024*1024));
				buf.append(" MB");
				drawString(buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;
				
				resetPerspectiveProjection();
			}

			gl.glEnable(GL_LIGHTING);
		}
		
		updateCamera();
	}
	
	public void addMesh(float width, float depth, float thickness, float p_density, float p_diameter, float height){
		//p_density is pore density - pores/cm^2
		//p_diameter is pore diameter in micrometers
		//although the diameter is given, for now the pores are square
		//In addition, the number of pores will be rows * cols - even if that's a few too many
		//The Mesh's x and z centers are the origin. The y center is height
		
		//find number of pores 
		double numPores = Math.ceil(p_density * width * depth * Math.pow(10, -8));
		
		//How many rows and columns will we need? R*C>=numPores and C/R approx w/l
		//So R^2 >= Nl/w
		int numRows = (int)Math.ceil(Math.sqrt(numPores * depth / width));
		int numCols = (int)Math.ceil(numPores/numRows);
		float colWidth = width/numCols;
		float rowHeight = depth/numRows;
		float z_space = (rowHeight - p_diameter)/2;
		float x_space = (colWidth - p_diameter)/2;
		//Add row spacers:
		for (int i = 0; i < numRows; i++){
			Vector3f wall_origin = new Vector3f(0f, (height - testTubeSize.y/2 - thickness/2), (depth / 2 - i * (z_space*2+p_diameter) - z_space/2));
			CMWall wall = new CMWall(width, thickness, z_space, wall_origin);
			wall.setColor(.7f, .7f, .9f);
			addBioObject(wall);
			wall_origin = new Vector3f(0f, (height - testTubeSize.y/2 - thickness/2), (depth / 2 - i * (z_space*2+p_diameter) -(z_space+p_diameter)- z_space/2 ));
			wall = new CMWall(width, thickness, z_space, wall_origin);
			wall.setColor(.7f, .7f, .9f);
			addBioObject(wall);
		}
		
		for (int i = 0; i < numRows; i++){
			float y_value = height - testTubeSize.y/2 - thickness/2;
			float z_value = depth/2 - z_space - p_diameter/2 - i * (2 * z_space + p_diameter);
			for (int j = 0; j < numCols; j++){
				float x_value = width/2 - (j * (2 * x_space + p_diameter)) - x_space/2;
				CMWall wall = new CMWall(x_space, thickness, p_diameter, new Vector3f(x_value, y_value, z_value));
				wall.setColor(.7f, .7f, .9f);
				addBioObject(wall);
				x_value = width/2 - (j * (2 * x_space + p_diameter)) - x_space/2-p_diameter-x_space;
				wall = new CMWall(x_space, thickness, p_diameter, new Vector3f(x_value, y_value, z_value));
				wall.setColor(.7f, .7f, .9f);
				addBioObject(wall);
			}
		}
	}
	
	public void addBioObject(CMBioObj obj){
		//This method adds a wall to the container
		modelObjects.add(obj);
		collisionShapes.add(obj.getCollisionShape());
		dynamicsWorld.addRigidBody(obj.getRigidBody());
	}
	
	public float nextRandomF(){
		return random.nextFloat();
	}
	
	public static void main(String[] args) throws LWJGLException {
		CMSimulation sim = new CMSimulation(LWJGL.getGL());
		sim.initPhysics();
		sim.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));

		LWJGL.main(args, 800, 600, "Cell Simulation", sim);
	}
	
}