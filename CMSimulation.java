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
 * Terri A. Grosso, Naralys Batista, Nancy Griffeth
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
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
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
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.extras.gimpact.GImpactCollisionAlgorithm;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.DebugDrawModes;
import javax.vecmath.Vector3f;
import org.lwjgl.LWJGLException;
import static com.bulletphysics.demos.opengl.IGL.*;
import java.util.Random;
import java.util.TimeZone;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.File;

/**
 * BasicDemo is good starting point for learning the code base and porting.
 * 
 * @author jezek2
 */
public class CMSimulation extends DemoApplication{


	// maximum number of objects (test tube walls and apparatus, molecules and cells)
	private static final int NUM_MOLECULES = 1500;
	private static final int NUM_CELLS = 5;
	private final CMAssay assayType = CMAssay.TRANSWELL;
	
	private static CMTranswellChamber chamber;
	private static CMMicrofluidicChannel channel;
	private Vector3f aabbMin = new Vector3f(), aabbMax = new Vector3f();
	
	private boolean needGImpact = false;
	
	private StringBuilder buf;
	
	private ObjectArrayList<CMBioObj> modelObjects = new ObjectArrayList<CMBioObj>();
	private ObjectArrayList<CMBioObjGroup> objectGroups = new ObjectArrayList<CMBioObjGroup>();
	private ObjectArrayList<CMGenericConstraint> constraints = new ObjectArrayList<CMGenericConstraint>();
	private CMBioObjGroup channelMols;
	private BroadphaseInterface broadphase;
	private CollisionDispatcher dispatcher;
	private ConstraintSolver solver;
	private DefaultCollisionConfiguration collisionConfiguration;
	private Random random;
	private long collisionID = 0L;
	private double summaryDelay = 1000; //Minimum number of milliseconds between summary reports
	private long startTime, currentTime, lastWriteTime;
	private File dataDir = null;
	private BufferedWriter summaryFile = null, positionFile = null, concentrationFile = null;
	SimpleDateFormat summaryFormat = new SimpleDateFormat("HH:mm:ss:SSS");
	
	private float baseCameraDistance = 100;
	
	public CMSimulation(IGL gl, long seed){
		super(gl);
		random = new Random(seed);
		buf = new StringBuilder();

		SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
	    Date now = new Date();
	    String dateString = df.format(now);
		
		try {
				dataDir = new File("CM-" + dateString);
				dataDir.mkdir();
				if (dataDir != null){
					summaryFile = new BufferedWriter(new FileWriter("CM-" + dateString + "/summary.csv"));
					summaryFile.write("Time Since Sim Start\tGroup Name\tCenter of Mass\tNumber Below Mesh");
					summaryFile.newLine();
					positionFile = new BufferedWriter(new FileWriter("CM-" + dateString + "/positions.csv"));
					positionFile.write("Time Since Sim Start\tType\tID\tCOM\tLinear Velocity");
					positionFile.newLine();
					concentrationFile = new BufferedWriter(new FileWriter("CM-" + dateString + "/concentrations.csv"));
					concentrationFile.write("Time Since Sim Start\t");
					concentrationFile.newLine();
				}
	            
		} catch (IOException e) {
	        	System.out.println("Cannot generate output files!");
	        	System.out.println("IOException: " + e.toString());
		}
		
		startTime = clock.getTimeMicroseconds(); //clock is inherited from DemoApplication
		lastWriteTime = startTime;
		summaryFormat.setTimeZone(TimeZone.getTimeZone("GMT")); //To get the right time formats, need Grenwhich Mean Time
	}
	
	public CMSimulation(IGL gl) {
		this(gl, System.currentTimeMillis());
	}
	
	public void initPhysics() {
		//setCameraDistance(50f);

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
		dynamicsWorld.setGravity(new Vector3f(0f, 0.0f, 0f));
		
		if (assayType == CMAssay.TRANSWELL){
			chamber = new CMTranswellChamber(this);
			chamber.makeChamber();
			objectGroups.add(CMCell.fillSpace(this, NUM_CELLS, chamber.getMinAboveMeshVector(), chamber.getMaxAboveMeshVector(), "RPCs"));
			//objectGroups.add(CMMolecule.fillSpace(this, NUM_MOLECULES, chamber.getMinBelowMeshVector(), chamber.getMaxBelowMeshVector(), "Netrin"));
			
			/*
			CMSegmentedCell cell1 = new CMSegmentedCell(this, 5f, new Vector3f(5f, 20f, 0f), 0); 
			addBioObject(cell1);
			cell1.setCellGravity();
			
			
			CMSegmentedCell cell2 = new CMSegmentedCell(this, 5f, new Vector3f(6f, 31f, 0f), 1); 
			addBioObject(cell2);
			cell2.setCellGravity();
			
			CMSegmentedCell cell3 = new CMSegmentedCell(this, 5f, new Vector3f(6f, 10f, 0f), 2); 
			addBioObject(cell3);
			cell3.setCellGravity();
			
			CMSegmentedCell cell4 = new CMSegmentedCell(this, 5f, new Vector3f(15f, 10f, 0f), 3); 
			addBioObject(cell4);
			cell4.setCellGravity();
			
			CMSegmentedCell cell5 = new CMSegmentedCell(this, 5f, new Vector3f(-5f, 35f, -10f), 4); 
			addBioObject(cell5);
			cell5.setCellGravity();
			*/
		}
		
		else if(assayType == CMAssay.MICROFULIDIC){
			int numSourceMols = 1000;
			int numSinkMols = 0;
			channel = new CMMicrofluidicChannel(this, numSourceMols, numSinkMols);
			channelMols = CMMolecule.fillSpace(this, numSourceMols, 
				channel.getMinReservoirVector(CMMicrofluidicChannel.LEFT), 
				channel.getMaxReservoirVector(CMMicrofluidicChannel.LEFT), "ChemoAttr");
			CMBioObjGroup sinkMols = CMMolecule.fillSpace(this, numSinkMols, 
					channel.getMinReservoirVector(CMMicrofluidicChannel.RIGHT), 
					channel.getMaxReservoirVector(CMMicrofluidicChannel.RIGHT), "ChemoAttr");
			channelMols.transferGroup(sinkMols); //sinkMols is now null
			objectGroups.add(channelMols);
		}
		
		if (needGImpact){
			GImpactCollisionAlgorithm.registerAlgorithm(dispatcher);
		}
		clientResetScene();
	}
	
	@Override
	public void clientMoveAndDisplay() {
		if (lastWriteTime == startTime){
			outputSummary();
			outputPositions();
		}
		
		int numObjects = modelObjects.size();
		for (int i = 0; i < numObjects; i++){
			CMBioObj bioObj = modelObjects.getQuick(i);
			bioObj.updateObject();
			RigidBody rb = bioObj.getRigidBody();
			aabbMin.set(0, 0, 0);
			aabbMax.set(0, 0, 0);
			rb.getAabb(aabbMin, aabbMax);
			/*
			assert aabbMin.x >= -testTubeSize.x/2 : bioObj.getType() + "-"+ bioObj.getID() + "Object to left of Test Tube";
			assert aabbMin.y >= -testTubeSize.y/2 : bioObj.getType() + "-"+ bioObj.getID() + "Object  below Test Tube";
			assert aabbMin.z >= -testTubeSize.z/2 : bioObj.getType() + "-"+ bioObj.getID() + "Object in front of Test Tube";
			assert aabbMax.x <= testTubeSize.x/2 : bioObj.getType() + "-"+ bioObj.getID() + "Object to right of Test Tube";
			assert aabbMin.y <= testTubeSize.y/2 : bioObj.getType() + "-"+ bioObj.getID() + "Object above Test Tube";
			assert aabbMin.z <= testTubeSize.z/2 : bioObj.getType() + "-"+ bioObj.getID() + "Object behind Test Tube";
			*/
		}

		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		// simple dynamics world doesn't handle fixed-time-stepping
		float deltaTime = clock.getTimeMicroseconds() - currentTime;
		currentTime = clock.getTimeMicroseconds();

		// step the simulation
		if (dynamicsWorld != null) {
			dynamicsWorld.stepSimulation(deltaTime / 1000000f);
			// optional but useful: debug drawing
			dynamicsWorld.debugDrawWorld();
			
			//Detect and take care of collisions
			
			int numManifolds = dynamicsWorld.getDispatcher().getNumManifolds();
			for (int i=0;i<numManifolds;i++){
				PersistentManifold contactManifold =  dynamicsWorld.getDispatcher().getManifoldByIndexInternal(i);
				CMRigidBody objA = (CMRigidBody)contactManifold.getBody0();
				CMRigidBody objB = (CMRigidBody)contactManifold.getBody1();
				//System.out.println("A " + objA + " B: " + objB);
			
				int numContacts = contactManifold.getNumContacts();
				for (int j=0; j<numContacts; j++){
					ManifoldPoint pt = contactManifold.getContactPoint(j);
					if (pt.getDistance()<0f){
						Vector3f localA = new Vector3f(0f, 0f, 0f);
						localA.set(pt.localPointA);
						//Vector3f com = new Vector3f(0f, 0f, 0f);
						//objA.getCenterOfMassPosition(com);
						//Vector3f worldA = new Vector3f(0f, 0f, 0f);
						//pt.getPositionWorldOnA(worldA);
						//if (objA.getParent().getType().equals("Cell")){
						//	System.out.println(com + ", " + pt.localPointA + ", " + worldA);
						//}
						Vector3f localB = new Vector3f(0f, 0f, 0f);
						localB.set(pt.localPointB);
						objA.getParent().collided(objB.getParent(), localA, localB, collisionID);
						objB.getParent().collided(objA.getParent(), localB, localA,  collisionID);
						collisionID++;
						if (collisionID >= Long.MAX_VALUE){
							collisionID = 0L;
						}
						
					}
				}
			}
		}
		
		if (assayType == CMAssay.MICROFULIDIC){
			channel.updateChannel(this, channelMols);
		}
		
		//remove objects marked for removal
		int numGroups = objectGroups.size();
		for (int i = 0; i < numGroups; i++){
			CMBioObjGroup group = objectGroups.getQuick(i);
			group.cleanGroup();
		}
		System.out.println("Number of constraints: " + constraints.size());
		int index = 0;
		while(index < constraints.size()){
			CMGenericConstraint con = constraints.getQuick(index);
			con.updateTime();
			if (!con.isActive()){
				constraints.remove(index);
			}
			else{
				index++;
			}
		}
		renderme();
		if ((currentTime - lastWriteTime)/1000.0 > summaryDelay){
			outputSummary();
			outputPositions();
			lastWriteTime = currentTime;
		}
	}

	@Override
	public void displayCallback() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		renderme();

		// optional but useful: debug drawing to detect problems
		if (dynamicsWorld != null) {
			dynamicsWorld.debugDrawWorld();
		}
	}

	
	
	public void addConstraint(CMGenericConstraint c){
		dynamicsWorld.addConstraint(c);
		constraints.add(c);
	}
	
	public void removeConstraint(CMGenericConstraint c){
		dynamicsWorld.removeConstraint(c);
		constraints.remove(c);
	}
	
	public boolean constraintExists(long id){
		int numConstraints = constraints.size();
		for (int i = 0; i < numConstraints; i++){
			CMGenericConstraint constraint = constraints.getQuick(i);
			if (constraint.getID() == id){
				return (true);
			}
		}
		return (false);
	}
	
	public CMGenericConstraint getConstraint(long id){
		int numConstraints = constraints.size();
		for (int i = 0; i < numConstraints; i++){
			CMGenericConstraint constraint = constraints.getQuick(i);
			if (constraint.getID() == id){
				return constraint;
			}
		}
		return null;
	}
	@Override
	public void myinit(){
		super.myinit();
		setCameraDistance(baseCameraDistance);
		ele = 5f;
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
					if (!bioObj.specialRender(gl, m)){
						GLShapeDrawer.drawOpenGL(gl, m, bioObj.getCollisionShape(), bioObj.getColor3Vector(), getDebugMode());
					}
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
				

				//buf.setLength(0);
				//buf.append("+- shooting speed = ");
				//FastFormat.append(buf, ShootBoxInitialSpeed);
				//drawString(buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				//yStart += yIncr;

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
		
		//updateCamera();
	}
	
	
	public void addBioObject(CMBioObj obj){
		//This method adds a wall to the container
		modelObjects.add(obj);
		dynamicsWorld.addRigidBody(obj.getRigidBody());
	}
	
	public void removeBioObject(CMBioObj obj){
		//Ugh!  How do I do this?
		dynamicsWorld.removeRigidBody(obj.getRigidBody());
		modelObjects.remove(obj);
	}
	
	public float nextRandomF(){
		float x = random.nextFloat();
		//System.out.println(x);
		return x;
	}
	
	public float nextGaussianF(){
		float x = (float)random.nextGaussian();
		return x;
	}
	
	public void setNeedsGImpact(boolean b){
		needGImpact = b;
	}
	
	public void outputSummary(){
		long nowTimeMS = clock.getTimeMilliseconds()- (startTime/1000);
		Date nowTime = new Date(nowTimeMS);
		String nowString = summaryFormat.format(nowTime);
		
		int numGroups = objectGroups.size();
		
		if (assayType == CMAssay.TRANSWELL){
			for (int i = 0; i < numGroups; i++){
				CMBioObjGroup group = (CMBioObjGroup)objectGroups.getQuick(i);
				String groupName = group.getName();
				Vector3f groupCOM = group.getCenterOfMass();
		
				int numBelowMesh = group.getNumObjectsInside("RPCs", chamber.getMinBelowMeshVector(), chamber.getMaxBelowMeshVector());
				try{
					summaryFile.write(nowString + "\t" + groupName + "\t" + groupCOM.toString() + "\t" + numBelowMesh);
					summaryFile.newLine();
				}
				catch(IOException e){
					System.out.println("Error writing data to summary file.");
					System.out.println(e.toString());
				}
			}
		}
		
		if (assayType == CMAssay.MICROFULIDIC){
			try{
				concentrationFile.write(nowString + "\t");
				channel.writeConcentrationData(concentrationFile, channelMols);
				concentrationFile.newLine();
				concentrationFile.flush();
			}
			catch(IOException e){
				System.out.println("Error writing data to concentration file.");
				System.out.println(e.toString());
			}
		}
	}
	
	public void outputPositions(){
		long nowTimeMS = clock.getTimeMilliseconds()- (startTime/1000);
		Date nowTime = new Date(nowTimeMS);
		String nowString = summaryFormat.format(nowTime);
		Vector3f vel = new Vector3f();
		Vector3f com = new Vector3f();
		
		int numObjects = modelObjects.size();
		for (int i = 0; i < numObjects; i++){
			CMBioObj bioObj = modelObjects.getQuick(i);
			if (bioObj.getMass() == 0){
				continue;
			}
			RigidBody rb = bioObj.getRigidBody();
			vel.set(0,0,0);
			com.set(0,0,0);
			rb.getLinearVelocity(vel);
			rb.getCenterOfMassPosition(com);
			try{
				positionFile.write(nowString + "\t" + bioObj.getType() + "\t" + bioObj.getID() + "\t" + com + "\t" + vel);
				positionFile.newLine();
			}
			catch(IOException e){
				System.out.println("Cannot write to position file");
				System.out.println(e.toString());
			}
		}
	}
	
	public long getCurrentTimeMicroseconds(){
		return currentTime;
	}
	
	public void setBaseCameraDistance(float d){
		baseCameraDistance = d;
	}
	
	public void wrapUp(){
		try{
			summaryFile.flush();
			summaryFile.close();
			positionFile.flush();
			positionFile.close();
			concentrationFile.flush();
			concentrationFile.close();
		}
		catch(IOException e){
			System.out.println("Unable to close output files");
			System.out.println(e.toString());
		}
	}
	
	public static void main(String[] args) throws LWJGLException {
		CMSimulation sim = new CMSimulation(LWJGL.getGL());
		sim.initPhysics();
		sim.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));

		CMLWJGL.main(args, 800, 600, "Cell Simulation", sim);
	}
	
}