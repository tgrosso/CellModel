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
import java.io.InputStream;
import java.nio.ByteBuffer;

/**
 * BasicDemo is good starting point for learning the code base and porting.
 * 
 * @author jezek2
 */
public class CMSimulation extends DemoApplication{
	//Unchanging variables
	
	private int width = 900, height = 600;

	// maximum number of objects (test tube walls and apparatus, molecules and cells)
	private static final int NUM_MOLECULES = 1500;
	private static final int NUM_CELLS = 1;
	private final CMAssay assayType = CMAssay.MICROFLUIDIC;
	private static SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
	private static SimpleDateFormat summaryFormat = new SimpleDateFormat("HH:mm:ss:SSS");
	
	
	private int viewingProtein = -1;
	
	private static CMTranswellChamber chamber;
	private static CMMicrofluidicChannel channel;
	private Vector3f aabbMin = new Vector3f(), aabbMax = new Vector3f();
	
	private boolean needGImpact = false;
	
	private StringBuilder buf;
	
	private ObjectArrayList<CMBioObj> modelObjects = new ObjectArrayList<CMBioObj>();
	private ObjectArrayList<CMBioObjGroup> objectGroups = new ObjectArrayList<CMBioObjGroup>();
	private ObjectArrayList<CMConstraint> constraints = new ObjectArrayList<CMConstraint>();
	private ObjectArrayList<CMMembraneProtein> proteins = new ObjectArrayList<CMMembraneProtein>();
	private boolean proteinsAdded = false;
	private BroadphaseInterface broadphase;
	private CollisionDispatcher dispatcher;
	private ConstraintSolver solver;
	private DefaultCollisionConfiguration collisionConfiguration;
	private Random random;
	private long collisionID = 0L;
	private double summaryDelay = 50000; //Minimum number of microseconds between summary reports
	private long startTime, currentTime, lastWriteTime; //Units microseconds
	private long deltaTime = 0;
	private boolean firstOutputWritten = false; 
	private long imageDelay = 1000000/30; //microseconds beteen images => 30 frames per second
	private long lastImageTime; // last time an images was generated in microseconds
	private File dataDir = null, solverDir = null, templateDir = null;
	private BufferedWriter groupData = null, cellData = null, concentrationData = null, membraneData = null, pdeData = null, logFile = null;
	
	private CMConcentrationSolver concentrationSolver;
	private long timeBetweenFrames = 60*1000000; //microseconds per frame
	private long lastImageWritten; //Units microseconds
	private CMImageGenerator imageGenerator;
	
	private float baseCameraDistance = 20;
	private boolean finished = false;
	
	private CMSimGenerator generator = null;
	
	public CMSimulation(IGL gl, CMSimGenerator gen){
		super(gl);
		
		generator = gen;
		random = new Random(generator.seed);
		buf = new StringBuilder();

		summaryDelay = (long)(generator.secBetweenOutput * 1000000);
		summaryFormat.setTimeZone(TimeZone.getTimeZone(generator.timeZone)); //To get the right time formats, need Grenwhich Mean Time

		//Create output files
		
		try {	
				if (generator.baseFile != null){
					String baseName = generator.baseFile + File.separator;
					groupData = new BufferedWriter(new FileWriter(baseName + "groupData.csv"));
					groupData.write("Time Since Sim Start\tGroup Name\tCenter of Mass\tNumber Below Mesh");
					groupData.newLine();
					cellData = new BufferedWriter(new FileWriter(baseName + "cellData.csv"));
					cellData.write("Time Since Sim Start\tType\tID\tCenter of Mass-x\tCenter of Mass-y\tCenter of Mass-z\tLinVelocity x\tLinVelocity y\tLinVelocity z");
					cellData.newLine();
					concentrationData = new BufferedWriter(new FileWriter(baseName + "ligandData.csv"));
					concentrationData.write("Time Since Sim Start\tExperimental Time\t");
					membraneData = new BufferedWriter(new FileWriter(baseName + "membraneProData.csv"));
					membraneData.write("Time Since Sim Start\tProtein\tCellID\tSegmentID\tBoundReceptors\tUnboundReceptors\tLigand Concentration");
					membraneData.newLine();
					if (generator.generateImages){
						File imageFile = new File(generator.baseFile, "image");
						imageFile.mkdirs();
						imageGenerator = new CMImageGenerator(imageFile, width, height, 4);
					}
				}
				else {
					System.err.println("No base file given!");
					System.err.println("Cannot generate output files!");
				}
	            
		} catch (IOException e) {
	        	System.err.println("Cannot generate output files!");
	        	System.err.println("IOException: " + e.toString());
		}
		
		
		//Generate solutions to microfluidic channel if sink < source
		//TODO check for the solver in the pdepe directory
		if (generator.sinkConc <= generator.sourceConc){
			concentrationSolver = new CMConcentrationSolver(generator.baseFile.toString(), generator.pdepeDirectory, 1, 13000, 90000, generator.timeToSteadyState, generator.sourceConc, generator.sinkConc);
			//TODO What exception is thrown if the pdepe directory doesn't work?
			//Can we check to see if one already exists?
		}
		else if (generator.sinkConc > generator.sourceConc){
			System.out.println("Cannot have source Concentration less than sink Concentration! Exitng.");
			finished = true;
		}
		
		//TODO - how do we put this into the generator?
		float[] proColor0 = new float[3];
		float[] proColor1 = new float[3];
		proColor0[0] = 1f; proColor0[1] = 0f; proColor0[2]=0f;
		//proColor1[0] = 1f; proColor1[1] = 1f; proColor1[2]=0f;
		proteins.add(new CMEGFR(this, proColor0));
		//proteins.add(new CMIntegrin(this, proColor1));
		viewingProtein = 0;
		
		//Set initial times
		startTime = clock.getTimeMicroseconds(); //clock is inherited from DemoApplication
		currentTime = 0;
		lastWriteTime = 0;
		lastImageWritten = 0;
		lastImageTime = 0; //last time an image was captured
	}
	
	public void initPhysics() {

		// collision configuration contains default setup for memory, collision setup
		collisionConfiguration = new DefaultCollisionConfiguration();

		// use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new CollisionDispatcher(collisionConfiguration);

		broadphase = new DbvtBroadphase();

		// the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
		solver = sol;
		
		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

		//Simulation takes place in fluid.  Gravity is set for individual cells
		dynamicsWorld.setGravity(new Vector3f(0f, 0f, 0f));
		
		if(assayType == CMAssay.MICROFLUIDIC){
			float distFromSource = generator.distFromSource;
			channel = new CMMicrofluidicChannel(this, distFromSource, concentrationSolver);
			channel.writeConcentrationData(concentrationData, currentTime, true);
			
			CMBioObjGroup microCells = CMSegmentedCell.randomFillSurface(this, generator.numCells, 10f, 1, channel.getMinChannelVector(), channel.getMaxChannelVector(), "RPC", true);
			objectGroups.add(microCells);
		}
		
		
		if (needGImpact){
			GImpactCollisionAlgorithm.registerAlgorithm(dispatcher);
		}
		clientResetScene();
	}
	
	@Override
	public void clientMoveAndDisplay() {
		if (!firstOutputWritten){
			//System.out.println("Writing first time. Current time " + currentTime);
			outputSummary();
			outputCellData();
			outputMembraneData();
			firstOutputWritten = true;
			currentTime = startTime;
			//System.out.println("First data written. Current time " + currentTime);
		}
		
		int numObjects = modelObjects.size();
		for (int i = 0; i < numObjects; i++){
			CMBioObj bioObj = modelObjects.getQuick(i);
			bioObj.updateObject();
		}

		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		// simple dynamics world doesn't handle fixed-time-stepping
		long newTime = clock.getTimeMicroseconds();
		deltaTime = newTime - currentTime;
		currentTime = newTime - startTime;
		if (currentTime/1000/1000 > generator.endTime){
			System.out.println("Time is up. " + currentTime);
			finished = true;
		}

		// step the simulation
		if (dynamicsWorld != null) {
			dynamicsWorld.stepSimulation(deltaTime / 1000000f);
			// optional but useful: debug drawing
			dynamicsWorld.debugDrawWorld();
			
			//Detect and take care of collisions
			
			int numManifolds = dynamicsWorld.getDispatcher().getNumManifolds();
			for (int i=0;i<numManifolds;i++){
				//System.out.println(collisionID);
				PersistentManifold contactManifold =  dynamicsWorld.getDispatcher().getManifoldByIndexInternal(i);
				CMRigidBody objA = (CMRigidBody)contactManifold.getBody0();
				CMRigidBody objB = (CMRigidBody)contactManifold.getBody1();
				int numContacts = contactManifold.getNumContacts();
				for (int j=0; j<numContacts; j++){
					ManifoldPoint pt = contactManifold.getContactPoint(j);
					if (pt.getDistance()<.05f){ //to accont for length of laminin and integrin
						objA.getParent().collided(objB.getParent(), pt, collisionID);
						objB.getParent().collided(objA.getParent(), pt, collisionID);
						collisionID++;
						
						if (collisionID >= Long.MAX_VALUE){
							collisionID = 0L;
						}
						
					}
				}
			}
		}
		
		//remove objects marked for removal
		int numGroups = objectGroups.size();
		for (int i = 0; i < numGroups; i++){
			CMBioObjGroup group = objectGroups.getQuick(i);
			group.cleanGroup();
		}

		int index = 0;
		while(index < constraints.size()){
			CMConstraint con = constraints.getQuick(index);
			con.updateTime();
			if (!con.isActive()){
				constraints.remove(index);
			}
			else{
				index++;
				//CMRigidBody objA = (CMRigidBody)con.getConstraint().getRigidBodyA();
				//objA.getParent().bind();
				//CMRigidBody objB = (CMRigidBody)con.getConstraint().getRigidBodyB();
				//objB.getParent().bind();
			}
		}

		renderme();
		if ((currentTime - lastWriteTime) > summaryDelay){
			outputSummary();
			outputCellData();
			outputMembraneData();
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

	
	
	public void addConstraint(CMConstraint c){
		dynamicsWorld.addConstraint(c.getConstraint());
		constraints.add(c);
	}
	
	public void removeConstraint(CMConstraint c){
		dynamicsWorld.removeConstraint(c.getConstraint());
		constraints.remove(c);
	}
	
	public boolean constraintExists(long id){
		int numConstraints = constraints.size();
		for (int i = 0; i < numConstraints; i++){
			CMConstraint constraint = constraints.getQuick(i);
			if (constraint.getID() == id){
				return (true);
			}
		}
		return (false);
	}
	
	public CMConstraint getConstraint(long id){
		int numConstraints = constraints.size();
		for (int i = 0; i < numConstraints; i++){
			CMConstraint constraint = constraints.getQuick(i);
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
		ele = 0f;
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
			gl.glColor3f(0f, 0f, 0f);
			
			setOrthographicProjection();
			String s = "Z to zoom in";
			drawString(s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
			resetPerspectiveProjection();

			gl.glEnable(GL_LIGHTING);
		}
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
	
	public int getNumProteins(){
		return proteins.size();
	}
	
	public CMMembraneProtein getProtein(int index){
		return proteins.getQuick(index);
	}
	
	public int getViewingProtein(){
		return viewingProtein;
	}
	
	public float getLigandConcentration(float x_value, long time){
		//time is in milliseconds
		if (assayType == CMAssay.MICROFLUIDIC){
			float dfm = channel.getDistanceFromMinimum(x_value);
			return (channel.getConcentration(dfm, time));
		}
		return 0f;
	}
	
	public void outputSummary(){
		//System.out.println("Output Suumary");
		long nowTimeMS = (currentTime)/1000;
		//System.out.println("Summary nowTime " + nowTimeMS);
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
					groupData.write(nowString + "\t" + groupName + "\t" + groupCOM.toString() + "\t" + numBelowMesh);
					groupData.newLine();
				}
				catch(IOException e){
					System.out.println("Error writing data to summary file.");
					System.out.println(e.toString());
				}
			}
		}
		
		else if (assayType == CMAssay.MICROFLUIDIC){
			long expTime = currentTime/1000 + channel.getTimeToReach();
			try{
				concentrationData.write(nowString + "\t" + summaryFormat.format(expTime) + "\t");
				channel.writeConcentrationData(concentrationData, currentTime/1000, false);
				concentrationData.newLine();
				concentrationData.flush();
			}
			catch(IOException e){
				System.out.println("Error writing data to concentration file.");
				System.out.println(e.toString());
			}
		}
	}
	
	public void outputCellData(){
		//System.out.println("Output Cell Data");
		long nowTimeMS = (currentTime)/1000;
		//System.out.println("Cell data nowTime " + nowTimeMS);
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
				cellData.write(nowString + "\t" + bioObj.getType() + "\t" + bioObj.getID() + "\t" + com.x + "\t" + com.y + "\t" + com.z + "\t" + vel.x + "\t" + vel.y + "\t" + vel.z);
				cellData.newLine();
				cellData.flush();
			}
			catch(IOException e){
				System.out.println("Cannot write to position file");
				System.out.println(e.toString());
			}
		}
	}
	
	public void outputMembraneData(){
		//System.out.println("Output Membrane Data");
		int numPro = proteins.size();
		if (numPro <= 0){
			return;
		}
		long nowTimeMS = (currentTime)/1000;
		//System.out.println("Membrane nowTime " + nowTimeMS);
		Date nowTime = new Date(nowTimeMS);
		String nowString = summaryFormat.format(nowTime);
		for (int i = 0 ;i < numPro; i++){
			int numObjects = modelObjects.size();
			for (int j = 0; j < numObjects; j++){
				CMBioObj obj = modelObjects.getQuick(j);
				if (obj.getType() == "Segmented Cell"){
					CMSegmentedCell cell = (CMSegmentedCell)obj;
					int numSeg = cell.getNumSegments();
					int cellID = cell.getID();
					for (int k = 0; k < numSeg; k++){
						try{
							//membraneData.write("Time Since Sim Start\tProtein\tCellID\tSegmentID\tBoundDensity\tUnboundDensity");
							membraneData.write(nowString + "\t" + proteins.getQuick(i).getName() + "\t" + cellID + "\t" + k + "\t" + cell.getDensity(k, i, false) + "\t" + cell.getDensity(k, i, true) + "\t" + cell.getTriangleLigandConc(k));
							membraneData.newLine();
							membraneData.flush();
						}
						catch(IOException e){
							System.out.println("Cannot write to membrane data file");
							System.out.println(e.toString());
						}
					}
				}
			}
			
		}
	}
	
	public long getCurrentTimeMicroseconds(){
		return currentTime;
	}
	
	public long getDeltaTimeMilliseconds(){
		//delta time in Milliseconds
		return deltaTime/1000;
	}
	
	public void setBaseCameraDistance(float d){
		baseCameraDistance = d;
	}
	
	public boolean timeToTakeScreenshot(){
		if (generator.generateImages){
			if (currentTime - lastWriteTime > imageDelay){
				lastWriteTime = currentTime;
				return true;
			}
			else{
				return false;
			}
		}
		else{
			return false;
		}
	}
	
	public boolean timeToOutputImage(){
		if (generator.generateImages){
			return (lastImageTime == startTime || (currentTime - lastImageTime > timeBetweenFrames));
		}
		return false;
	}
	
	public ByteBuffer getImageBuffer(int w, int h){
		return imageGenerator.getBuffer(w, h);
	}
	
	public void outputImage(){
		imageGenerator.makeImage();
		lastImageTime = currentTime;
	}
	
	public boolean readyToQuit(){
		return finished;
	}
	
	public void wrapUp(){
		try{
			groupData.flush();
			groupData.close();
			cellData.flush();
			cellData.close();
			concentrationData.flush();
			concentrationData.close();
			membraneData.flush();
			membraneData.close();
		}
		catch(IOException e){
			System.out.println("Unable to close output files");
			System.out.println(e.toString());
		}
		if (concentrationSolver != null){
			concentrationSolver.destroy();
		}
	}
	/*
	public static void main(String[] args) throws LWJGLException {
		CMSimulation sim = new CMSimulation(LWJGL.getGL());
		sim.initPhysics();
		sim.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));

		CMLWJGL.main(args, sim.width, sim.height, "Cell Simulation", sim);
	}*/
	
}