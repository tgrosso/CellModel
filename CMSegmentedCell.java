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

import javax.vecmath.Matrix3f;
import javax.vecmath.Vector3f;

import com.bulletphysics.BulletGlobals;

import com.bulletphysics.collision.narrowphase.ManifoldPoint;
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

import com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor;

import cellModel.shapes.CMGImpactMeshSphere;

import static com.bulletphysics.demos.opengl.IGL.*;

/**
 * @author tagsit
 *
 */
public class CMSegmentedCell extends CMCell{
	
	private static int cell_ids = 0;
	
	
	private CMRigidBody body;
	private CMGImpactMeshSphere cellShape;


	private CMSimulation sim;
	
	private boolean isVisible = true;
	private boolean toRemove = false;
	private int myId;
	private int numSegments;
	
	private float density = 1.10f;
	private float mass;
	private float radius;
	private float volume;
	private float[] freeColor = {1.0f, .8f, .8f};
	private float[] boundColor = {.8f, .8f, 1.0f};
	private float[][] freeProteins;
	private float[][] boundProteins;
	private float[] triangleAreas;
	private float cellSurfaceArea;
	private boolean viewFreeReceptors = true;
	private float[][] color;
	private float maxVel = 25f;
	private int currentVisualizingProtein = -1;
	private long lastTimeMilliseconds, currentTimeMilliseconds;
	
	private Transform trans;
	private Vector3f origin;
	
	private static Vector3f aabbMax = new Vector3f(1e30f, 1e30f, 1e30f);
	private static Vector3f aabbMin = new Vector3f(-1e30f, -1e30f, -1e30f);
	
	private static float[] glMat = new float[16];
	
	
	
	public CMSegmentedCell(CMSimulation s, float r, Vector3f o, int dl, boolean setPros){
		super(s, o);
		this.origin = new Vector3f(o);
		this.sim = s;
		myId = cell_ids;
		cell_ids++;
		this.radius = r;
		volume = (float)(4.0 / 3.0 * Math.PI * radius * radius * radius);
		mass = density * volume;
		//System.out.println("radius: " + radius + " volume: " + volume + " mass: " + mass);
		int detail_level = dl;
		if (detail_level < 0){
			detail_level = 0;
		}
		if (detail_level > 3){
			detail_level = 3;
		}
		cellShape = new CMGImpactMeshSphere(detail_level);
		cellShape.setLocalScaling(new Vector3f(radius, radius, radius));
		cellShape.updateBound();
		Vector3f localInertia = new Vector3f(0, 0, 0);
		cellShape.calculateLocalInertia(mass, localInertia);
		//System.out.println("shape " + cellShape);
		
		trans = new Transform();
		trans.setIdentity();
		trans.origin.set(this.origin);
		
		//System.out.println("local inertia " + localInertia + " radius: " + radius);
		DefaultMotionState motionState = new DefaultMotionState(trans);
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, cellShape, localInertia);
		body = new CMRigidBody(rbInfo, this);
		
		numSegments = cellShape.getNumTriangles();
		
		sim.setNeedsGImpact(true);
		
		cellSurfaceArea = (float)(4.0 * Math.PI * radius * radius); //an initial approximation
		System.out.println("Total surface area: " + cellSurfaceArea);
		triangleAreas = new float[numSegments];
		color = new float[numSegments][];
		for (int i = 0; i < numSegments; i++){
			color[i] = new float[3];
			for (int j = 0; j < 3; j++){
				color[i][j] = (float)Math.random();
			}
			triangleAreas[i] = cellSurfaceArea/numSegments;
			System.out.println("Triangle " + i + ": Area " + triangleAreas[i]);
		}
		objectType = "Segmented Cell";
		//System.out.println("Segmented Cell Created");
		if (setPros){
			setProteins();
		}
		lastTimeMilliseconds = 0L;
		currentTimeMilliseconds = sim.getCurrentTimeMicroseconds()/1000;
	}
	
	public static CMBioObjGroup fillSpace(CMSimulation sim, int numCell, float r, int dl, Vector3f minP, Vector3f maxP, String name, boolean setPro){
		//Will evenly spread the cells throughout the space
		//If the space is not big enough for the cells, it will evenly spread out the maximum
		//number of cells
		
		float interCell = (float)(r * .01); //distance between cells is 1% of the radius
		float width = maxP.x - minP.x - (2 * interCell); //Make sure there is space on the edges
		float height = maxP.y - minP.y - (2 * interCell);
		float depth = maxP.z - minP.z - (2 * interCell);
		
		
		//Divide the space up into Rows, Columns and Pages
		//RCP >= numCells and C/R approx w/h and P/R approx d/h
		//So R^3 >= numCells * h^2 / w * d
		int numRows = (int)(Math.ceil(Math.pow(numCell * height * height / (width * depth), 1.0/3.0)));
		System.out.println("Num rows: " + numRows);
		float rowHeight = height/numRows;
		System.out.println("Row Height: " + rowHeight);
		int numCols = (int)(Math.ceil(numRows * width / height));
		System.out.println("Num cols: " + numCols);
		float colWidth = width/numCols;
		System.out.println("colWidth: " + colWidth);
		int numPages = (int)(Math.ceil(numCell / ((float)numRows * numCols)));
		System.out.println("numPages: " + numPages);
		float pageDepth = depth/numPages;
		System.out.println("pageDepth: " + pageDepth);
		
		//TODO:  If rowHeight or colWidth or pageDepth are too small to fit the cell
		//Adjust the number of rows or columns until
		CMBioObjGroup theCells = new CMBioObjGroup(sim, name);
		int numSquares = numRows * numCols;
		for (int i = 0; i < numCell; i++){
			int col = i % numCols;
			int row = i / numCols % numRows;
			int page = i / numSquares;
			
			float x = minP.x + (interCell/2) + (col * colWidth) + (colWidth/2);
			float y = minP.y + (interCell/2) + (row * rowHeight) + (rowHeight/2);
			float z = minP.z + (interCell/2) + (page * pageDepth) + (pageDepth/2);
			//System.out.println("New Cell: " + x + ", " + y + ", " + z);
			CMSegmentedCell newCell = new CMSegmentedCell(sim, r, new Vector3f(x,y,z), dl, setPro);
			theCells.addObject(newCell);
			newCell.setCellGravity();
		}
		
		return theCells;
	}
	
	public float getRadius(){
		return this.radius;
	}
	
	
	public void setCellGravity(){
		//set the gravity for the cell - it is modified by boyancy
		//a = g(rho_water * volume - mass)/mass + rho_water * Volume
		//This assumes rho_water = 1
		float acceleration = (float)(9.8) * (volume - mass)/(mass + volume);
		//System.out.println("acceleration " + acceleration);
		body.setGravity(new Vector3f(0,acceleration,0));
	}
	
	public CollisionShape getCollisionShape(){
		return cellShape;
	}
	
	public CMRigidBody getRigidBody(){
		return body;
	}
	
	public void setProteins(){
		int numProteins = sim.getNumProteins();
		if (numProteins > 0){
			int numTriangles = cellShape.getNumTriangles();
			freeProteins = new float[numTriangles][numProteins];
			boundProteins = new float[numTriangles][numProteins];
			for (int i = 0; i < numTriangles; i++){
				freeProteins[i] = new float[numProteins];
				boundProteins[i] = new float[numProteins];
				for (int j = 0; j < numProteins; j++){
					float totalProteins = sim.getProtein(j).getBaseDensity() * cellSurfaceArea;
					freeProteins[i][j] = totalProteins*(triangleAreas[i]/cellSurfaceArea);
					boundProteins[i][j] = 0f;
					//System.out.println("Total Protein: " + totalProteins + " Seg: " + i + " Pro: " + j + " Free: " + freeProteins[i][j] + " Bound: " + boundProteins[i][j]);
				}
			}
		}
		currentVisualizingProtein = sim.getViewingProtein();
	}
	
	
	public void collided(CMBioObj c, ManifoldPoint pt, long collId){
		if (c instanceof CMWall){
			if (sim.constraintExists(collId)){
				sim.getConstraint(collId).checkIn();
			}
			else{
				//Find out if the wall has laminin on it
				CMWall wall = (CMWall)c;
				if (!wall.isLamininCoated()){
					return;
				}
				
				//Get the position and orientation of the constraint
				Vector3f positionA = new Vector3f();
				Vector3f positionB = new Vector3f();
				pt.getPositionWorldOnA(positionA);
				pt.getPositionWorldOnB(positionB);
				Vector3f position = new Vector3f();
				position.add(positionA, positionB);
				position.scale(0.5f);
				
				float[] mat = {1f, 0f, 0f, 
								0f, 1f, 0f,
								0f, 0f, 1f};
				Matrix3f world = new Matrix3f(mat);
				Transform worldTrans = new Transform();
				worldTrans.basis.set(world);
				worldTrans.origin.set(position);
				
				//Get the world transforms from the two bodies
				Transform myTrans = new Transform();
				Transform otherTrans = new Transform();
				
				body.getMotionState().getWorldTransform(myTrans);
				c.getRigidBody().getMotionState().getWorldTransform(otherTrans);
				
				//Get the inverses of the transforms
				myTrans.inverse();
				otherTrans.inverse();
				
				//Multiply the inverses by the world transform
				myTrans.mul(worldTrans);
				otherTrans.mul(worldTrans);
				
				//Determine the strength of the constraint
				//Find out if the protein binds to laminin
				int numProteins = sim.getNumProteins();
				int boundProteins = 0;
				int maxProteins = 0;
				int triangle = pt.index0;
				if (triangle<0){
					triangle = pt.index1;
				}
				if (triangle < 0){
					//no triangle found
					return;
				}
				for (int i = 0; i < numProteins; i++){
					CMMembraneProtein pro = sim.getProtein(i);
					/*
					if (pro.bindsToLaminin()){
						int ligandProteins = Math.round(wall.getLamininDensity() * triangleAreas[triangle]);
						int freeReceptors = Math.round(freeProteins[triangle][i] * triangleAreas[triangle]);
						int boundReceptors = Math.round(boundProteins[triangle][i] * triangleAreas[triangle]);
						int newBoundProteins = pro.bindReceptors(ligandProteins, freeReceptors);
						if (newBoundProteins < 10){
							newBoundProteins = 0; //minimum number of proteins required for bond
						}
						maxProteins += (freeReceptors + boundReceptors);
						freeReceptors = freeReceptors - newBoundProteins;
						if (freeReceptors < 0){
							freeReceptors = 0;
						}
						freeDensities[triangle][i] = freeReceptors / triangleAreas[triangle];
						boundReceptors = boundReceptors + newBoundProteins;
						boundDensities[triangle][i] = boundReceptors / triangleAreas[triangle];
						
						boundProteins += newBoundProteins;
						System.out.println("Free Receptors: " + freeReceptors + " Bound: " + boundReceptors);
					} */
				}

				//System.out.println("Bound: " + boundProteins + " Max: " + maxProteins);
				CMGenericConstraint con = new CMGenericConstraint(sim, body, c.getRigidBody(), myTrans, otherTrans, true, 5000, 50, collId, boundProteins, maxProteins, triangle);
				con.checkIn();
				sim.addConstraint(con);
				 
			}
		}
	}
	
	public void breakBonds(int brokenBonds, int seg){
		//return some of the broken bonds to be free
		if (seg >= 0){
			int numProteins = sim.getNumProteins();
			for (int i = 0; i<numProteins; i++){
				CMMembraneProtein pro = sim.getProtein(i);
				if (pro.bindsToLaminin()){
					/*
					float maxDensity = pro.getMaxDensity();
					int freeReceptors = Math.round(freeProteins[seg][i] * triangleAreas[seg]);
					int boundReceptors = Math.round(boundProteins[seg][i] * triangleAreas[seg]);
					boundReceptors -= brokenBonds * .9f;
					freeReceptors += brokenBonds * .9f;
					freeProteins[seg][i] = freeReceptors/triangleAreas[seg];
					boundProteins[seg][i] = boundReceptors/triangleAreas[seg];
					*/
				}
			}
		}
	}
	
	public void updateObject(){
		if (!body.isActive()){
			//System.out.println("Cell " + this.id + " has been deactivated.");
			body.activate();
		}
		//randomly rotate
		int axis = (int)(sim.nextRandomF() * 3);
		float vel = sim.nextRandomF() *2 - 1;
		float[] velVec = new float[3];
		for (int i = 0; i < 3; i++){
			velVec[i] = 0f;
		}
		velVec[axis] = vel;
		body.setAngularVelocity(new Vector3f(velVec));
		
		//update the proteins
		updateProteinsCallback proCallback = new updateProteinsCallback(this);
		cellShape.processAllTriangles(proCallback, aabbMin, aabbMax);
		lastTimeMilliseconds = currentTimeMilliseconds;
		currentTimeMilliseconds = sim.getCurrentTimeMicroseconds()/1000;
		
		//set the segment colors
		float percent = 0f;
		CMMembraneProtein pro = sim.getProtein(currentVisualizingProtein);
		float maxDensity = pro.getMaxDensity();
		float[] c = new float[3];
		for (int i = 0; i < numSegments; i++){
			if (viewFreeReceptors){
				percent = freeProteins[i][currentVisualizingProtein]/maxDensity;
				c = freeColor;
			}
			else{
				percent = boundProteins[i][currentVisualizingProtein]/maxDensity;
				c = boundColor;
			}
			setSegmentColor(i, c[0] * percent, c[1] * percent, c[2] * percent);
		}
		
	}
	
	public boolean specialRender(IGL gl, Transform t){
		gl.glPushMatrix();
		t.getOpenGLMatrix(glMat);
		gl.glMultMatrix(glMat);
		
		drawSegmentsCallback drawCallback = new drawSegmentsCallback(gl, this);
		cellShape.processAllTriangles(drawCallback, aabbMin, aabbMax);
		gl.glPopMatrix();
		
		return true;
	}
	
	public float[] getSegmentColor(int segment){
		return color[segment];
	}
	
	public void setSegmentColor(int segment, float r, float g, float b){
		color[segment][0] = r;
		color[segment][1] = g;
		color[segment][2] = b;
	}
	
	public Vector3f getColor3Vector(){
		return new Vector3f();
	}
	
	public String getType(){
		return objectType;
	}
	
	public int getNumSegments(){
		return numSegments;
	}
	
	public float getDensity(int segment, int protein, boolean freeDen){
		if (freeDen && freeProteins != null){
			return freeProteins[segment][protein];
		}
		if (!freeDen && boundProteins != null){
			return boundProteins[segment][protein];
		}
		return 0f;
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
			gl.glBegin(GL_LINES);
			gl.glColor3f(0f, 0f, 0f);
			gl.glVertex3f(triangle[0].x, triangle[0].y, triangle[0].z);
			gl.glVertex3f(triangle[1].x, triangle[1].y, triangle[1].z);
			gl.glVertex3f(triangle[2].x, triangle[2].y, triangle[2].z);
			gl.glVertex3f(triangle[0].x, triangle[0].y, triangle[0].z);
			gl.glEnd();
		}
	}
	
	private static class updateProteinsCallback extends TriangleCallback {
		CMSegmentedCell parent;

		public updateProteinsCallback(CMSegmentedCell p) {
			this.parent = p;
		}
		
		public void processTriangle(Vector3f[] triangle, int partId, int triangleIndex) {
			//find the area
			float ab_squ = (triangle[1].x - triangle[0].x) * (triangle[1].x - triangle[0].x) +
					(triangle[1].y - triangle[0].y) * (triangle[1].y - triangle[0].y) +
					(triangle[1].z - triangle[0].z) * (triangle[1].z - triangle[0].z);
			float bc_squ = (triangle[2].x - triangle[1].x) * (triangle[2].x - triangle[1].x) +
					(triangle[2].y - triangle[1].y) * (triangle[2].y - triangle[1].y) +
					(triangle[2].z - triangle[1].z) * (triangle[2].z - triangle[1].z);
			float ca_squ = (triangle[2].x - triangle[0].x) * (triangle[2].x - triangle[0].x) +
					(triangle[2].y - triangle[0].y) * (triangle[2].y - triangle[0].y) +
					(triangle[2].z - triangle[0].z) * (triangle[2].z - triangle[0].z);
				
			float area = (float)(.25 * Math.sqrt(4 * ab_squ * bc_squ - (Math.pow(ab_squ + bc_squ - ca_squ, 2))));
			parent.triangleAreas[triangleIndex] = area;

			//find the ligand concentration
			long time = parent.currentTimeMilliseconds;
			//System.out.print("Current time " + time + " area: " + area);
			float x_dist = (float)((triangle[0].x + triangle[1].x + triangle[2].x)/3.0);
			//System.out.print(" x dist: " + x_dist);
			float ligand = parent.sim.getLigandConcentration(x_dist, time);
			//System.out.print(" ligand: " + ligand);
			//need delta time in minutes
			float deltaTime = (time - parent.lastTimeMilliseconds)/1000f/60f; //convert to minutes
			//System.out.println(" deltaTime: " + deltaTime);
			if (deltaTime > 0){
				for (int i = 0; i < parent.sim.getNumProteins(); i++){
					//update the free receptors
					//parent.freeProteins[triangleIndex][i] = parent.sim.getProtein(i).updateFreeProteins(area, ligand, parent.boundProteins[triangleIndex][i], parent.freeProteins[triangleIndex][i], deltaTime);
					parent.freeProteins[triangleIndex][i] = parent.sim.getProtein(i).updateFreeReceptors(ligand, parent.boundProteins[triangleIndex][i], parent.freeProteins[triangleIndex][i], area/parent.cellSurfaceArea, deltaTime);
					//update the bound receptors
					parent.boundProteins[triangleIndex][i] = parent.sim.getProtein(i).updateBoundReceptors(ligand, parent.boundProteins[triangleIndex][i], parent.freeProteins[triangleIndex][i], deltaTime);
					//System.out.println("Segment: " + triangleIndex + " free: " + parent.freeDensities[triangleIndex][i] + " bound: " + parent.boundDensities[triangleIndex][i]);
				}
			}
			
		}
	}

}
