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


import javax.vecmath.Vector3f;

import cellModel.shapes.CMGImpactMeshSphere;

import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.TriangleCallback;

import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor;

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
	
	private float density = 1.50f;
	private float mass;
	private float radius;
	private float volume;
	private float[] freeColor = {1.0f, .8f, .8f};
	private float[] boundColor = {.8f, .8f, 1.0f};
	private float[][] freeProteins;
	private float[][] boundProteins;
	private float[][] boundEndocytosisRates;
	private float[][] unboundEndocytosisRates;
	private float[][] exocytosisRates;
	private float[] triangleAreas;
	private float[] ligandConc;
	private float cellSurfaceArea;
	private boolean viewFreeReceptors = true;
	private float[][] color;
	private float maxVel = 25f;
	private int currentVisualizingProtein = -1;
	private int molsPerConstraint = 100;
	
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
		
		//System.out.println("Total surface area: " + cellSurfaceArea);
		triangleAreas = new float[numSegments];
		
		triangleAreaCallback areaCallback = new triangleAreaCallback(this);
		cellShape.processAllTriangles(areaCallback, aabbMin, aabbMax);
		cellSurfaceArea = 0;
		for (int i = 0; i < numSegments; i++){
			cellSurfaceArea += triangleAreas[i];
			//System.out.println("triangle " + i + " surface area " + triangleAreas[i]);
		}
		//System.out.println("surface area: " + cellSurfaceArea);
		
		ligandConc = new float[numSegments];
		color = new float[numSegments][];
		for (int i = 0; i < numSegments; i++){
			color[i] = new float[3];
			for (int j = 0; j < 3; j++){
				color[i][j] = (float)Math.random();
			}
			ligandConc[i] = 0f;
		}
		objectType = "Segmented Cell";
		//System.out.println("Segmented Cell Created");
		if (setPros){
			setProteins();
		}
		
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
		//System.out.println("Num rows: " + numRows);
		float rowHeight = height/numRows;
		//System.out.println("Row Height: " + rowHeight);
		int numCols = (int)(Math.ceil(numRows * width / height));
		//System.out.println("Num cols: " + numCols);
		float colWidth = width/numCols;
		//System.out.println("colWidth: " + colWidth);
		int numPages = (int)(Math.ceil(numCell / ((float)numRows * numCols)));
		//System.out.println("numPages: " + numPages);
		float pageDepth = depth/numPages;
		//System.out.println("pageDepth: " + pageDepth);
		
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
	
	public static CMBioObjGroup randomFillSurface(CMSimulation sim, int numCell, float r, int dl, Vector3f minP, Vector3f maxP, String name, boolean setPro){
		//Places cells just above a surface with numCells
		//System.out.println("minP " + minP + " maxP " + maxP);
		int padding = 6;
		int x_cells = (int)((maxP.x - minP.x) / (2 * r + padding));
		int z_cells = (int)((maxP.z - minP.z) / (2 * r + padding));
		float col_width = (maxP.x - minP.x) / x_cells;
		float row_width = (maxP.z - minP.z) / z_cells;
		float center_y = minP.y + r + (padding/2.0f);
		//System.out.println("x cells " + x_cells + " z cells " + z_cells);
		//System.out.println("colWidth " + col_width + " row_width " + row_width);
		int max_cells = x_cells * z_cells;
		int num = numCell;
		if (num > max_cells){
			num = max_cells;
		}
		
		//get a random order for placing the cells
		int[] indices = new int[max_cells];
		for (int i = 0; i < max_cells; i++){
			indices[i] = i;
		}
		for (int i = 0; i < max_cells; i++){
			int swap_index = (int)(sim.nextRandomF() * max_cells);
			int temp = indices[i];
			indices[i] = indices[swap_index];
			indices[swap_index] = temp;
		}
		
		
		CMBioObjGroup theCells = new CMBioObjGroup(sim, name);
		
		for (int i = 0; i < num; i++){
			int row = indices[i]/x_cells;
			int col = indices[i]%x_cells;
			float center_x = minP.x + (col * col_width) + padding/2.0f + r;
			float center_z = minP.z + (row * row_width) + padding/2.0f + r;
			CMSegmentedCell newCell = new CMSegmentedCell(sim, r, new Vector3f(center_x,center_y,center_z), dl, setPro);
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
			boundEndocytosisRates = new float[numTriangles][numProteins];
			unboundEndocytosisRates = new float[numTriangles][numProteins];
			exocytosisRates = new float[numTriangles][numProteins];
			for (int i = 0; i < numTriangles; i++){
				freeProteins[i] = new float[numProteins];
				boundProteins[i] = new float[numProteins];
				boundEndocytosisRates[i] = new float[numProteins];
				unboundEndocytosisRates[i] = new float[numProteins];
				exocytosisRates[i] = new float[numProteins];
				for (int j = 0; j < numProteins; j++){
					CMMembraneProtein pro = sim.getProtein(j);
					float totalProteins = pro.getInitialProteins(1.0f);
					freeProteins[i][j] = pro.getInitialProteins(triangleAreas[i]/cellSurfaceArea);
					boundProteins[i][j] = 0;
					//System.out.println("Total Protein: " + totalProteins + " Seg: " + i + " Pro: " + j + " Free: " + freeProteins[i][j] + " Bound: " + boundProteins[i][j]);
					boundEndocytosisRates[i][j] = pro.getBoundEndocytosisRate();
					unboundEndocytosisRates[i][j] = pro.getUnboundEndocytosisRate();
					exocytosisRates[i][j] = pro.getExocytosisRate() * triangleAreas[i]/cellSurfaceArea;
				}
			}
		}
		currentVisualizingProtein = sim.getViewingProtein();
	}
	
	
	public void collided(CMBioObj c, ManifoldPoint pt, long collId){
		if (sim.constraintExists(collId)){
			sim.checkInConstraints(collId); //check in to all constraints from this collision
			return;
		}
		if (c instanceof CMWall){
			//Find out if the wall has laminin on it
			//Really we should be seeing if the other object has any proteins that bind!!
			CMWall wall = (CMWall)c;
			if (!wall.isLamininCoated()){
				return;
			}
			
			Transform myTrans = new Transform();
			body.getMotionState().getWorldTransform(myTrans);
			Transform otherTrans = new Transform();
			c.getRigidBody().getMotionState().getWorldTransform(otherTrans);
				
			//Find out the triangle which is colliding
			int triangleIndex = pt.index0;
			if (triangleIndex<0){
				triangleIndex = pt.index1;
			}
			if (triangleIndex < 0){
				//no triangle found
				return;
			}
				
			//sim.writeToLog(sim.getFormattedTime() + " Cell " + myId + " Collision. Triangle " + triangleIndex + " Collision Id " + collId);
			
			//Get the surface Laminin density
			float lamininDensity = wall.getLamininDensity();
				
			//Find the vertices of the triangle
			Vector3f[] vertices = new Vector3f[3];
			triangleVerticesCallback tcb = new triangleVerticesCallback(triangleIndex);
			cellShape.processAllTriangles(tcb, aabbMin, aabbMax);
			vertices = tcb.getVertices();
			//TODO - should we just update the Transform whenever we update?
			//sim.writeToLog("   Finding Triangle Vertices: ");
			for (int i = 0; i < vertices.length; i++){
				//sim.writeToLog("      Pre-transform vertex " + i + vertices[i]);
				myTrans.transform(vertices[i]);
				//sim.writeToLog("      Post-transform vertex " + i + vertices[i]);
			}
			
			//For simplicities sake, we project the membrane segment's triangle onto the laminin-coated wall
			//We are currently assuming that the wall is below the triangle
			//TODO Figure out where the wall is in relation to the triangle
			Vector3f min = new Vector3f();
			Vector3f max = new Vector3f();
			wall.getRigidBody().getAabb(min, max);
			float wallY = max.y;
			
				
			//Find the area of the triangle projected onto the wall
			//Find the number of laminin molecules on this area of the wall
			//We are going to project onto xz plane - this should be generalized!
			Vector3f[] wallVert = new Vector3f[3];
			for (int i = 0; i < 3; i++){
				wallVert[i] = new Vector3f(vertices[i].x, wallY, vertices[i].z);
				//sim.writeToLog("   Wall Vertex[" + i + "] " + wallVert[i]);
			}
			float wallArea = findTriangleArea(wallVert);
			float lamininMolecules = lamininDensity * wallArea;
			//sim.writeToLog("   laminin molecules: " + lamininMolecules);
			
			//check for proteins that bind with laminin
			for (int i = 0; i < sim.getNumProteins(); i++){
				CMMembraneProtein pro = sim.getProtein(i);
				//sim.writeToLog("Protein? " + pro.getName() + " binds? " + pro.bindsToLaminin());
				if (!pro.bindsToLaminin()){
					continue;
				}
				//sim.writeToLog("   Protein binding " + pro.getName());
				//Find the number of unbound integrin molecules on the segment
				//System.out.println("Protein " + pro.getName());
				int unboundProteins = (int)freeProteins[triangleIndex][i];
				//sim.writeToLog("   unbound proteins: " + unboundProteins);
				if (unboundProteins < molsPerConstraint){
					continue;
				}
					
				//Find out the number of constraints formed with this protein
				int newBound = pro.bindReceptors((int)lamininMolecules, unboundProteins);
				newBound = Math.min(newBound, unboundProteins);
				int numConstraints = newBound/molsPerConstraint;
				//sim.writeToLog("   Binding Proteins: " + newBound);
				//sim.writeToLog("   Attempting to make " + numConstraints + " constraints");
					
				//TODO let user determine the number of bonds/constraint
					
				//Distribute the constraints randomly around the triangle
				//Note - to find a random point on the triangle:
				//http://adamswaab.wordpress.com/2009/12/11/random-point-in-a-triangle-barycentric-coordinates/
				for (int j = 0; j < numConstraints; j++){
					//Find a random point on the triangle
					//get two vectors on the triangle
					Vector3f ab = new Vector3f(vertices[1]);
					ab.sub(vertices[0]);
					//System.out.println(vertices[1] + " - " + vertices[0] + " = " + ab);
					Vector3f ac = new Vector3f(vertices[2]);
					ac.sub(vertices[0]);
					//System.out.println(vertices[2] + " - " + vertices[0] + " = " + ac);
					float r = sim.nextRandomF();
					float s = sim.nextRandomF();
					if (r + s >= 1){
						r = 1 - r;
						s = 1 - s;
					}
					ab.scale(r);
					ac.scale(s);
					Vector3f randVec = new Vector3f(vertices[0]);
					randVec.add(ab);
					randVec.add(ac);
					//sim.writeToLog("Random Triangle Point: " + randVec);
						
					//Find the distance between this point and the point on the wall.
					float yDist = Math.abs(randVec.y - wallY);
					if (yDist <= 2.5){
						//make the constraint
						Vector3f wallVec = new Vector3f(randVec.x, wallY, randVec.z);
						//sim.writeToLog("Random Wall Point: " + wallVec);
						//We need the points on the cell and on the wall in the local frames of reference
						
						Transform cellTrans = new Transform(myTrans);
						cellTrans.inverse();
						cellTrans.transform(randVec);
						cellTrans.origin.set(randVec);
						cellTrans.basis.set(myTrans.basis);
						//sim.writeToLog("myTrans origin " + myTrans.origin);
						//sim.writeToLog("myTrans basis \n" + myTrans.basis);
						//sim.writeToLog("cellTrans origin " + cellTrans.origin);
						//sim.writeToLog("cellTrans basis " + cellTrans.basis);
						
						Transform wallTrans = new Transform(otherTrans);
						wallTrans.inverse();
						wallTrans.transform(wallVec);
						wallTrans.origin.set(wallVec);
						wallTrans.basis.set(otherTrans.basis);
						//sim.writeToLog("otherTrans origin " + otherTrans.origin);
						//sim.writeToLog("otherTrans basis \n" + otherTrans.basis);
						//sim.writeToLog("wallTrans origin " + wallTrans.origin);
						//sim.writeToLog("wallTrans basis " + wallTrans.basis);
					
						CMGenericConstraint con = new CMGenericConstraint(sim, c.getRigidBody(), body, wallTrans, cellTrans, true, collId, j, triangleIndex, i);
						con.checkIn();
						
						Generic6DofConstraint constraint = con.getConstraint();
						constraint.setLimit(0, 0f, 0f);
						constraint.setLimit(1, 0f, 1f);
						constraint.setLimit(2, 0f, 0f);
						TranslationalLimitMotor tlm = constraint.getTranslationalLimitMotor();
						tlm.damping = 2.0f;
						tlm.restitution = .1f;
						

						//sim.addConstraint(con);
						freeProteins[triangleIndex][i] -= molsPerConstraint; //remove free proteines
						boundProteins[triangleIndex][i] += molsPerConstraint; //add bound proteins
						//sim.writeToLog("      Made a constraint");
						
					}//end if constraint is short enough
					else{
						//sim.writeToLog("      Constraint too long. Not created.");
					}
				}//end for loop to go through constraints	
			}//end for loop for each protein
		}//end if collision is with a wall
	}//end collided
	
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
		
		//set the segment colors
		float percent = 0f;
		CMMembraneProtein pro = sim.getProtein(currentVisualizingProtein);
		float maxDensity = pro.getMaxDensity();
		float[] c = new float[3];
		for (int i = 0; i < numSegments; i++){
			if (viewFreeReceptors){
				percent = (freeProteins[i][currentVisualizingProtein]/triangleAreas[i])/maxDensity;
				c = freeColor;
			}
			else{
				percent = boundProteins[i][currentVisualizingProtein]/triangleAreas[i]/maxDensity;
				c = boundColor;
			}
			setSegmentColor(i, c[0] * percent, c[1] * percent, c[2] * percent);
		}
		
	}
	
	public void reclaimMembraneProteins(int segment, int pro){
		boundProteins[segment][pro] -= molsPerConstraint;
		freeProteins[segment][pro] += molsPerConstraint;
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
	
	public float getTriangleLigandConc(int index){
		return ligandConc[index];
	}
	
	private float findTriangleArea(Vector3f[] vertices){
		//Find the lengths of the sides
		float a = (float)Math.sqrt(Math.pow((vertices[1].x - vertices[0].x), 2) +
				Math.pow((vertices[1].y - vertices[0].y), 2) + Math.pow((vertices[1].z - vertices[0].z), 2));
		float b = (float)Math.sqrt(Math.pow((vertices[2].x - vertices[1].x), 2) +
				Math.pow((vertices[2].y - vertices[1].y), 2) + Math.pow((vertices[2].z - vertices[1].z), 2));
		float c = (float)Math.sqrt(Math.pow((vertices[2].x - vertices[0].x), 2) +
				Math.pow((vertices[2].y - vertices[0].y), 2) + Math.pow((vertices[2].z - vertices[0].z), 2));
		//Use Heron's forumla
		float s = (float)(.5 * (a + b + c));
		
		float area = (float)(Math.sqrt(s * (s-a) * (s-b) * (s-c)));
		//System.out.println("Finding area for: ");
		//for (int i = 0; i < 3; i++){
		//	System.out.print(vertices[i].toString() + " ");
		//}
		//System.out.println(" Area: " + area);
		return area;
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
			//find the ligand concentration
			//System.out.print("Current time " + time + " area: " + area);
			float x_dist = (float)((triangle[0].x + triangle[1].x + triangle[2].x)/3.0);
			//System.out.print(" x dist: " + x_dist);
			float ligand = parent.sim.getLigandConcentration(x_dist, parent.sim.getCurrentTimeMicroseconds()/1000);
			parent.ligandConc[triangleIndex] = ligand;
			float deltaTime = parent.sim.getDeltaTimeMilliseconds()/60f/1000;
			//System.out.println(" deltaTime: " + deltaTime);
			if (deltaTime > 0){
				for (int i = 0; i < parent.sim.getNumProteins(); i++){
					//update the free receptors
					parent.freeProteins[triangleIndex][i] = parent.sim.getProtein(i).updateFreeReceptors(ligand, parent.boundProteins[triangleIndex][i], parent.freeProteins[triangleIndex][i], parent.unboundEndocytosisRates[triangleIndex][i], parent.exocytosisRates[triangleIndex][i], deltaTime);
					//update the bound receptors
					parent.boundProteins[triangleIndex][i] = parent.sim.getProtein(i).updateBoundReceptors(ligand, parent.boundProteins[triangleIndex][i], parent.freeProteins[triangleIndex][i], parent.boundEndocytosisRates[triangleIndex][i], deltaTime);
					//System.out.println("Segment: " + triangleIndex + " free: " + parent.freeDensities[triangleIndex][i] + " bound: " + parent.boundDensities[triangleIndex][i]);
				}
			}
			
		}
	}
	
	private static class triangleAreaCallback extends TriangleCallback {
		CMSegmentedCell parent;

		public triangleAreaCallback(CMSegmentedCell p) {
			this.parent = p;
		}
		
		public void processTriangle(Vector3f[] triangle, int partId, int triangleIndex) {
			//System.out.println("Processing triangle areas");
			//find the area
				
			float area = parent.findTriangleArea(triangle);
			parent.triangleAreas[triangleIndex] = area;
			//System.out.println("triangle " + triangleIndex + " area " + area);
			
		}
	}
	
	private static class triangleVerticesCallback extends TriangleCallback {
		Vector3f[] vertices;
		int index;

		public triangleVerticesCallback(int i) {
			vertices = new Vector3f[3];
			index = i;
		}
		
		public void processTriangle(Vector3f[] triangle, int partId, int triangleIndex) {
			if (triangleIndex == index){
				for (int i = 0; i < 3; i++){
					vertices[i] = new Vector3f(triangle[i]);
				}
				return;
			}
			
		}
		
		public Vector3f[] getVertices(){
			return vertices;
		}
	}


}