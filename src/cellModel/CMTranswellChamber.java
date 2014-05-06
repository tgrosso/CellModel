/**
 * Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 * Package: cellModel
 * File: CMTranswellChamber.java
 * Jul 27, 2013 4:05:23 PM
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

/**
 * @author tagsit
 * Although the diameter of the pores is given, for now the pores are square
 * In addition, the number of pores in the mesh will be rows * cols - even if that's a few too many.
 */
public class CMTranswellChamber {
	

	private float wallThick = 2f;
	private float meshThick; 
	private float poreDensity;
	private float poreDiameter;
	private float wellDepth;
	private float insetDepth;
	private float testTubeX;
	private float testTubeZ;
	private float[] meshColor = {.7f, .7f, .9f};
	private float[] wallColor = {.4f, .2f, .2f};
	private Vector3f testTubeSize;
	private CMSimulation sim;
	
	public CMTranswellChamber(CMSimulation s){
		sim = s;
		setInsetDepth(60f);
		setMeshThickness(10f);
		setPoreDensity(100000f);
		setPoreDiameter(10.2f);
		setTestTubeX(80f);
		setTestTubeZ(80f);
		setWellDepth(60f);
		testTubeSize = new Vector3f(testTubeX, wellDepth + insetDepth + meshThick, testTubeZ);
	}
	
	public void setMeshColor(float r, float g, float b){
		meshColor[0] = r;
		meshColor[1] = g;
		meshColor[2] = b;
	}
	
	public void setWallColor(float r, float g, float b){
		wallColor[0] = r;
		wallColor[1] = g;
		wallColor[2] = b;
	}
	
	public void setInsetDepth(float depth){
		insetDepth = depth;
	}
	
	public void setMeshThickness(float thick){
		meshThick = thick;
	}
	
	public void setPoreDensity(float den){
		poreDensity = den;
	}
	
	public void setPoreDiameter(float diam){
		poreDiameter = diam;
	}
	
	public void setTestTubeX(float x){
		testTubeX = x;
	}
	
	public void setTestTubeZ(float z){
		testTubeZ = z;
	}
	
	public void setWellDepth(float depth){
		wellDepth = depth;
	}
	
	public Vector3f getMaxBelowMeshVector(){
		return new Vector3f((float)(testTubeX/2.0), -(float)(meshThick/2.0), (float)(testTubeZ/2.0));
	}
	
	public Vector3f getMinBelowMeshVector(){
		return new Vector3f(-(float)(testTubeX/2.0), -(float)(wellDepth + meshThick/2.0), -(float)(testTubeZ/2.0));
	}
	
	public Vector3f getMaxAboveMeshVector(){
		return new Vector3f((float)(testTubeX/2.0), (float)(insetDepth + meshThick/2.0), (float)(testTubeZ/2.0));
	}
	
	public Vector3f getMinAboveMeshVector(){
		return new Vector3f(-(float)(testTubeX/2.0), (float)(meshThick/2.0), -(float)(testTubeZ/2.0));
	}
	
	public void makeChamber(){
		/** 
		 * Adds the chamber and mesh walls to the simulation
		 */
		CMWall nextWall;
		
		//botom
		nextWall = new CMWall(sim, testTubeSize.x, wallThick, testTubeSize.z, new Vector3f(0f, -(testTubeSize.y+wallThick)/2, 0f));
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		sim.addBioObject(nextWall);
		
		//top
		nextWall = new CMWall(sim, testTubeSize.x, wallThick, testTubeSize.z, new Vector3f(0f, (testTubeSize.y+wallThick)/2, 0f));
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		sim.addBioObject(nextWall);
		
		//left
		nextWall = new CMWall(sim, wallThick, testTubeSize.y, testTubeSize.z, new Vector3f(-(testTubeSize.x+wallThick)/2, 0f, 0f));
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		//nextWall.setVisible(false);
		sim.addBioObject(nextWall);
		
		//right
		nextWall = new CMWall(sim, wallThick, testTubeSize.y, testTubeSize.z, new Vector3f((testTubeSize.x+wallThick)/2, 0f, 0f));
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		//nextWall.setVisible(false);
		sim.addBioObject(nextWall);
		
		//back
		nextWall = new CMWall(sim, testTubeSize.x, testTubeSize.y, wallThick, new Vector3f(0f, 0f, (testTubeSize.z + wallThick)/2));
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		//nextWall.setVisible(false);
		sim.addBioObject(nextWall);
		
		//front
		nextWall = new CMWall(sim, testTubeSize.x, testTubeSize.y, wallThick, new Vector3f(0f, 0f, -(testTubeSize.z+wallThick)/2));
		nextWall.setVisible(false);
		sim.addBioObject(nextWall);

		addMesh(testTubeSize.x, testTubeSize.z, meshThick, poreDensity, poreDiameter, wellDepth);
		sim.setCameraDistance(100f);
	}
	
	private void addMesh(float width, float depth, float thickness, float p_density, float p_diameter, float height){
		//p_density is pore density - pores/cm^2
		//p_diameter is pore diameter in micrometers
		
		
		//find number of pores 
		double numPores = Math.ceil(p_density * width * depth * Math.pow(10, -8));
		
		//How many rows and columns will we need? R*C>=numPores and C/R approx w/l
		//So R^2 >= Nl/w
		int numRows = (int)Math.ceil(Math.sqrt(numPores * depth / width));
		int numCols = (int)Math.ceil((float)numPores/numRows);
		float colWidth = width/numCols;
		float rowHeight = depth/numRows;
		float z_space = (rowHeight - p_diameter)/2;
		float x_space = (colWidth - p_diameter)/2;
		//Add row spacers:
		for (int i = 0; i < numRows; i++){
			Vector3f wall_origin = new Vector3f(0f, (-testTubeSize.y/2 + height + thickness/2), (depth / 2 - i * (z_space*2+p_diameter) - z_space/2));
			CMWall wall = new CMWall(sim, width, thickness, z_space, wall_origin);
			wall.setColor(meshColor[0], meshColor[1], meshColor[2]);
			sim.addBioObject(wall);
			wall_origin = new Vector3f(0f, (-testTubeSize.y/2 + height + thickness/2), (depth / 2 - i * (z_space*2+p_diameter) -(z_space+p_diameter)- z_space/2 ));
			wall = new CMWall(sim, width, thickness, z_space, wall_origin);
			wall.setColor(meshColor[0], meshColor[1], meshColor[2]);
			sim.addBioObject(wall);
		}
		
		for (int i = 0; i < numRows; i++){
			float y_value = -testTubeSize.y/2 + height + thickness/2;
			float z_value = depth/2 - z_space - p_diameter/2 - i * (2 * z_space + p_diameter);
			for (int j = 0; j < numCols; j++){
				float x_value = width/2 - (j * (2 * x_space + p_diameter)) - x_space/2;
				CMWall wall = new CMWall(sim, x_space, thickness, p_diameter, new Vector3f(x_value, y_value, z_value));
				wall.setColor(meshColor[0], meshColor[1], meshColor[2]);
				sim.addBioObject(wall);
				x_value = width/2 - (j * (2 * x_space + p_diameter)) - x_space/2-p_diameter-x_space;
				wall = new CMWall(sim, x_space, thickness, p_diameter, new Vector3f(x_value, y_value, z_value));
				wall.setColor(meshColor[0], meshColor[1], meshColor[2]);
				sim.addBioObject(wall);
			}
		}
	}
	
}
