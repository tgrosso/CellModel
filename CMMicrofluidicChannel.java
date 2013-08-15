/**
 * Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 * Package: cellModel
 * File: CMMicrofluidicChannel.java
 * Aug 5, 2013 5:16:31 PM
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
import java.io.BufferedWriter;
import java.io.IOException;

/**
 * @author tagsit
 *
 */
public class CMMicrofluidicChannel {
	public static final int LEFT = 0, RIGHT = 1;
	private float wallThick = 2f;
	private float channelWidth = 1000f, channelHeight = 100f, channelDepth = 90f;
	private float resWidth = 100;
	private int sourceMolecules, sinkMolecules;
	private float[] wallColor = {.4f, .2f, .2f};
	private float[] resColor = {.2f, .2f, .4f};
	private int numSegments = 5;
	private float segmentWidth = 20;
	private int source = LEFT, sink = RIGHT;
	private CMBioObjGroup molecules;
	private CMSimulation sim;
	
	public CMMicrofluidicChannel(CMSimulation s, int srcMols, int snkMols){
		sourceMolecules = srcMols;
		sinkMolecules = snkMols;
		sim = s;
		makeChannel(sim);
	}
	
	public void makeChannel(CMSimulation sim){
		/** 
		 * Adds the channel and reservoirs to the simulation
		 */
		CMWall nextWall;
		Vector3f position = new Vector3f(0f, 0f, 0f);
		
		//Make the channel
		//bottom
		position.set(0f, -(float)((channelHeight+wallThick)/2.0), 0f);
		nextWall = new CMWall(sim, channelWidth, wallThick, channelDepth, position); 
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		sim.addBioObject(nextWall);
		
		//top
		position.set(0f, (float)((channelHeight+wallThick)/2.0), 0f);
		nextWall = new CMWall(sim, channelWidth, wallThick, channelDepth, position); 
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		sim.addBioObject(nextWall);
		
		//back
		position.set(0f, 0f, (float)((channelDepth+wallThick)/2.0));
		nextWall = new CMWall(sim, channelWidth, channelHeight, wallThick, position); 
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		sim.addBioObject(nextWall);
		
		//front
		position.set(0f, 0f, -(float)((channelDepth+wallThick)/2.0));
		nextWall = new CMWall(sim, channelWidth, channelHeight, wallThick, position); 
		nextWall.setVisible(false);
		sim.addBioObject(nextWall);
		
		makeReservoir(sim, LEFT);
		makeReservoir(sim, RIGHT);
		
		sim.setBaseCameraDistance(600f);
	}
	
	private void makeReservoir(CMSimulation sim, int direction){
		CMWall nextWall;
		Vector3f position = new Vector3f(0f, 0f, 0f);
		float absShift = (float)((channelWidth + resWidth)/2.0);
		float xShift = (direction == LEFT) ? (absShift) : (-absShift);
		
		//bottom
		position.set(xShift, -(float)((channelHeight+wallThick)/2.0), 0f);
		nextWall = new CMWall(sim, resWidth, wallThick, channelDepth, position); 
		nextWall.setColor(resColor[0], resColor[1], resColor[2]);
		sim.addBioObject(nextWall);
		
		//top
		position.set(xShift, (float)((channelHeight+wallThick)/2.0), 0f);
		nextWall = new CMWall(sim, resWidth, wallThick, channelDepth, position); 
		nextWall.setColor(resColor[0], resColor[1], resColor[2]);
		sim.addBioObject(nextWall);
		
		//back
		position.set(xShift, 0f, (float)((channelDepth+wallThick)/2.0));
		nextWall = new CMWall(sim, resWidth, channelHeight, wallThick, position); 
		nextWall.setColor(resColor[0], resColor[1], resColor[2]);
		sim.addBioObject(nextWall);
				
		//front
		position.set(xShift, 0f, -(float)((channelDepth+wallThick)/2.0));
		nextWall = new CMWall(sim, resWidth, channelHeight, wallThick, position); 
		nextWall.setVisible(false);
		//nextWall.setColor(resColor[0], resColor[1], resColor[2]);
		
		//side
		float xPos = (direction == LEFT)?(float)(xShift  + (resWidth + wallThick)/2.0):
			(float)(xShift - (resWidth - wallThick)/2.0);
		position.set(xPos, 0f, 0f);
		nextWall = new CMWall(sim, wallThick, channelHeight, channelDepth, position);
		nextWall.setColor(resColor[0], resColor[1], resColor[2]);
		sim.addBioObject(nextWall);
	}
	
	public void setSource(int numObjects, int direction){
		source = direction;
		sourceMolecules = numObjects;
	}
	
	public void setSink(int numObjects, int direction){
		sink = direction;
		sinkMolecules = numObjects;
	}
	
	public void updateChannel(CMSimulation sim, CMBioObjGroup group){
		updateReservoir(sim, LEFT, group);
		updateReservoir(sim, RIGHT, group);
		//System.out.println("Number of Molecules: " + group.getNumMembers());
	}
	
	public void updateReservoir(CMSimulation sim, int direction, CMBioObjGroup group){
		//Find the current number of molecules in the reservoir
		int numObj = group.getNumObjectsInside(group.getName(), getMinReservoirVector(direction), 
				getMaxReservoirVector(direction));
		int target = (direction == source) ? sourceMolecules : sinkMolecules;
		if (numObj < target){
			//add objects here
			CMBioObjGroup newGroup = CMMolecule.fillSpace(sim, (target-numObj), 
					getMinReservoirVector(direction), getMaxReservoirVector(direction), 
					group.getName());
			group.transferGroup(newGroup);
		}
		if (numObj > target){
			//remove random objects here - may not remove the exact number of objects
			int numToRemove = numObj - target;
			for (int i = 0; i < numToRemove; i++){
				int rand = (int)(sim.nextRandomF() * numObj);
				CMBioObj obj = group.getObject(rand);
				obj.markForRemoval();
			}
		}
	}
	
	public void writeConcentrationData(BufferedWriter output, CMBioObjGroup group){
		//System.out.println("Writing Concentration Data");
		//System.out.println(numSegments);
		String out = group.getNumMembers() + "\t";
		for (int i = 0; i < numSegments; i++){
			int numObj = group.getNumObjectsInside(group.getName(), getMinSegmentVector(i), 
					getMaxSegmentVector(i));
			out += numObj + "\t";
		}
		//System.out.println(out);
		try{
			output.write(out);
		}
		catch(IOException e){
			System.out.println("Could not write concentration data!");
			System.out.println("IOException: " + e.toString());
		}
		
	}
	
	public Vector3f getMinReservoirVector(int direction){
		Vector3f minVector = new Vector3f(0f, 0f, 0f);
		float xpos, ypos, zpos;
		if (direction == LEFT){
			xpos = (float)(channelWidth/2.0);
		}
		else{
			xpos = -(float)(channelWidth/2.0 + resWidth);
		}
		ypos = -(float)(channelHeight/2.0);
		zpos = -(float)(channelDepth/2.0);
		minVector.set(xpos, ypos, zpos);
		return minVector;
	}
	
	public Vector3f getMaxReservoirVector(int direction){
		Vector3f maxVector = new Vector3f(0f, 0f, 0f);
		float xpos, ypos, zpos;
		if (direction == LEFT){
			xpos = (float)(channelWidth/2.0 + resWidth);
		}
		else{
			xpos = -(float)(channelWidth/2.0);
		}
		ypos = (float)(channelHeight/2.0);
		zpos = (float)(channelDepth/2.0);
		maxVector.set(xpos, ypos, zpos);
		return maxVector;
	}
	
	public Vector3f getMinSegmentVector(int segment){
		float interspace = (float)(channelWidth/numSegments);
		float startX = -(float)(channelWidth/2.0);
		float xpos = startX + (segment * interspace);
		float ypos = -(float)(channelHeight/2.0);
		float zpos = -(float)(channelDepth/2.0);
		Vector3f minVector = new Vector3f(xpos, ypos, zpos);
		//System.out.println("   Min Segment " + segment + ": " + minVector);
		return minVector;
	}
	
	public Vector3f getMaxSegmentVector(int segment){
		float interspace = (float)(channelWidth/numSegments);
		float startX = -(float)(channelWidth/2.0);
		float xpos = startX + ((segment+1) * interspace);
		float ypos = (float)(channelHeight/2.0);
		float zpos = (float)(channelDepth/2.0);
		Vector3f maxVector = new Vector3f(xpos, ypos, zpos);
		//System.out.println("   Max Segment " + segment + ": " + maxVector);
		return maxVector;
	}
	
	public int getNumberSegments(){
		return numSegments;
	}
}
