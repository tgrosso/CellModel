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
import java.nio.ByteBuffer;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.linearmath.Transform;

import static com.bulletphysics.demos.opengl.IGL.*;

/**
 * @author tagsit
 *
 */
public class CMMicrofluidicChannel {
	private float wallThick = 2f;
	private float channelWidth = 1000f, channelHeight = 100f, channelDepth = 90f;
	private int sourceConcentration, sinkConcentration;
	private float[] wallColor = {.4f, .2f, .2f};
	private float[] baseConcColor = {.4f, .2f, .2f};
	private int measureSegments = 5;
	private CMSimulation sim;
	private CMBioObjGroup cells;
	private boolean ligandAdded;
	private long currentTime;
	
	public CMMicrofluidicChannel(CMSimulation s, int srcConc, int snkConc){
		sourceConcentration = srcConc;
		sinkConcentration = snkConc;
		ligandAdded = false;
		sim = s;
		makeChannel(sim);
		currentTime = 0L;
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
		
		//left
		position.set((float)((channelWidth+wallThick)/2.0), 0f, 0f);
		nextWall = new CMWall(sim, wallThick, channelHeight, channelDepth, position);
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		sim.addBioObject(nextWall);
		
		//right
		position.set((float)(-(channelWidth+wallThick)/2.0), 0f, 0f);
		nextWall = new CMWall(sim, wallThick, channelHeight, channelDepth, position);
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		sim.addBioObject(nextWall);
		
		//front
		position.set(0f, 0f, -(float)((channelDepth+wallThick)/2.0));
		nextWall = new CMWall(sim, channelWidth, channelHeight, wallThick, position); 
		nextWall.setVisible(false);
		sim.addBioObject(nextWall);
		
		//sim.addBioObject(new InnerConcentration(sim, channelWidth, channelHeight, 1.0f, new Vector3f(0f, 0f, -(float)((channelDepth + wallThick)/2.0)), this));
		
		sim.setBaseCameraDistance(600f);
	}
	
	public void addLigand(){
		ligandAdded = true;
	}
	
	public void setTime(long time){
		currentTime = time;
	}
	
	public float getConcentration(long timeMs, float distFromSource){
		if (!ligandAdded){
			//No ligand added yet
			return 0f;
		}
		return 0f;
	}
	
	public Vector3f getMinChannelVector(){
		Vector3f minVector = new Vector3f();
		minVector.set((float)(-channelWidth/2.0),(float)(-channelHeight/2.0), (float)(-channelDepth/2.0));
		return minVector;
	}
	
	public Vector3f getMaxChannelVector(){
		Vector3f maxVector = new Vector3f();
		maxVector.set((float)(channelWidth/2.0),(float)(channelHeight/2.0), (float)(channelDepth/2.0));
		return maxVector;
	}
	
	public void writeConcentrationData(BufferedWriter output, long time){
		String out = "";
		float distanceBetweenMeasures = channelWidth / (measureSegments-1);
		for (int i = 0; i < measureSegments; i++){
			out += getConcentration(time, i * distanceBetweenMeasures) + "\t";
		}
		out += getConcentration(time, channelWidth);
		try{
			output.write(out);
		}
		catch(IOException e){
			System.out.println("Could not write concentration data!");
			System.out.println("IOException: " + e.toString());
		}
		
	}
	
	private class InnerConcentration extends CMWall{
		private float[] glMat = new float[16];
		private int drawnSegments;
		CMMicrofluidicChannel channel;
		ByteBuffer buf;
		//Stippling method based on that found here: http://lwjgl.org/forum/index.php?action=printpage;topic=1528.0
		
		private byte halftone[] = {
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55,
			    (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0xAA, (byte)0x55, (byte)0x55, (byte)0x55, (byte)0x55};
		
		public InnerConcentration(CMSimulation s, float w, float h, float d, Vector3f o, CMMicrofluidicChannel mc){
			super(s, w, h, d, o);
			body.setCollisionFlags(body.getCollisionFlags()|CollisionFlags.NO_CONTACT_RESPONSE);
			drawnSegments = 1000;
			channel = mc;
			buf = ByteBuffer.allocateDirect(1024+128);
			buf.put(halftone);
			buf.rewind();
		}
		public void collided(CMBioObj c, ManifoldPoint pt, boolean isObjA, long collId){
			System.out.println("Collision with innerConcentration");
		}
		public boolean specialRender(IGL gl, Transform t){
			//This just draws an overlay in front of 
			gl.glPushMatrix();
			t.getOpenGLMatrix(glMat);
			gl.glMultMatrix(glMat);
			gl.glBegin(GL_QUADS);
			for (int i = 0; i < drawnSegments; i++){
				float leftEdge = i * width/drawnSegments;
				float leftPercent = channel.getConcentration(channel.currentTime, leftEdge)/channel.sourceConcentration;
				
				float rightEdge = (i + 1) * width/drawnSegments;
				float rightPercent = channel.getConcentration(channel.currentTime, rightEdge)/channel.sourceConcentration;
				
				float baseRed = channel.baseConcColor[0];
				float remainRed = 1.0f - baseRed;
				float baseGreen = channel.baseConcColor[1];
				float remainGreen = 1.0f - baseGreen;
				float baseBlue = channel.baseConcColor[2];
				float remainBlue = 1.0f - baseBlue;
				
				gl.glColor3f(baseRed + leftPercent * remainRed, 
						baseGreen + leftPercent * remainGreen,
						baseBlue + leftPercent * remainBlue);
				//bottom left
				gl.glVertex3f(-leftEdge, -(float)(channelHeight/2.0), -(float)(channelDepth/2.0));
				//top left
				gl.glVertex3f(-leftEdge, (float)(channelHeight/2.0), -(float)(channelDepth/2.0));
				
				gl.glColor3f(baseRed + rightPercent * remainRed, 
						baseGreen + rightPercent * remainGreen,
						baseBlue + rightPercent * remainBlue);
				
				//top right
				gl.glVertex3f(leftEdge, (float)(channelHeight/2.0), -(float)(channelDepth/2.0));
				//bottom right
				gl.glVertex3f(leftEdge, -(float)(channelHeight/2.0), -(float)(channelDepth/2.0));
			}
			gl.glEnd();
			gl.glPopMatrix();
			
			return true;
		}
	}
}
