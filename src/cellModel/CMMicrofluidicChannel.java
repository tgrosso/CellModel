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
import org.lwjgl.opengl.GL11;

import static com.bulletphysics.demos.opengl.IGL.*;


/**
 * @author tagsit
 *
 */
public class CMMicrofluidicChannel {
	private float wallThick = 2f;
	private float channelWidth = 300f, channelHeight = 90f, channelDepth = 100f;
	private float sourceConcentration, sinkConcentration;
	private float distFromSource, totalLength;
	private float[] wallColor = {.4f, .2f, .2f};
	private float[] baseConcColor = {1.0f, .9f, .9f};
	private int measureSegments = 5;
	private CMSimulation sim;
	private CMConcentrationSolver solver;
	private long timeToReach = 0;
	
	
	public CMMicrofluidicChannel(CMSimulation s, float d, CMConcentrationSolver sol){
		sim = s;
		makeChannel(sim);
		distFromSource = d;
		totalLength = 13000f; //13 * 10^5 microns is total length of channel
		solver = sol;
		sourceConcentration = solver.getSourceConcentration();
		sinkConcentration = solver.getSinkConcentration();
		
		if (distFromSource + channelWidth > totalLength){
			distFromSource = totalLength - channelWidth;
		}
		timeToReach = sol.timeToReach(distFromSource, .01f);
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
		nextWall.addLaminin();
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
		nextWall.setVisible(false);
		sim.addBioObject(nextWall);
		
		//left
		position.set((float)((channelWidth+wallThick)/2.0), 0f, 0f);
		nextWall = new CMWall(sim, wallThick, channelHeight, channelDepth, position);
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		nextWall.setVisible(false);
		sim.addBioObject(nextWall);
		
		//right
		position.set((float)(-(channelWidth+wallThick)/2.0), 0f, 0f);
		nextWall = new CMWall(sim, wallThick, channelHeight, channelDepth, position);
		nextWall.setColor(wallColor[0], wallColor[1], wallColor[2]);
		nextWall.setVisible(false);
		sim.addBioObject(nextWall);
		
		//front
		position.set(0f, 0f, -(float)((channelDepth+wallThick)/2.0));
		nextWall = new CMWall(sim, channelWidth, channelHeight, wallThick, position); 
		nextWall.setVisible(false);
		sim.addBioObject(nextWall);
		
		sim.addBioObject(new ConcentrationOverlay(this));
		
		sim.setBaseCameraDistance(110f);
	}
	
	
	public float getConcentration(float distFromMin, long time){
		//time is in milliseconds - NOT microseconds!
		float dist = distFromSource + distFromMin;
		long experimentalTime = time + timeToReach;
		return solver.getConcentration(dist, time);
	}
	
	
	
	public Vector3f getMinChannelVector(){
		Vector3f minVector = new Vector3f();
		minVector.set((float)(-channelWidth/2.0),(float)(-channelHeight/2.0), (float)(-channelDepth/2.0));
		//System.out.println(minVector);
		return minVector;
	}
	
	public Vector3f getMaxChannelVector(){
		Vector3f maxVector = new Vector3f();
		maxVector.set((float)(channelWidth/2.0),(float)(channelHeight/2.0), (float)(channelDepth/2.0));
		//System.out.println(maxVector);
		return maxVector;
	}
	
	public long getTimeToReach(){
		return timeToReach;
	}
	
	public void writeConcentrationData(BufferedWriter output, long time, boolean title){
		//Units of time are miliseconds
		String out = "";
		float distanceBetweenMeasures = channelWidth / (measureSegments-1);
		
		for (int i = 0; i < measureSegments; i++){
			if (title){
				out+= ((distFromSource + (i * distanceBetweenMeasures))/1000) + " mm\t";
			}
			else{
				out += getConcentration(i * distanceBetweenMeasures, time) + "\t";
			}
		}
	
		
		try{
			output.write(out);
			if (title){
				output.newLine();
			}
		}
		catch(IOException e){
			System.out.println("Could not write concentration data!");
			System.out.println("IOException: " + e.toString());
		}
		
	}
	
	public float getDistanceFromMinimum(float x_value){
		return (x_value - getMinChannelVector().x);
	}
	
	private class ConcentrationOverlay extends CMWall{
		private float[] glMat = new float[16];
		CMMicrofluidicChannel channel;
		int drawnSegments;
		
		public ConcentrationOverlay(CMMicrofluidicChannel mc){
			super(mc.sim, mc.channelWidth, mc.channelHeight, 1f, new Vector3f(0f, 0f, (float)((mc.channelDepth+mc.wallThick+4)/2.0)));
			body.setCollisionFlags(body.getCollisionFlags()|CollisionFlags.NO_CONTACT_RESPONSE);
			drawnSegments = 100;
			channel = mc;
			this.setColor(.9f, .9f, .2f, .2f);
		}
		
		public void collided(CMBioObj c, ManifoldPoint pt, boolean isObjA, long collId){
		}
		
		public boolean specialRender(IGL gl, Transform t){
			//This just draws an representation of the concentration gradient behind the channel
			float right = -channel.channelWidth/2;
			float bottom = -channel.channelHeight/2;
			float z = 0;
			//System.out.println("left: " + left + " right: " + right + " top: " + top + " bottom: " + bottom);
		
			float blockWidth = channelWidth/drawnSegments;
			//get the time in milliseconds
			long ti = channel.sim.getCurrentTimeMicroseconds()/1000;
			//System.out.println("blockwidth: " + blockWidth + " time: " + ti); 
			
			gl.glPushMatrix();
			
			t.getOpenGLMatrix(glMat);
			gl.glMultMatrix(glMat);
			GL11.glColor3f(channel.baseConcColor[0], channel.baseConcColor[1], channel.baseConcColor[2]);
			GL11.glNormal3f( 0f, 0f, -1f); 
			GL11.glBegin(GL_QUADS);
			for (int i = 0; i < drawnSegments; i++){
				float ri = i * blockWidth;
				float le = ri + blockWidth; 
				float con = channel.getConcentration(ri, ti);
				float mi = con/(channel.sourceConcentration - channel.sinkConcentration) * channel.channelHeight + bottom;
				//System.out.println(i + "ri: " + ri + " le: " + le + " con: " + con + " mi: " + mi);
				GL11.glVertex3f(le+right,mi,z);
				GL11.glVertex3f(ri+right,mi,z);
				GL11.glVertex3f(ri+right,bottom,z);
				GL11.glVertex3f(le+right,bottom,z);
			}
			GL11.glEnd();
			
			gl.glPopMatrix();
			
			return true;
		}
	}
}
