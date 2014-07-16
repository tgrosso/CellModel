/**
 * Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 * Package: cellModel
 * File: CMMembraneProtein.java
 * Aug 21, 2013 2:30:04 PM
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

import com.bulletphysics.util.ObjectArrayList;
import java.util.Vector;

/**
 * @author tagsit
 *
 */
public class CMMembraneProtein {
	protected CMSimulation sim;
	protected float k_e = 0; //internalization rate of bound receptor - units /min
	protected float k_t = 0; //internalization rate of unbound receptor - units /min
	protected float maximumDensity;
	protected float baseDensity;
	protected float[] baseColor;
	protected String name;
	public float avosNum = (float)(6.022e23);
	public float k_on = 0;
	protected float Q_r = 0; //Synthesization rate - units nM/min
	
	public CMMembraneProtein(CMSimulation s, float maxDen, float baseDen, float[] base, String n){
		sim = s;
		maximumDensity = maxDen;
		baseDensity = baseDen;
		baseColor = base;
		name = n;
	}
	
	protected float getBoundEndocytosisRate(){
		return k_e;
	}
	
	protected float getUnboundEndocytosisRate(){
		return k_t;
	}
	
	protected float getExocytosisRate(){
		return Q_r;
	}
	protected float[] getBaseColor(){
		return baseColor;
	}
	
	protected String getName(){
		return name;
	}
	
	protected float getBaseDensity(){
		return baseDensity;
	}
	
	protected float getMaxDensity(){
		return maximumDensity;
	}
	
	protected long getInitialProteins(float portion){
		return 0;
	}
	
	protected long updateFreeReceptors(float ligandConcentration, long currentBound, long currentFree, float freeEndo, float exo, float deltaTime){
		return 0;
	}
	
	protected long updateBoundReceptors(float ligandConcentration, long currentBound, long currentFree, float boundEndo, float deltaTime){
		return 0;
	}
	
	public boolean bindsToLaminin(){
		return false;
	}
	
	public int bindReceptors(int numLigands, int numFreeReceptors){
		return 0;
	}
	
}