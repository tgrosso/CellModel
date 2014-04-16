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
	protected float maximumDensity;
	protected float baseDensity;
	protected float[] baseColor;
	protected String name;
	
	public CMMembraneProtein(CMSimulation s, float maxDen, float baseDen, float[] base, String n){
		sim = s;
		maximumDensity = maxDen;
		baseDensity = baseDen;
		baseColor = base;
		name = n;
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
	
	protected float updateFreeReceptors(float ligandConcentration, float currentBound, float currentFree, float portion, float deltaTime){
		return 0f;
	}
	
	protected float updateBoundReceptors(float ligandConcentration, float currentBound, float currentFree, float deltaTime){
		return 0f;
	}
	
	public boolean bindsToLaminin(){
		return false;
	}
	
	public int bindReceptors(int numLigands, int numFreeReceptors){
		return 0;
	}
	
}
