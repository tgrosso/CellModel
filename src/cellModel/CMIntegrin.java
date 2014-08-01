/**
 * Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 * Package: cellModel
 * File: CMIntegrin.java
 * Nov 9, 2013 10:28:15 AM
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

/**
 * @author tagsit
 *
 */
public class CMIntegrin extends CMMembraneProtein {
	
	//These are values for EGF.  Need to find new values for CMMembraneProtein
	float k_off = .24f; //reverse rate of binding of ligand to receptor units /min
	float k_e = .15f; //internalization rate of bound receptor - units /min
	float k_t = .02f; //internalization rate of unbound receptor - units /min
	float k_D = 2.47f; //Dissociation constant - units nM
	//Data taken from human mammary epithelial cells - diameter about 15 micrometer
	long R_t = 200000; 
		//steady state receptor abundance for whole cell units molecules
	float k_on=k_off/k_D; //forward rate of ligand binding
	float Q_r = R_t * k_t; //Synthesization rate - units molecules/min
	boolean attaches = true;

	public CMIntegrin(CMSimulation s, float[] base){
		super(s, (200000f / (float)(4 * Math.PI * 7.5 * 7.5)), (200000f / (float)(4 * Math.PI * 7.5 * 7.5)), base, "EGFR");
		k_on = .05f;
		Q_r = R_t * k_t;
	}
	
	@Override
	protected float getBoundEndocytosisRate(){
		return k_e; //units /min
	}
	
	@Override
	protected float getUnboundEndocytosisRate(){
		return k_t; //units /min
	}
	
	@Override
	protected float getExocytosisRate(){
		return Q_r; //units molecules/min
	}
	
	@Override
	protected float updateFreeReceptors(float ligandConcentration, float currentBound, float currentFree, float freeEndo, float exo, float deltaTime){
		//For integrins, we don't use the ligandConcentration - they bind in a different method
		//receptor units are molecules
		//rate units are /min
		//exo units are molecules/min
		//deltaT units is minutes
		//using forward Euler method

		float R = currentFree; //Number of free receptors on surface
		float dR = deltaTime * (-freeEndo * R + exo);
		float newFree = R + dR;
		return (newFree);
	}
	
	@Override
	protected float updateBoundReceptors(float ligandConcentration, float currentBound, float currentFree, float boundEndo, float deltaTime){
		//for integrins, the number of bound receptors is solely based on the 
		//number of constraints that are made.
		//We only use all of these parameters because it is a membrane protein
		return (currentBound);
	}
	
	@Override
	public float getInitialProteins(float portion){
		return (R_t * portion);
	}
	
	@Override
	public boolean bindsToLaminin(){
		return attaches;
	}
	
	@Override
	public int bindReceptors(int numLigands, int numFreeReceptors){
		//float deltaTime = sim.getDeltaTimeMilliseconds() / 1000f / 60f;
		//return (int)(Math.round(numLigands * numFreeReceptors * k_on * deltaTime));
		int possibleBonds = Math.min(numLigands, numFreeReceptors);
		int bonds = (int)(sim.nextRandomF() * k_on * possibleBonds);
		return bonds;
	}
	
	@Override
	public String getName(){
		return "Integrin";
	}
}
