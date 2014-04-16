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
	float k_t = .02f; //internalization rate of unbound recept - units /min
	float k_D = 2.47f; //Dissociation constant - units nM
	//Data taken from human mammary epithelial cells - diameter about 15 micrometer
	float R_t = 200000f / (float)(4 * Math.PI * 7.5 * 7.5); 
		//steady state receptor abundance - units molecules/micron^2
	float k_on; //forward rate of ligand binding
	float Q_r; //Synthesization rate - units nM/min
	boolean attaches = true;

	public CMIntegrin(CMSimulation s, float[] base){
		super(s, (200000f / (float)(4 * Math.PI * 7.5 * 7.5)), (200000f / (float)(4 * Math.PI * 7.5 * 7.5)), base, "EGFR");
		k_on = .7f;
		Q_r = R_t * k_t;
	}
	
	@Override
	protected float updateFreeReceptors(float ligandConcentration, float currentBound, float currentFree, float portion, float deltaTime){
		//ligandConcentration units is nM
		//receptor units are molecules
		//deltaT units is minutes
		//using forward Euler method
		System.out.println("***Updating Free Integrins***");
		System.out.print("Ligand Conc " + ligandConcentration);
		float R = currentFree; //Number of free receptors on surface
		System.out.print(" Free Receptors: " + R);
		float C = currentBound; //Number of bound receptors on surface
		System.out.print(" Bound Receptors: " + C);
		float dR = deltaTime * (-k_on * R * ligandConcentration + k_off * C - k_t * R + Q_r * portion);
		System.out.print(" delta R = " + dR);
		float newFree = R + dR;
		System.out.println( " newFree = " + newFree);
				
		return newFree;
	}
	
	@Override
	protected float updateBoundReceptors(float ligandConcentration, float currentBound, float currentFree, float deltaTime){
		//ligandConcentration units is nM
		//receptor units are molecules/micon^2
		//surface area units is micron^2
		//deltaT units is minutes
		//using forward Euler method
		System.out.println("***Updating Bound Integrins***");
		System.out.print("Ligand Conc " + ligandConcentration);
		float R = currentFree; //Number of free receptors on surface
		System.out.print(" Free Receptors: " + R);
		float C = currentBound; //Number of bound receptors on surface
		System.out.print(" Bound Receptors: " + C);
		float dC = deltaTime * (k_on * R * ligandConcentration - k_off * C - k_e * C);
		System.out.print(" delta C = " + dC);
		float newBound = C + dC;
		System.out.println( " newBound = " + newBound);
		
		return (C + dC);
	}
	
	public int bindReceptors(int numLigands, int numFreeReceptors){
		int boundProteins = Math.round(k_on * Math.min(numLigands, numFreeReceptors));
		//System.out.println(numLigands + ", " + numFreeReceptors + ", " + boundProteins);
		return boundProteins;
	}
	
	public boolean bindsToLaminin(){
		return attaches;
	}
	
	public String getName(){
		return "Integrin";
	}
}

