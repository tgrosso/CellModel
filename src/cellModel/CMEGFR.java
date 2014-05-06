package cellModel;

public class CMEGFR extends CMMembraneProtein{
	
	float k_off = .24f; //reverse rate of binding of ligand to receptor units /min
	float k_e = .15f; //internalization rate of bound receptor - units /min
	float k_t = .02f; //internalization rate of unbound receptor - units /min
	float k_D = 2.47f; //Dissociation constant - units nM
	//Data taken from human mammary epithelial cells - diameter about 15 micrometer
	long R_t = 200000; 
		//steady state receptor abundance for whole cell units molecules
	float k_on=k_off/k_D; //forward rate of ligand binding
	float Q_r = R_t * k_t; //Synthesization rate - units nM/min

	public CMEGFR(CMSimulation s, float[] base){
		super(s, (200000f / (float)(4 * Math.PI * 7.5 * 7.5)), (200000f / (float)(4 * Math.PI * 7.5 * 7.5)), base, "EGFR");
	}
	
	@Override
	protected long updateFreeReceptors(float ligandConcentration, long currentBound, long currentFree, float portion, float deltaTime){
		//ligandConcentration units is nM
		//receptor units are molecules
		//surface area units is micron^2
		//deltaT units is minutes
		//portion is the percentage of the synthesized receptors that should be added
		//using forward Euler method
		
		//System.out.println("***Updating Free EGFR***");
		//System.out.print("Ligand Conc " + ligandConcentration);
		long R = currentFree; //Number of free receptors on surface
		//System.out.print(" Free Receptors: " + R);
		long C = currentBound; //Number of bound receptors on surface
		//System.out.print(" Bound Receptors: " + C);
		float dR = deltaTime * (-k_on * R * ligandConcentration + k_off * C - k_t * R + Q_r * portion);
		//System.out.print(" delta R = " + dR);
		float newFree = R + dR;
		//System.out.println( " newFree = " + newFree);
		
		return (long)(newFree);
	}
	
	@Override
	protected long updateBoundReceptors(float ligandConcentration, long currentBound, long currentFree, float deltaTime){
		//ligandConcentration units is nM
		//receptor units are molecules
		//deltaT units is minutes
		//using forward Euler method
		//System.out.println("***Updating Bound EGFR***");
		//System.out.print("Ligand Conc " + ligandConcentration);
		long R = currentFree; //Number of free receptors on surface
		//System.out.print(" Free Receptors: " + R);
		long C = currentBound; //Number of bound receptors on surface
		//System.out.print(" Bound Receptors: " + C);
		float dC = deltaTime * (k_on * R * ligandConcentration - k_off * C - k_e * C);
		//System.out.print(" delta C = " + dC);
		float newBound = C + dC;
		//System.out.println( " newBound = " + newBound);
		
		return (long)(newBound);
	}
	
	public boolean bindsToLaminin(){
		return false;
	}
	
	public long getInitialProteins(float portion){
		return (long)(R_t * portion);
	}
}
