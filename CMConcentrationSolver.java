/**
 * Copyright (C) 2014 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 * Package: cellModel
 * File: CMConcentrationSolver.java
 * Jan 12, 2014 3:45:03 PM
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

import java.io.FileInputStream;
import java.io.PrintStream;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.File;
import java.io.IOException;
import java.io.FileNotFoundException;

public class CMConcentrationSolver {
	//Recall units - length: micrometers
	//time - seconds
	String outputPath, solverPath;
	float min_x, max_x, sourceConc, sinkConc;
	String scriptFile, templateFile, concentrationFile, distanceFile;
	int timeSteps = 100, distSteps = 200;
	long time, timeToSteady, lastTime, nextTime;
	double[] previousConcentrations, nextConcentrations, distances;
	boolean usePDE;
	BufferedReader concReader = null;
	boolean finalTimeReached = false;
	
	public CMConcentrationSolver(String oPath, String sPath, float minx, float maxx, long t, long tts, float source, float sink){
		min_x = minx;
		max_x = maxx;
		time = t; //This is time in SECONDS - to conform to differential solving code
		timeToSteady = tts; //Time in SECONDS - to steady state
		sourceConc = source;
		sinkConc = sink;
		solverPath = sPath;
		outputPath = oPath;
		scriptFile = "script";
		templateFile = "template";
		concentrationFile = "concentrations.csv";
		distanceFile = "distancesFromSource.csv";
		distances = new double[distSteps];
		previousConcentrations = new double[distSteps];
		nextConcentrations = new double[distSteps];
		for (int i = 0; i < distSteps; i++){
			previousConcentrations[i] = sinkConc;
			nextConcentrations[i] = sinkConc;
			distances[i] = 0;
		}
		lastTime = 0;
		nextTime = 0;
		usePDE = sourceConc > sinkConc;
		if (usePDE){
			try {
				writeTemplate();
				System.out.println("Solving Differential Equations. Please wait.");
				Process p = Runtime.getRuntime().exec("octave " + outputPath + "/" +scriptFile);
            
				BufferedReader reader = new BufferedReader(new InputStreamReader(p.getErrorStream()));
				String line=reader.readLine();
				while (line != null) {    
					System.out.println(line);
					line = reader.readLine();
				}
				if (p.waitFor() !=0){
					System.out.println("Error solving concentration differential equations.");
				}

			}
			catch(IOException e1) {
				System.err.println("Cannot use differential equations - using linear gradient");
				usePDE = false;
			}
			catch(InterruptedException e2) {
				System.err.print("Cannot use differential equations - using linear gradient");
				usePDE = false;
			}
		}
		if (usePDE){
			//read in the distances that were used in the equations
			try{
				File distFile = new File(outputPath, distanceFile);
				FileInputStream fis = new FileInputStream(distFile);
				InputStreamReader isr = new InputStreamReader(fis);
				BufferedReader br = new BufferedReader(isr);
				String line = br.readLine();
				String[] values = line.split(",");
				
				for (int i = 1; i < distSteps; i++){
					try{
						distances[i] = Float.parseFloat(values[i]);
						//System.out.println("distances " + i + ": " + distances[i]);
					}
					catch(NumberFormatException e){
						System.err.println("Error reading distances!");
						usePDE = false;
					}
				}
				br.close();
				isr.close();
				fis.close();
			}
			catch(FileNotFoundException e){
				System.err.print("Cannot read distance file. Using Linear gradient");
				usePDE = false;
			}
			catch(IOException a){
				System.err.print("Cannot read distance file. Using Linear gradient");
				usePDE = false;
			}
		}
		if (usePDE){
			//read in the initial ligand concentrations
			try{
				File concFile = new File(outputPath, concentrationFile);
				FileInputStream fis = new FileInputStream(concFile);
				InputStreamReader isr = new InputStreamReader(fis);
				concReader = new BufferedReader(isr);
				readNextConcentration();
				readNextConcentration();
			}
			catch(FileNotFoundException e){
				System.err.print("Cannot read concentration file. Using Linear gradient");
				usePDE = false;
			}
		}
		/*
		System.out.println("Use PDE? " + usePDE);
		System.out.println("Testing concentration solver");
		System.out.println("Find Concentration at distance 0 micron from source at time 0. (Should be 6.6)");
		System.out.println(getConcentration(0, 0));
		System.out.println("Find Concentration at distance 1000 micron from source at time 0. (Should be 0)");
		System.out.println(getConcentration(1000, 0));
		System.out.println("Find Concentration at distance 13000 micron from source at time 0. (Should be 0)");
		System.out.println(getConcentration(13000, 0));
		System.out.println("Find Concentration at distance 0 micron from source at time 5000. (Should be 6.6)");
		System.out.println(getConcentration(0, 5000));
		System.out.println("Find Concentration at distance 1000 micron from source at time 5000. (Should be .003641288994)");
		System.out.println(getConcentration(1000, 5000));
		System.out.println("Find Concentration at distance 13000 micron from source at time 5000. (Should be 0)");
		System.out.println(getConcentration(13000, 5000));
		System.out.println("Find Concentration at distance 12900 micron from source at time 1000000. (Should be 1.176647787*10^-42)");
		System.out.println(getConcentration(12900, 1000000));
		*/
		
		System.out.println("Testing time to reach");
		System.out.println("Find time to reach 0 micron from source with threshold .1 (should be 0)");
		System.out.println(timeToReach(0, .1f));
		System.out.println("Find time to reach 2700 micron from source with threshold .1 (should be 0)");
		System.out.println(timeToReach(2700, .1f));
	}
	
	private void writeTemplate() throws IOException{
		PrintStream scriptOut = new PrintStream(new File(outputPath, scriptFile));
		scriptOut.println("addpath(\"" + outputPath + "\");");
		scriptOut.println("addpath(\"" + solverPath + "\");");
		scriptOut.println(templateFile);
		scriptOut.close();
		
		File tempFile = new File(outputPath, templateFile + ".m");
		File concFile = new File(outputPath, concentrationFile);
		File distFile = new File(outputPath, distanceFile);
		PrintStream tempOut = new PrintStream(tempFile);
		tempOut.println("function " + templateFile);
		tempOut.println("m=0;\n min_x = " + min_x + ";\n max_x = " + max_x + ";\n x_steps = " + distSteps + ";");
		tempOut.println("x = linspace(min_x, max_x, x_steps);");
		
		tempOut.println("min_t = 0;\n max_t = " + time + ";\n t_steps = " + timeSteps + ";");
		tempOut.println("t = linspace(min_t, max_t, t_steps);");

		tempOut.println("sol = pdepe(m,@pdex1pde,@pdex1ic,@pdex1bc,x,t);");
		tempOut.println("u = sol(:,:,1);");
		tempOut.println("u=fliplr(u);");
		tempOut.println("x = " + max_x + " .- x;");
		tempOut.println("x=flipdim(x);");

		tempOut.println("csvwrite('" + distFile.getAbsolutePath() + "', x)");
		tempOut.println("csvwrite('" + concFile.getAbsolutePath() + "', [t(:) u])");
		
		tempOut.println("function [c,f,s] = pdex1pde(x,t,u,DuDx)");
		tempOut.println("  c = 1;");
		tempOut.println("  f =2.0e2*DuDx;");
		tempOut.println("  s = 0;");

		tempOut.println("function u0 = pdex1ic(x)");
		tempOut.println("  if x<" + max_x);
		tempOut.println("    u0=0;");
		tempOut.println("  else u0 = " + sourceConc + ";");
		tempOut.println("end");
		
		tempOut.println("function [pl,ql,pr,qr] = pdex1bc(xl,ul,xr,ur,t)");
		tempOut.println("  pl = ul;");
		tempOut.println("  ql = 0;");
		tempOut.println("  pr = ur-" + sourceConc + ";");
		tempOut.println("  qr = 0;");
	}
	
	public float getConcentration(float distFromSource, long experimentalTime){
		//experimental time is different from simulation time
		//experimental time in milliseconds
		//distance in micrometers
		if (sourceConc == sinkConc){
			return sourceConc;
		}
		if (!usePDE){ //The pde solver did not initialize correctly. Use a linear gradient
			float frontSpeed = (max_x) / (timeToSteady * 1000); //microns per millisecond
			float frontPosition = frontSpeed * experimentalTime;
			if (frontPosition > max_x){
				frontPosition = max_x;
			}
			if (frontPosition < distFromSource){
				return sinkConc; //front of source concentration hasn't gotten there yet
			}
			else{
				float slope = (sourceConc - sinkConc) / frontPosition;
				return sinkConc + (slope * (frontPosition - distFromSource)); 
			}
		}
		//Fill in the edge cases
		if (distFromSource <= 0){
			return sourceConc; //we are at the source 
		}
		if (distFromSource >= max_x){
			return sinkConc; //we are at the sink
		} 
		if (experimentalTime == 0){
			//By definition, at time = 0, it's the source conc at the xource and sink conc everywhere else
			//distFromSource can't be <= 0 because that has already been taken care of
			return sinkConc;
		}
		
		//If here, we need the data from the solution files
		//Is the most recent time at or before the experimental Time? We assume
		//the experimental Time does not go backwards!
		//TODO But COULD re-load and go back - think about it.
		if (experimentalTime < lastTime){
			System.err.println("Why has experimental time gone backwards? Major problem. Using linear gradient.");
			usePDE = false;
			return getConcentration(distFromSource, experimentalTime);
		}
		while (experimentalTime > nextTime && !finalTimeReached){
			readNextConcentration();
		}
		
		if (finalTimeReached){
			experimentalTime = nextTime; //This is the final time that we have data for and the concentrations don't change after this
		}
		//How far between the times are we?
		double timePer = (double)(experimentalTime - lastTime)/(nextTime - lastTime);
		//System.out.println("timePer " + timePer);
		float value = 0f;
		//Now find where in the distances we are
		int distIndex = 0;
		while (distIndex < distSteps && distFromSource > distances[distIndex]){
			//System.out.println("distIndex " + distIndex + " distances[index] " + distances[distIndex]);
			distIndex++;
		}
		//System.out.println("distIndex " + distIndex);
	
		if (distIndex >= distSteps){
			//get concentration at the sink end of the channel
			return sinkConc;
		}
		if (distIndex == 0){
			//get concentration at the source
			return sourceConc;
		}
		//System.out.println("prevDist " + distances[distIndex-1] + " nextDist " + distances[distIndex]);
		//Find the concentration at this point and this time
		double distPer = (distFromSource - distances[distIndex-1])/(distances[distIndex] - distances[distIndex-1]);
		//System.out.println("distPer " + distPer);
		//System.out.println("nextConc[distIndex-1] " + nextConcentrations[distIndex-1]);
		//System.out.println("nextConc[distIndex] " + nextConcentrations[distIndex]);
		double nextDiff = nextConcentrations[distIndex] - nextConcentrations[distIndex-1];
		//System.out.println("nextDiff " + nextDiff);
		double prevDiff = previousConcentrations[distIndex] - previousConcentrations[distIndex-1];
		//System.out.println("prevDiff " + prevDiff);
		double nConc = nextConcentrations[distIndex-1] + (distPer * nextDiff);
		//System.out.println("nConc " + nConc);
		double pConc = previousConcentrations[distIndex-1] + (distPer * prevDiff);
		//System.out.println("pConc " + pConc);
		double concDiff = nConc - pConc;
		//System.out.println("concDiff " + concDiff);
		value = (float)(pConc + timePer * concDiff);
		return value;
	}
	
	private void readNextConcentration(){
		long newTime = 0;
		double[] newConcentrations = new double[distSteps];
		try{
			String line = concReader.readLine();
			if (line == null){
				finalTimeReached = true;
				return;
			}
			String[] values = line.split(",");
			//System.out.println("Length of values: " + values.length);
			newTime = (long)(Float.parseFloat(values[0]) * 1000); //function times are in milliseconds
			//System.out.println("New time: " + newTime);
			for (int i = 1; i <= distSteps; i++){
				newConcentrations[i-1] = Double.parseDouble(values[i]);
				//System.out.print(newConcentrations[i-1] + " ");
			}
			//System.out.println("");
		}
		catch(NumberFormatException e){
			System.err.println("Number Format Exception reading concentrations. UGH! What to do?");
			usePDE = false;
		}
		catch(IOException e){
			System.err.println("IOException reading concentrations. UGH! What to do?");
			usePDE = false;
		}
		lastTime = nextTime;
		nextTime = newTime;
		previousConcentrations = nextConcentrations;
		nextConcentrations = newConcentrations;
		//System.out.println("last Time " + lastTime + "Next Time " + nextTime);
	}
	
	public long timeToReach(float distanceFromSource, float threshold){
		//return time in milliseconds
		if (threshold > sourceConc){
			System.err.println("Threshold cannot be greater than source concentration!");
			System.err.println("Returning time to steady state.");
			return timeToSteady * 1000;
		}
		if (sourceConc == sinkConc){
			//If there is no gradient, all parts of the channel are the same
			return 0;
		}
		if (distanceFromSource == 0){
			//You are already at the maximum concentration
			return 0;
		}
		if (distanceFromSource > max_x){
			System.err.println("Outside of channel! Returning time to steady state.");
			return timeToSteady * 1000;
		}
		if (!usePDE){
			//Using a linear gradient, find how long it takes for this distance to reach this threshold
			double distToFront = (sourceConc * distanceFromSource) / (sourceConc - threshold);
			float frontSpeed = (max_x) / (timeToSteady * 1000);
			return (long)(distToFront/frontSpeed);
		}
		//Must use the concentration file to find out how long it takes for this threshold to be reached at this distance from source
		//First find where the distance falls
		int distIndex = 0;
		while (distIndex < distSteps && distanceFromSource > distances[distIndex]){
			//System.out.println("distIndex " + distIndex + " distances[index] " + distances[distIndex]);
			distIndex++;
		}
		//System.out.println("distIndex " + distIndex);
		//System.out.println("distances[distIndex] " + distances[distIndex] + " distances[distIndex+1 " + distances[distIndex+1]);
		
		try{
			File concFile = new File(outputPath, concentrationFile);
			FileInputStream fis = new FileInputStream(concFile);
			InputStreamReader isr = new InputStreamReader(fis);
			BufferedReader timeReader = new BufferedReader(isr);
			
			String line = timeReader.readLine();
			float t1 = -1, t2 = -1;
			float oldHigh = 0, oldLow = 0;
			long oldTime = 0;
			String[] values = null;
			while (line != null){
				values = line.split(",");
				float newHigh = Float.parseFloat(values[distIndex]);
				float newLow = Float.parseFloat(values[distIndex+1]);
				long newTime = (long)(Float.parseFloat(values[0]) *  1000);
				//System.out.println("newHigh " + newHigh + " newLow " + newLow + " newTime " + newTime);
				if (newHigh > threshold && t1 < 0){
					//This is the first time that the concentration is over the threshold
					t1 = oldTime + ((newTime-oldTime) * (threshold - oldHigh) / (newHigh - oldHigh));
				}
				if (newLow > threshold && t2 < 0){
					//This is the first time that the concentration is over the threshold
					t2 = oldTime + ((newTime-oldTime) * (threshold - oldLow) / (newLow - oldLow));
				}
				//System.out.println("t1 " + t1 + " t2 " + t2);
				if (t1 >= 0 && t2 >= 0){
					break;
				}
				oldHigh = newHigh;
				oldLow = newLow;
				oldTime = newTime;
				line = timeReader.readLine();
			}
			if (line == null){
				return oldTime; //This is hte maximum time possible.
			}
			long thresholdTime = (long)(t2 + ((t1-t2) * (distanceFromSource - distances[distIndex])/(distances[distIndex-1]-distances[distIndex])));
			return thresholdTime;
		}
		catch(NumberFormatException a){
			System.err.println("Error in concentration file! " + a);
		}
		catch(IOException e){
			System.err.print("Cannot read concentration file. Using Linear gradient" + e);
		}
		return (timeToSteady*1000); //I guess this is as good a default as any!
	}
	
	public float getSourceConcentration(){
		return sourceConc;
	}
	
	public float getSinkConcentration(){
		return sinkConc;
	}
	
	public void destroy(){
		try{
			if (concReader != null){
				concReader.close();
			}
		}
		catch(IOException e){
			System.err.println("Error closing concentration file. " + e.toString());
		}
	}
}
