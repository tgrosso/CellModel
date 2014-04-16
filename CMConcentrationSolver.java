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

import java.io.PrintStream;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.File;
import java.io.IOException;

public class CMConcentrationSolver {
	//Recall units - length: micrometers
	//time - seconds
	String outputPath, solverPath;
	float min_x, max_x, time, sourceConc, sinkConc;
	String scriptFile, templateFile, concentrationFile, distanceFile;
	int timeSteps = 100, distSteps = 100;
	long lastTime, nextTime;
	double[] previousConcentrations, nextConcentrations;
	boolean usePDE;
	
	public CMConcentrationSolver(String oPath, String sPath, float minx, float maxx, float t, float source, float sink)
		throws IOException{
		min_x = minx;
		max_x = maxx;
		time = t; //This is time in SECONDS
		sourceConc = source;
		sinkConc = sink;
		solverPath = sPath;
		outputPath = oPath;
		scriptFile = "script";
		templateFile = "template";
		concentrationFile = "concentrations.csv";
		distanceFile = "distancesFromSink.csv";
		previousConcentrations = new double[distSteps];
		nextConcentrations = new double[distSteps];
		for (int i = 0; i < distSteps; i++){
			previousConcentrations[i] = sinkConc;
			nextConcentrations[1] = sinkConc;
		}
		lastTime = 0;
		usePDE = sourceConc > sinkConc;
		if (usePDE){
			writeTemplate();
			System.out.println("Solving Differential Equations. Please wait.");
			try {
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
		//time in milliseconds
		//distance in micrometers
		if (sourceConc == sinkConc){
			return sourceConc;
		}
		if (!usePDE){ //source Concentration is 0 or less
			return 0.0f;
		}
		return 0.0f;
	}
}
