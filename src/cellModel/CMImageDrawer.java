/**
 *  Copyright (C) 2013 Terri A. Grosso
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 *   This code was suggested here: http://www.lwjgl.org/wiki/index.php?title=Taking_Screen_Shots
 * 
 * Terri A. Grosso
 * Package: cellModel
 * File: CMWall.java
 * June 8, 2012 11:20:30 PM
 */

package cellModel;

import java.nio.ByteBuffer;
import java.io.File;
import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;
import java.io.IOException;

public class CMImageDrawer implements Runnable{
	
	private int width, height, bitPerPixel;
	private ByteBuffer buffer;
	private File imageFile;
	
	public CMImageDrawer(int w, int h, ByteBuffer bb, File f, int bpp){
		width = w;
		height = h;
		buffer = bb;
		imageFile = f;
		bitPerPixel = bpp;
	}
	
	public void run(){
		BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
		for(int x = 0; x < width; x++){
			for(int y = 0; y < height; y++){
				int i = (x + (width * y)) * bitPerPixel;
				int r = buffer.get(i) & 0xFF;
				int g = buffer.get(i + 1) & 0xFF;
				int b = buffer.get(i + 2) & 0xFF;
				image.setRGB(x, height - (y + 1), (0xFF << 24) | (r << 16) | (g << 8) | b);
			}
		}
		
		try {
			ImageIO.write(image, "PNG", imageFile);
		} catch (IOException e) {
			System.out.println("Could not write image");
			System.out.println(e.toString()); 
		}
	}
}
