/**
 * Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 * Package: cellModel
 * File: CMConstraint.java
 * Aug 31, 2013 1:25:14 PM
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

import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;

/**
 * @author tagsit
 *
 */
public interface CMConstraint {
	public void checkIn();
	public void updateTime();
	public boolean isActive();
	public boolean hasBeenActive();
	public void destroy();
	public long getCollId();
	public int getConId();
	public TypedConstraint getConstraint();
}