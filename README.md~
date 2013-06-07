Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 
    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 
    The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

CellModel is a program to model the migration of cells

The simulation is extended from JBullet's BasicDemo application.
The JBullet code had to be recompiled with the changes suggested
	here: http://blog.lolyco.com/sean/2011/05/14/trying-out-jbullet-intbuffer-is-not-direct/

TO DO:

Set the initial camera distance - *Done*

Reproduce model of transwell experiment

Model the walls of the test tube - *Done*

Model the "molecules" of chemoattractant - *Done*

	Brownian motion is simulated by changes in velocity. *Done* Should it be application of random forces?

Model the transwell mesh - *Done*

Model cells in the insert

	Cells move randomly. *Done*

	Cells move randomly with a downward force due to gravity that is modified by bouyancy. *Done*

	Cells are attracted to the walls and mesh - how to model?

Model cells reacting to collisions with molecules

	Cells move towards the highest concentration of chemoattractant

	Cells move randomly, perferentially towards the chemoattractant

	Sections of cell membrane apply force to the overall cells (like pseudopodia pulling the cell along). 

		Overall force on the cell is the sum of these force vectors. 

		Sections activated by chemoattractant are more likely to apply a force.

