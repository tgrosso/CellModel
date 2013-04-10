CellModel is a program to model the migration of cells

The simulation is extended from JBullet's BasicDemo application.
The JBullet code had to be recompiled with the changes suggested
	here: http://blog.lolyco.com/sean/2011/05/14/trying-out-jbullet-intbuffer-is-not-direct/

TO DO:
Set the initial camera distance (why is this difficult?)
Reproduce model of transwell experiment
Model the walls of the test tube - *Done*
Model the "molecules" of chemoattractant - *Done*
	Brownian motion is simulated by changes in velocity. Should it be application of random forces?
Model the transwell mesh
Model cells in the insert
	Cells move randomly with a downward force due to gravity that is modified by bouyancy
	Cells are attracted to the walls and mesh - how to model?
Model cells reacting to collisions with molecules
	Cells move towards the highest concentration of chemoattractant
	Cells move randomly, perferentially towards the chemoattractant
	Sections of cell membrane apply force to the overall cells (like pseudopodia pulling the cell along). 
		Overall force on the cell is the sum of these force vectors. 
		Sections activated by chemoattractant are more likely to apply a force.
