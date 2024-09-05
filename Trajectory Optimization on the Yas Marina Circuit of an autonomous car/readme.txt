There are 2 files ".m" and 4 folders in this deliverable.

File "main_MLPT.m" runs the NLP problem and its associated optimization routine. Then, it displays the solution with an animation.

File "results_visualization.m" loads one of the pre-computed solutions and show the results. 


Folder "Circuits":
	Contains two dataset:
			I) "simple_curve.mat" contains the data for a simple curve as described in the report
			II) "YasMarina.mat" contains the data for the Yas Marina Circuit (Abu Dhabi)



Folder "Functions":
	Contains the functions used in the program. Each one is detailed by its heading.



Folder "Parameters initialization":
	Contains 3 files:
			I) "collocation_parameters.m" initializes all the values for the direct orthogonal collocation method
			II) "tyre_parameters.m" initializes all the parameters related to the tires
			III)"vehicle_parameters.m" initializes all the parameters related to the vehicle


Folder "Solutions":
	Contains 2 subfolders, each one containing a dataset representing a solution obtained from the optimization problem as detailed in the report.
		
		"Simple Curve" subfolder contains one dataset:
			- "solution_simplecurve_dsk_col_30.mat" is the solution obtained on the simple curve with the parameters setting specified in the report, with dsk_col=30  
		
		"Yas Marina" subfolder contains two datasets:
			- "solution_YasMarina_dsk_col_50.mat" is the solution obtained on the Yas Marina circuit with the parameters setting specified in the report, with dsk_col=50 
			- "solution_YasMarina_dsk_col_70.mat" is the solution obtained on the Yas Marina circuit with the parameters setting specified in the report, with dsk_col=70 




