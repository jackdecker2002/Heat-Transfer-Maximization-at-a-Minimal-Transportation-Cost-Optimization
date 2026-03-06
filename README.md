# Heat-Transfer-Maximization-at-a-Minimal-Transportation-Cost-Optimization

## Overview
This project showcases my role within a team where the mission was to maximize heat transfer of a heat sink for a CPU while also maximizing profit. The team worked together to define reasonable constraints based on real world limitations. The overall structure of the project fell mainly into 3 different subsystem to optimize - Fin Geometry, Fluid Convection, and Transportation Cost - as 3 individual subsystems and also as an overall optimization problem through the Analytical Target Cascading method.

## My Specific Contributions
The specific subsystem I was in charge of was the transportation cost and I spearheaded the overall optimization via ATC.

## Design Results
Results for treating each subsystem as seperate optimization problems:
Length of the fins: 0.008 m
Diameter of the pipes: 0.006 m
Heat Transfer: 251 W
Profit: $6.3M

Results for treating the system as one optimization problem via ATC:
Length of the fins: 0.005 m
Diameter of the pipes: 0.004 m
Heat Transfer: 250 W
Profit: $13.5M

## Brief Discussion on Analytical Target Cascading (ATC) Method Being Applied
ATC is not a true entire system level optimization method in the traditional sense. Instead, it treats the optimization as an interconnected tree, with the transportation subsystem sitting on top and the physics constraints of the fin lengths and pipe diameters feeding into that. Profit was assumed to be a very large number and the algorithm (implemented in Matlab) solved subsystem optimizations, compared responses versus targets, updated the targets, and then repeated until convergence (the script was actually just set up to run 5 times which would be representative of convergence). Please refer to the file titled "AnalyticalTargetCascade.pdf" for a more mathematically accuracte definition.

## Detailed Report
Please refer to the file titled "DesignOptimizationFinalReport.pdf" for a complete breakdown of the findings of this project.

