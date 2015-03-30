# AA 203 - Motion Planning Algorithms

Matlab code for a Spring 2014 AA 203 (Introduction to Optimal Control Theory) lecture on motion planning. Perhaps a little late to put this code up -- has it really been a whole year? -- and I have no idea what happened to the slides, but I hope this can help illustrate some basic graph expansion concepts for those new to the field. The following sampling-based geometric planning algorithms are included:
* [RRT](http://msl.cs.uiuc.edu/rrt/)
* [RRT*](http://arxiv.org/abs/1105.1186)
* [PRM*](http://arxiv.org/abs/1105.1186)
* [FMT*](http://arxiv.org/abs/1306.3532)

Note that this code exists only to illustrate how the local connection graphs grow. Few inferences, if any, about algorithm run time or performance should be made. IIRC an important implementation detail missing here for RRT and RRT* is a goal biasing parameter for state sampling.

This sort of thing would probably live a better life as a Web 2.0 JavaScript/HTML5 Canvas/buzzword medley, so I may port it over at some point - stay tuned (or not: it may take me another year).

## Usage
Check ```motion_planning.m``` for some basic usage examples. Notably, the last argument of each planner function (```RRT```, ```RRTstar```, ```PRMstar```, ```FMTstar```) can be set to 0, 1, or 2 depending on the level of step illustration desired.