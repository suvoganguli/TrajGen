# GIG-MPC
Updatd - 12 May, 2018

*Under laplacian the following branches are stored in the repo. The master can possibily be neglected*  

### no_popup
- Avoid known obstacles
- Avoid obstacles in the form of rectangle with safe zone only on the left and right sides.

### popup
- Avoid unknown obstacles
- Avoid obstacles in the form of rectangle with safe zone only on the left and right sides.

### tradeoffs
- Performed tradeoff studies for mid-term review for gig - should have the tradeoff charts.

_______________________________________________________________________________________________

*The following approach does not use the Laplacian planner as an initializer

### circular_safezone: 
- Works well - creates circular safezones around obstacles. 
- May lead to conversatism for rectangular shapes.
- Gets slow with too many obstacles. No. of constraints ~ N * o



