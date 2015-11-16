%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Environment setup
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 belongs_to(a0, r1).
 belongs_to(a1, r1).
 belongs_to(a2, r1).

 belongs_to(r1,f1).
 belongs_to(f1,b1).

 is_connected(d1, r1).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Testing / Scenarios
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%  Initial location of robot 
holds(has_location(rob0, a0), 0).

holds(has_location(tab0, a0), 0).

%% Initial scene description
