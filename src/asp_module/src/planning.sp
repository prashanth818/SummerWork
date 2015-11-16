%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Planning Module
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

success :- goal(I).
  :- not success.

                              
%% Do not allow concurrent actions:
:- occurs(A1,I),
   occurs(A2,I),
   A1 != A2.

%% An action occurs at each step before the goal is achieved:
something_happened(I) :- occurs(A,I).

%% %% Action Generation
occurs(A,I) | -occurs(A,I) :- not goal(I), #agent_action(A).

:- #step(I),
   not something_happened(I),
   something_happened(I+1).
