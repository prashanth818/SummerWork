#const numRooms = 2.
#const numDoors = 1.
#const numAreas = 5.
#const numFloors = 1.
#const numBuildings = 1.
#const numObjects = 0.
#const numPersons = 1.
#const numRobots = 0.
#const n = 3.
#const numTables = 0.

#const numSurfaces = 3.
#const numBlocks = 2. 
#const numSteps = 16.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 sorts
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#step = 0..numSteps.

%% Location hierachy
#area = [a][0..numAreas].
#room = {r1}.
#door = {d1}.
#floor = [f][0..numFloors].
#building = [b][0..numBuildings].

%% Object definitions
#person = [p][0..numPersons].
#robot = [rob][0..numRobots].
#table = [tab][0..numTables].
#blocks = [block][0..numBlocks].
#object =  #blocks + #table.

%% Object Properties
#size = {small, medium, large}.
#colour = {red, green, blue, purple, yellow, black}.
#shape = {cuboid, prism, cube}.
#surface = [s][0..numSurfaces].

%% World hierachy
#class = {entity_class, real_class, abstract_class, animate_class, controllable_class, inanimate_class, person_class, robot_class, 
  object_class, location_class, building_class, floor_class, room_class, area_class, door_class}.

#controllable = #robot.
#animate = #person + #controllable.
#inanimate = #object +  #table.
#location = #building + #floor+ #room + #door + #area + #surface.
#real = #animate + #inanimate.
#abstract = #location.
#entity = #real + #abstract.

%% States 
#state_door = {open, closed}.
%% #state_table  = {occupied, available}.
%% #state_dish = {empty, pasta, noodles}.
%% #state_person = {attended, unattended} + #table.
#state =  #state_door.

%% Actions
#exogenous_action = {test}.

#agent_action = travel(#robot, #location) + pick_up(#robot, #object) 
                + put_down(#robot, #object, #surface)
                 + open_door(#robot, #door).

#action = #exogenous_action + #agent_action.

%% Fluents
#inertial_fluent = has_location(#real, #location) + has_object(#animate, #inanimate)
                   + is_open(#door)  + on(#real, #surface) + has_state(#real, #state).

#defined_fluent = can_move_to(#animate, #location) + is_above(#real, #real).

#fluent = #inertial_fluent + #defined_fluent.

%% Used for Explanation generation - obs
#boolean = {true, false}.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 predicates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Membership Predicates
is_subclass(#class, #class).
siblings(#class, #class).
is_a(#entity, #class).


%% Location Predicates
belongs_to(#location, #location). %%ownership
is_connected(#location, #location). %%logical

%% Planning Predicates
success().
goal(#step).
something_happened(#step).

%% Object Predicates
has_size(#real, #size).
has_colour(#real, #colour).
has_shape(#real, #shape).
has_surface(#real, #surface).


%% Other Predicates
holds(#fluent, #step).
occurs(#action, #step).
obs(#fluent, #boolean, #step).
hpd(#action, #step).
expl(#exogenous_action, #step).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Membership Rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

is_subclass(real_class, entity_class).
is_subclass(abstract_class, entity_class).

is_subclass(animate_class, real_class).
is_subclass(person_class, animate_class).
is_subclass(controllable_class, animate_class).
is_subclass(robot_class, controllable_class).

is_subclass(inanimate_class, real_class).
is_subclass(object_class, inanimate_class).

%% hierachies of locations
is_subclass(location_class,  abstract_class).
is_subclass(building_class, location_class).
is_subclass(floor_class, building_class).
is_subclass(door_class, floor_class).
is_subclass(room_class, floor_class).
is_subclass(area_class, room_class).

is_a(A1, area_class) :- #area(A1).
is_a(R1, room_class) :- #room(R1).
is_a(D1, door_class) :- #door(D1).
is_a(F1, floor_class) :- #floor(F1).
is_a(B1, building_class) :- #building(B1).


%% subclass relation rules
-is_subclass(C1, C2) :- not is_subclass(C1, C2).


%% sibling rules
siblings(C1, C2) :- is_subclass(C1, C3), is_subclass(C2, C3), C1!=C2, C1!=C3, C2!=C3.
                    -siblings(C1, C2) :- not siblings(C1, C2).
  
%% location membership rules
belongs_to(L1, L3) :- belongs_to(L1, L2), belongs_to(L2, L3), is_subclass(C2, C3), 
                      is_a(L2, C2), is_a(L3,C3), L1!=L2, L1!=L3, L3!=L2.
-belongs_to(L1, L2) :- not belongs_to(L1, L2).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Defaults
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fluent and Action Rules
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CWA for Defined Fluents

-holds(F,I) :- #defined_fluent(F),
               not holds(F,I).

%% General Inertia Axiom

holds(F,I+1) :- #inertial_fluent(F),
                holds(F,I),
                not -holds(F,I+1).

-holds(F,I+1) :- #inertial_fluent(F),
                 -holds(F,I),
                 not holds(F,I+1).
                 
%% CWA for Actions
-occurs(AC,I) :- not occurs(AC,I).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Causal Laws for agent actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %% robot can travel to a location
holds(has_location(R,L2),I+1) :- occurs(travel(R, L2),I), #robot(R). 

%% If robot moves to a location L2 of a type, then it is no longer in any location types
%% that are "lower" than L2 in the locations hierachy
-holds(has_location(R,L3),I+1) :- occurs(travel(R, L2),I), holds(has_location(R,L1),I),
                                  L1!=L2, is_a(L1, C1), is_a(L2, C2), siblings(C1,C2),
                                  holds(has_location(R,L3),I), belongs_to(L3,L1).

-holds(has_location(O,L3),I+1) :- occurs(travel(R, L2),I), holds(has_location(R,L1),I),
                                  L1!=L2, is_a(L1, C1), is_a(L2, C2), siblings(C1,C2),
                                  holds(has_location(R,L3),I), belongs_to(L3,L1),
                                  holds(has_object(R,O),I).


%% %% robot can pick up an object
holds(has_object(R, O),I+1) :- occurs(pick_up(R, O), I), #robot(R).


%% If you pick up an object then it is no longer where it was
-holds(on(O1, S2), I+1) :- occurs(pick_up(R, O1), I), holds(on(O1, S2), I), 
                              #robot(R), has_surface(O2, S2).

%% %% robot can put down an object
-holds(has_object(R, O),I+1) :- occurs(put_down(R,  O, L), I), #robot(R).

%% if a robot puts down an object at a location and there is already an object O2 there, O1 is on O2.
 holds(on(O1,S2), I+1) :- occurs(put_down(R,O1, S2),I), holds(has_location(R, L), I), 
                          holds(has_location(O2, L),I), has_surface(O2, S2), #robot(R).


%% %% robot can open a door
holds(is_open(D),I+1) :- occurs(open_door(R, D), I).




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% State constraints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% if an entity is in a specific location, it is also in the locations above it in the hierachy
holds(has_location(Th, L2),I) :- holds(has_location(Th, L1), I), belongs_to(L1, L2), L1!=L2.

%% an entity can only be in one level of location at a tim
-holds(has_location(Th, L1), I) :- holds(has_location(Th, L2), I), siblings(C1, C2),
                                   L1!=L2,  is_a(L1, C1), is_a(L2, C2).
-holds(has_location(Th, L1), I) :- holds(has_location(Th, L2), I), L1!=L2,  
                                   is_a(L1, C1), is_a(L2, C2), C1=C2.

%% if a animate being has an object, the objects location is that of the being  
holds(has_location(O,L),I) :- holds(has_location(A,L), I), holds(has_object(A,O), I).


%%If areas A1 and A2 are in the same room than they are connected
is_connected(A1, A2) :- belongs_to(A1, R), belongs_to(A2, R), #area(A1), #area(A2),
                        #room(R), A1!=A2.

%% If rooms are connected to the same door, then they are connected
is_connected(R1, R2) :- is_connected(D1, R1), is_connected(D1, R2), #door(D1),
                         #room(R1), #room(R2), R1!=R2.

%% Locations are not connected unless you explicitly say they are
-is_connected(L1, L2) :- not is_connected(L1, L2).

%% Location are commutative
is_connected(L1, L2) :- is_connected(L2, L1), L1!=L2.


%% If areas are connected, then you can move between them
holds(can_move_to(R, A2), I) :- is_connected(A1, A2), A1!=A2, #area(A1), #area(A2),
                                holds(has_location(R,A1),I).

%% If rooms are connected and the door between them is open, you can move between them
holds(can_move_to(R, R2),I) :- is_connected(R1, R2), holds(is_open(D), I), R1!=R2,
                               #room(R1), #room(R2), holds(has_location(R,R1),I).

%% If doors belong to the same room, you can move between them
holds(can_move_to(R, D2), I) :- is_connected(D1, R1), is_connected(D2, R1),
                                #door(D2), #door(D1), #room(R1), D1!=D2, 
                                holds(has_location(R,D1),I), holds(is_open(D1), I).

%% If an area and a door belong to the same room, you can travel to the door
holds(can_move_to(R, D1),I) :- belongs_to(A1, R1), is_connected(D1, R1), #door(D1),
                               #room(R1), #area(A1), holds(has_location(R, A1),I).

%% If an room and a door belong to the same room, you can travel to the door
holds(can_move_to(R, D1),I) :-  is_connected(D1, R1), #door(D1), #room(R1),
                                holds(has_location(R, R1),I).

%% If an area and a door belong to the same room, you can travel from a door
holds(can_move_to(R, A1),I) :- belongs_to(A1, R1), is_connected(D1, R1), #door(D1),
                               #room(R1), #area(A1), holds(has_location(R, D1),I),
                               holds(is_open(D1), I).

%% If a room and a door belong to the same room, you can travel from a door
holds(can_move_to(R, R1),I) :- is_connected(D1, R1), #door(D1), #room(R1),
                               holds(has_location(R, D1),I), holds(is_open(D1), I) .

%% If you are in a room R2 and A1 is directly in R2, you can travel to A1 directly.
holds(can_move_to(R, A1), I) :- belongs_to(A1, R2), holds(has_location(R, R2), I),
                                 #room(R2), #area(A1).


%% Real things can only have 1 state at a time - inertial_fluent
-holds(has_state(R, S1), I) :- holds(has_state(R, S2), I), S1!=S2.

%% If an something O1 is on something O2, then O1 has the location of O2
holds(has_location(O1, L1), I) :- holds(on(O1, S2),I), 
                                holds(has_location(O2, L1), I),
                               has_surface(O2, S2), O1!=O2.

%% If an something O1 is on something O2, and O2 is on something O3, then O1 is above O3.
holds(is_above(O1, O3),I) :- holds(on(O1, S2),I), holds(on(O2, O3), I),
                            has_surface(O2, S2), O1!=O2, O2!=O3, O1!=O3.

%% If O1 is above O2 and O2 is above O3 then O1 is above O3.
holds(is_above(O1,O3), I) :-  holds(is_above(O1, O2), I), 
                              holds(is_above(O2, O3),I), O1!=O2, O2!=O3, O1!=O3. 

%% an object cannot be on itself
-holds(on(O1, S2), I) :- has_surface(O2, S2), O1 = O2.

%% an object cannot be above itself
-holds(is_above(O1, O2), I) :- O1 = O2.

%% only one object can have a surface
-has_surface(O2, S1) :- has_surface(O1, S1), O1!=O2.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Executability constraints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% do not plan to travel to where you are
 -occurs(travel(R,  L), I) :- holds(has_location(R, L), I), #area(L).

%% robot cannot pick up an object if it is not at the objects location
-occurs(pick_up(R, O),I) :- holds(has_location(R, L1), I), holds(has_location(O, L2), I),
                            L1!=L2,  #area(L1), #area(L2).

%% robot cannot pick up an object if they have it 
-occurs(pick_up(R,O), I) :-  holds(has_object(R,O), I), #robot(R).

%% cannot put down an object that is not being held
-occurs(put_down(R,  O, L),I) :- not holds(has_object(R, O),I).

%% Cannot open door from a distance
-occurs(open_door(R, D),I) :- -holds(has_location(R, D),I).

%%Don't plan to open a door when you don't where the agent is
-occurs(open_door(R, D),I) :- not holds(has_location(R, D),I).

%% Do not try to open a door if it is open
-occurs(open_door(R, D),I) :- holds(is_open(D),I).

%% We can't travel between areas that you can't move between
-occurs(travel(R, L2),I) :- -holds(can_move_to(R, L2),I),holds(has_location(R, L1),I), L1!=L2.

%% cannot put down an object at location L if we aren't there
 -occurs(put_down(R, O, S), I) :- has_surface(O2, S), holds(has_location(O2, L),I),
                                -holds(has_location(R, L), I), #robot(R).

%% robot can only hold one object at a time
-occurs(pick_up(R, O1),I) :-  holds(has_object(R,O2), I), #robot(R), O1!=O2.

%robot cannot pick up an object if there is something on top of it
-occurs(pick_up(R,O1), I) :-  holds(on(O2, S1), I), has_surface(O1, S1).

%robot cannot put down an object onto O2 if there is something already on O2
-occurs(put_down(R, O1, S2), I) :- has_surface(O2, S2), holds(on(O3, S2), I), O1!=O2, O2!=O3, O3!=O1,
                                  #blocks(O1), #blocks(O2), #blocks(O3).

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
has_surface(tab0, s3).
has_size(block0, medium).
has_colour(block0, red).
has_shape(block0, cube).
has_surface(block0, s0).
holds(on(block0, s2), 0).
has_size(block1, medium).
has_colour(block1, blue).
has_shape(block1, prism).
has_surface(block1, s1).
holds(on(block1, s0), 0).
has_size(block2, medium).
has_colour(block2, green).
has_shape(block2, cuboid).
has_surface(block2, s2).
holds(on(block2, s3), 0).
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
goal(I) :-  holds(on(block0, s3), I), holds(on(block1, s2), I), holds(on(block2, s3), I).