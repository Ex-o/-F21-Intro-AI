:- [library(aggregate)].

% Start method for BFS
start_bfs([StartX, StartY], [GoalX, GoalY], L) :-
    retractall(visited(X, Y)),
    retractall(parent((A, B), (C, D))),
    assert(visited(StartX, StartY)),
    assert(covid(-1, -1)),
    assert(mask([-1, -1])),
    bfs([[StartX, StartY]], [StartX, StartY], [GoalX, GoalY]),
    getPath([GoalX, GoalY], [StartX, StartY], L),
    nb_setval(minpath, L).

% Implementation of BFS 

bfs([ [GoalX, GoalY] | _ ], [StartX, StartY], [GoalX, GoalY]).

bfs([[X, Y] | Rest], [StartX, StartY], [GoalX, GoalY]) :-
    setof([XN, YN], moore(X, Y, XN, YN), Pnts),
    getPath([X, Y], [StartX, StartY], PathSoFar),
    %Generating all possible good moves
    findall([XX, YY], (
                        member([XX, YY], Pnts), 
                        ((\+ covid(XX, YY) ; (include(mask, PathSoFar, M), member(_, M))), % making sure I have a mask or cell isn't covid
                          \+ visited(XX, YY)) % Making sure I haven't visited this cell before
                      ),
            GoodPnts),
    findall([XX, YY], (
                        member([XX, YY], GoodPnts), assert(visited(XX, YY)), assert(parent((XX, YY), (X, Y)))), % Updating shortest path to current cell
           _),
    append(Rest, GoodPnts, Queue),
    bfs(Queue, [StartX, StartY], [GoalX, GoalY]).


% Going through masks to ensure a path to home is found.

goThroughMask([StartX, StartY], [MaskX, MaskY], [GoalX, GoalY], L) :-
    assert(parent((StartX, StartY), (StartX, StartY))),
    start_bfs([StartX, StartY], [MaskX, MaskY], L1),
    start_bfs([MaskX, MaskY], [GoalX, GoalY], [[MaskX, MaskY] |L2]),
    append(L1, L2, L),

    nb_setval(minpath, L). % Updating minimum path.

% Helper method to append starting node to path.

getPath([X, Y], [RootX, RootY], L) :-
  getParent([X, Y], [RootX, RootY], L2),
  append(L2, [[X, Y]], L).

%Recursively get parent cells of [X, Y] used in path regeneration of BFS.

getParent([RootX, RootY], [RootX, RootY], []).
getParent([X, Y], [RootX, RootY], L) :-
    (   findall([XX, YY], parent((X, Y), (XX, YY)), [ [PX, PY] | _ ])
         -> (getParent([PX, PY], [RootX, RootY], L2), append(L2, [ [PX, PY] ], L))).

% generate moore neighbourhood
moore1d(X, X2) :-
    SX is X - 1,
    BX is X + 1,
    SX1 is max(SX, 0),
    BX1 is min(BX, 8),
    between(SX1, BX1, X2).


moore(X, Y, X2, Y2) :-
    moore1d(X, X2),
    moore1d(Y, Y2),
    [X, Y] \== [X2, Y2].

% Getting length of a list.
list_length([]     , 0 ).
list_length([_|Xs] , L ) :- list_length(Xs,N) , L is N+1 .

travel([ActorX, ActorY], [HomeX, HomeY], Path, Length) :-
    travel_rec([HomeX, HomeY], [[ActorX, ActorY]], Path, Length).

% found home
travel_rec([HomeX, HomeY], [[HomeX, HomeY]|Visited], [[HomeX, HomeY]|Visited], 0) :-
    list_length(Visited, X),
    nb_getval(cur_min, CUR),
    Len is X+1,
    write('Found a path with length: '), write(Len), nl,
    (CUR > X -> (nb_setval(cur_min, X))).

% Wrapper to help with guided search instead of blind search.
process([X, Y],[X2,Y2], [HomeX, HomeY], Length, Path, Visited) :-
    list_length(Visited, LEN),
    nb_getval(cur_min, C),
    LEN < C - 1,
    not(member([X2, Y2], [X, Y]|Visited)),
    travel_rec([HomeX, HomeY], [[X2, Y2], [X, Y]|Visited], Path, Length).

% Handling the case where BFS fails to generate a path [Discussed in details in report highlights section.]
tryBFS([ActorX, ActorY], [Mask1X, Mask1Y], [Mask2X, Mask2Y], [HomeX, HomeY]) :-
    nb_getval(cur_min, CurrentMin),
    (
      start_bfs([ActorX, ActorY], [HomeX, HomeY], L1) 
        -> (true) ; 
        ((
          (goThroughMask([ActorX, ActorY], [Mask1X, Mask1Y], [HomeX, HomeY], L2)) 
            -> (list_length(L2, Len2), (CurrentMin > Len2 -> (nb_setval(cur_min, Len2), nb_setval(minpath, L2)))) ; 
            ((goThroughMask([ActorX, ActorY], [Mask2X, Mask2Y], [HomeX, HomeY], L3) 
              -> (list_length(L3, Len3), (CurrentMin > Len2 -> (nb_setval(cur_min, Len3), nb_setval(minpath, L3)))) ; (fail)))
        ))
    ).
    


entry(K, V) :- V is K.
getPath([[0|L]], L).

% Getting minimum direct distance between two cells (used for pruning)
getDistance(X1, Y1, X2, Y2, D) :-
    D1 is abs(X1- X2),
    D2 is abs(Y1- Y2),
    D is max(D1, D2).

% Backtracking implementation 
travel_rec([HomeX, HomeY], [[X, Y]|Visited], Path, Length) :-
    not(member([HomeX, HomeY], [[X, Y]|Visited])),
    setof([XN, YN], moore(X, Y, XN, YN),Pnts), % Getting all possible moves.
    nb_getval(cur_min, C),
    findall([XX, YY], (
              member([XX, YY], Pnts), 
              not(member([XX, YY], Visited)), % Excluding already visisted cells.
              getDistance(HomeX, HomeY, XX, YY, D), D < C - 1, % Pruning cells where the distance is guaranteed to not get better than best-known path
              ((\+ covid(XX, YY) ; (include(mask, [[X,Y]|Visited], M), member(_, M))))), % Making sure that the cell is safe.
    GoodPnts),
    findall([V, [XX, YY]], (
              member([XX, YY], GoodPnts), 
              entry(abs(HomeX - XX) + abs(HomeY - YY), V)), % Calculating direct distance to home from possible moves.
    PntsDist),
    sort(PntsDist, SortedPnts), % Sorting by minimum distance (guided search) -> ensures we find some path with small length ASAP and thus more pruning.
    findall([Length, Path], (member([_,[X2, Y2]], SortedPnts), process([X, Y], [X2, Y2],[HomeX, HomeY], Length, Path, Visited)), L),  member(_, L), findall(Q, member([Q, _], L), LL), findall([P, L2], member([L2, P], L), LLL),
     min2(LL, ML),
     Length is ML + 1,
     getPath(L, [K|_]),
     nb_setval(minpath, K), % Update minimum path.
     write(K),nl,nl,
     min_path(LLL, [Path, Length]).


% generating minimum from a list.
min_path([F|Rest], [Path, Length]) :- min(Rest, F, [Path, Length]).

min([], M, M).
min([[P, L]|Rest], [_, CurrentMinLen], Min) :- L < CurrentMinLen, !, min(Rest, [P, L], Min).
min([_|Rest], CurrentMin, Min) :- min(Rest, CurrentMin, Min).
min2(L, M) :- aggregate(min(E), member(E, L), M).

%function to graphically represent the map using ascii characters.
drawMap(Size) :-
       forall(between(0,Size, I), 
            ((forall(between(0, Size, J),(getchar(I, J, C), write(' '), write(C), write(' |')))), nl,
                forall(between(0,Size, _), write('___ ')), nl)).

% debug code to read map from file
debug(File) :-
    statistics(runtime,[Start|_]),
    nb_setval(cur_min, 100),
    nb_setval(minpath, []),
    open(File, read, Stream),
    read(Stream, Deminesion),
    Size is Deminesion - 1,
    read(Stream, [ActorX, ActorY]),
    read(Stream, [HomeX, HomeY]),
    read(Stream, [Covid1X, Covid1Y]),
    read(Stream, [Covid2X, Covid2Y]),
    read(Stream, [Mask1X, Mask1Y]),
    read(Stream, [Mask2X, Mask2Y]),
    assert(actor(ActorX, ActorY)),
    assert(home(HomeX, HomeY)),
    assert(mask([Mask2X, Mask2Y])),
    assert(covid(Covid1X, Covid1Y)),
    assert(covid(Covid2X, Covid2Y)),
    forall(moore(Covid1X, Covid1Y, X2, Y2), assert(covid(X2, Y2))),
    forall(moore(Covid2X, Covid2Y, X2, Y2), assert(covid(X2, Y2))),
    assert(mask([Mask1X, Mask1Y])),
    % write(Algorithm),nl,
    close(Stream),
    drawMap(Size),
    tryBFS([ActorX, ActorY], [Mask1X, Mask1Y], [Mask2X, Mask2Y], [HomeX, HomeY]),
    nb_getval(minpath, L),
    write(L), nl,
    drawMap(Size).
    % start_bfs([ActorX, ActorY], [HomeX, HomeY]),
    % nb_getval(cur_min, M),
    % (   M =:= 100 -> (write('Could not solve the map :(')); nl),
    %  forall(between(0,8, I), ((forall(between(0, 8, J),(getchar(I, J, C), write(' '), write(C), write(' |')))), nl,forall(between(0,8, _), write('___ ')), nl)),
    % statistics(runtime,[Stop|_]),
    % Runtime is (Stop - Start)/1000,
    % write('Runtime = '), write(Runtime), write('s'), nl,
    % resetEnv.

% Random number generator
rng(S, E, X) :-
    random_between(S, E, X).

resetEnv :-
    retractall(covid(X, Y)), retractall(mask([X, Y])), retractall(home(X, Y)),
      retractall(actor(X, Y)).

start(N) :-
    statistics(runtime,[Start|_]), % Runtime calculation.
    nb_setval(cur_min, 100),
    nb_setval(minpath, []),
    ActorX is 8, ActorY is 0, % Actor Starting Cell Left-Bottom
    Size is N - 1,
    assert(covid(-1, -1)), assert(mask([-1, -1])), assert(home(-1, -1)), 
    assert(actor(ActorX, ActorY)),
    repeat, generateCovidAgent(Covid1X, Covid1Y, Size),
    repeat, generateCovidAgent(Covid2X, Covid2Y, Size),
    repeat, generateMask(Mask1X, Mask1Y, Size),
    repeat, generateMask(Mask2X, Mask2Y, Size),
    repeat, generateHome(HomeX, HomeY, Size),
    write('Initial Map: '), nl,
    drawMap(Size),
     ( not(travel([ActorX, ActorY], [HomeX, HomeY], _, _)) ; nl),
     nb_getval(cur_min, M),
     (   M =:= 100 -> ((write('Could not solve the map using backtracking!'), nl)); (write('Backtracking Solution:'), nl, drawMap(Size))),
     (tryBFS([ActorX, ActorY], [Mask1X, Mask1Y], [Mask2X, Mask2Y], [HomeX, HomeY]) -> (write('Breadth-first-search Solution: '), nl,nb_getval(minpath, L),
    write(L), nl, drawMap(Size));
      write('Breadth-first-search could not find a solution.'), nl),
     statistics(runtime,[Stop|_]),
     Runtime is (Stop - Start)/1000,
     write('Runtime = '), write(Runtime), write('s'), nl,
     resetEnv.

% Helper method used in drawing the map graphically.
getchar(X, Y, C) :-
    nb_getval(minpath, M),
    (actor(X, Y) -> C = 'S' ; 
      (   (home(X, Y)) -> (C = 'H') ; (member([X, Y], M) -> (((mask([X, Y]) -> (C = 'M') ; (C = '*') ))) ; ((covid(X, Y)) -> (C = 'C') ; ((mask([X, Y]) -> (C = 'M') ; (C = ' ') ))))  )
    ).

generateHome(HomeX, HomeY, N) :-
    rng(0, N, HomeX),
    rng(0, N, HomeY),
    ( (not(covid(HomeX, HomeY)), not(home(HomeX,HomeY)), not(mask([HomeX, HomeY])), not(actor(HomeX, HomeY))) -> (assert(home(HomeX, HomeY)), write('Home Generated at: '), write([HomeX, HomeY]), nl)).

generateCovidAgent(X, Y, N) :-
  rng(0, N, X),
  rng(0, N, Y),
  (   (not(covid(X, Y)), not(mask([X, Y])), not(actor(X, Y)), not(home(X, Y))) -> (assert(covid(X, Y)), write('Covid Generated at: '), write([X, Y]), nl, forall(moore(X, Y, X2, Y2), assert(covid(X2, Y2))))).

generateMask(X, Y, N) :-
  rng(0, N, X),
  rng(0, N, Y),
  (   (not(covid(X, Y)), not(mask([X, Y])), not(actor(X, Y)), not(home(X, Y))) -> (assert(mask([X, Y])), write('Mask Generated at: '), write([X, Y]), nl)).
