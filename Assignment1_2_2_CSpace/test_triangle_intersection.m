% Tests triangle_intersection file

testTriangleIntersection = @(P1,P2) triangle_intersection(P1, P2);

testEquals('Test 1.a','Triangles Intersecting, A overlaps B',1,...    
    testTriangleIntersection([2,3;3,1;4,2],[2,2;5,2;5,4]));

testEquals('Test 1.b','Triangles intersecting, A & B flipped',1,...    
    testTriangleIntersection([2,3;3,1;4,2],[2,2;5,2;5,4]));

testEquals('Test 2.0','triangles not intersecting, neither triangle are near each other',0,...    
testTriangleIntersection([2,2;2,4;3,4],[3,2;5,2;5,4]));

testEquals('Test 3.a','triangles inclusion, A should include B',1,...    
    testTriangleIntersection([3,1;4,1;4,2],[0,0;5,0;5,5]));

testEquals('Test 3.b','triangles inclusion, flipped - A should not be in B (A is bigger)',0,...    
testTriangleIntersection([0,0;5,0;5,5],[3,1;4,1;4,2]));

testEquals('Test 4.a','triangles just touching along 1 edge',1,...    
    testTriangleIntersection([5,0;5,5;0,0],[0,0;0,5;5,5]));

testEquals('Test 4.b','triangles just touching at a single point', 1, ...    
testTriangleIntersection([5,0;5,1;0,0],[5,1;5,5;0,0]));

testEquals('Test 5.a','2 scalene triangles intersecting', 1, ...    
testTriangleIntersection([1,1;10,3;5,6.2],[2,2.1;3,8.8;7,4]));

testEquals('Test 5.b','2 scalene triangles intersecting', 1, ...    
testTriangleIntersection([1,1.1234;10.1521353,3.324132;5.234124,6.2134235],[2.2135,2.2351251;3.152351,8.2135253;7.12352,4.12552]));

testEquals('Test 5.c','2 large triangles intersecting', 1, ...    
testTriangleIntersection([1,1;1000000,10000000;400000,4000000],[3.41234213423,3.123412342;725345,453765765453;2354345,23533234]));

testEquals('Test 6.a','Degenerate Triangle - one is single point',1,...    
    testTriangleIntersection([0,0;5,0;5,5],[0,0;0,0;0,0]));

testEquals('Test 6.b','Degenerate Triangle - one is single point - flipped',1,...    
testTriangleIntersection([0,0;0,0;0,0],[0,0;5,0;5,5]));

testEquals('Test 6.c','Degenerate Triangles - both single point',1,...    
    testTriangleIntersection([1,1;1,1;1,1],[1,1;1,1;1,1]));

testEquals('Test 6.d','Degenerate Triangle - a line',1,...    
testTriangleIntersection([0,0;1,0;0,0],[0,0;0,1;0,0]));

testEquals('Test 7.a','Super skinny triangles overlapping',1,...    
testTriangleIntersection([0,0;1,0;0,0.02],[0,0;0,1;0,0.02]));