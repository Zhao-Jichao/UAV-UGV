% øÿ÷∆–≠“È

L = [3 1 1 1;
     1 3 1 1;
     1 1 3 1;
     1 1 1 3;];

distance = pa - p0;

v0 = va + K * (distance);

F1 = diff(U1, p1);

u1 = alpha * ( (p2-p1)+(p3-p1)+(p4-p1) ) + beta * ( (v2-v1)+(v2-v1)+(v2-v1) )...
   + gamma * (p0-p1)                     + ga   * (v0-v1)...
   + F1;


