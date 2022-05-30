# Application-of-PSO-and-for-finding-optimized-solutions-in-Aluminum-Turning-Process
The project is about writing and applying the nature-inspired algorithm to solve the problems for finding optimized solutions for a given problem
#individual best PSO
clear all;
clc;
ff = 'vijmanpso'; % Objective Function
popsize = 1; % Size of the swarm
npar = 4; % Dimension of the problem
maxit = 100; % Maximum number of iterations
c1 = 1.6; % cognitive parameter
c2 = 2; % social parameter
C=1; % constriction factor
% Initializing swarm and velocities
lwb1=700;
lwb2=0.1;
lwb3=0.1;
lwb4=5;
upb1=1200;
upb2=0.16;
upb3=0.3;
upb4=15;
par1=lwb1;
par2=lwb2;
par3=lwb3;
par4=lwb4;
%%par2=rand(popsize,1)*0.5;
%par3=rand(popsize,1)*0.2;
par=[par1,par2,par3,par4];
cost=feval(ff,par);
iter=0;
[cost]
while iter<=maxit
iter=iter+1;
par1=rand(popsize,1)*(upb1-lwb1)+lwb1;
par2=rand(popsize,1)*(upb2-lwb2)+lwb2;
par3=rand(popsize,1)*(upb3-lwb3)+lwb3;
par4=rand(popsize,1)*(upb4-lwb4)+lwb4;
par=[par1,par2,par3,par4];
currcost=feval(ff,par);
all=[currcost(1) currcost(2) currcost(3)];
[all]
[par1 par2 par3 par4]
if currcost(1)<=cost(1)
cost(1)=currcost(1);
indx1=iter;
bestpar1=par;
meanc=mean(cost);
end
if currcost(2)<=cost(2)
cost(2)=currcost(2);
indx2=iter;
bestpar2=par;
end
if currcost(3)<=cost(3)
cost(3)=currcost(3);
indx3=iter;
bestpar3=par;
end
end
[indx1 bestpar1 cost(1)]
[indx2 bestpar2 cost(2)]
[indx3 bestpar3 cost(3)]
figure(24)
iters=0:length(meanc)-1;
plot(iters,meanc,iters,currcost);
xlabel('generation');ylabel('cost');
text(0,meanc,'best');


















#Final PSO
clear all;
clc;
ff = 'vijmanpso'; % Objective Function
popsize = 1; % Size of the swarm
npar = 4; % Dimension of the problem
maxit = 100; % Maximum number of iterations
c1 = 1.6; % cognitive parameter
c2 = 2; % social parameter
C=1; % constriction factor
% Initializing swarm and velocities
lwb1=700;
lwb2=0.1;
lwb3=0.1;
lwb4=5;
upb1=1200;
upb2=0.16;
upb3=0.3;
upb4=15;
par1=lwb1;
par2=lwb2;
par3=lwb3;
par4=lwb4;
%%par2=rand(popsize,1)*0.5;
%par3=rand(popsize,1)*0.2;
par=[par1,par2,par3,par4];
cost=feval(ff,par);
iter=0;
[cost]
while iter<=maxit
iter=iter+1;
par1=rand(popsize,1)*(upb1-lwb1)+lwb1;
par2=rand(popsize,1)*(upb2-lwb2)+lwb2;
par3=rand(popsize,1)*(upb3-lwb3)+lwb3;
par4=rand(popsize,1)*(upb4-lwb4)+lwb4;
par=[par1,par2,par3,par4];
currcost=feval(ff,par);
all=[currcost(1) currcost(2) currcost(3)];
[all]
[par1 par2 par3 par4]
if currcost<=cost
cost=currcost;
indx=iter
bestpar=par
meanc=mean(cost)
end
end
[indx bestpar cost]
figure(24)
iters=0:length(meanc)-1;
plot(iters,meanc,iters,currcost);
xlabel('generation');ylabel('cost');
text(0,meanc,'best');





















# Initial PSO
clear all;
clc;
ff = 'vijmanpso'; % Objective Function
% Initializing variables
popsize = 1; % Size of the swarm
npar = 4; % Dimension of the problem
maxit = 100; % Maximum number of iterations
c1 = 1.6; % cognitive parameter
c2 = 2; % social parameter
C=1; % constriction factor
% Initializing swarm and velocities
lwb1=700;
lwb2=0.1;
lwb3=0.1;
lwb4=5;
upb1=1200;
upb2=0.16;
upb3=0.3;
upb4=15;
par1=rand(popsize,1)*(upb1-lwb1)+lwb1;
par2=rand(popsize,1)*(upb2-lwb2)+lwb2;
par3=rand(popsize,1)*(upb3-lwb3)+lwb3;
par4=rand(popsize,1)*(upb4-lwb4)+lwb4;
%%par2=rand(popsize,1)*0.5;
%par3=rand(popsize,1)*0.2;
par=[par1,par2,par3,par4]; % random population of
% continuous values
vel = rand(popsize,npar); % random velocities
% Evaluate initial population
[par1 par2 par3 par4]
cost=feval(ff,par); % calculates population cost using
[cost]% ff
minc(1)=min(cost); % min cost
meanc(1)=mean(cost); % mean cost
globalmin(1)=minc(1); % initialize global minimum
% Initialize local minimum for each particle
localpar = par; % location of local minima
localcost = cost; % cost of local minima
% Finding best particle in initial population
[globalcost,indx] = min(cost);
globalpar=par(indx,:);
iter = 0; % counter
while iter < maxit
iter = iter + 1;
% update velocity = vel
w=(maxit-iter)/maxit; %inertia weiindxht
r1 = rand(popsize,npar); % random numbers
r2 = rand(popsize,npar); % random numbers
vel = C*(w*vel + c1 *r1.*(localpar-par)+c2*r2.*(ones(popsize,1)*globalpar-par));
% update particle positions
par = par + vel; % updates particle position
overlimit=par<=1;
underlimit=par>=0;
par=par.*overlimit+not(overlimit);
par=par.*underlimit;
% Evaluate the new swarm
cost = [feval(ff,par)]; % evaluates cost of swarm
% Updating the best local position for each particle
% Updating index g
[temp, t] = min(localcost);
if temp<globalcost
globalpar=par(t,:); indx=t; globalcost=temp;
end
[iter globalcost]
[cost]% print output each
% iteration
minc(iter+1)=min(cost); % min for this
% iteration
globalmin(iter+1)=globalcost; % best min so far
meanc(iter+1)=mean(cost); % avg. cost for
% this iteration
end% while
figure(24)
iters=0:length(minc)-1;
plot(iters,minc,iters,meanc,iters,globalmin);
xlabel('generation');ylabel('cost');
text(0,minc(1),'best');text(1,minc(2),'population average')
