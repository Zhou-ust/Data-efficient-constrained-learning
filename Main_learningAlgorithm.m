clc
clear all
close all


%% Choose a feasible solution to run!
% load('feasibleSolution0.mat');
load('feasibleSolution1.mat');
% load('feasibleSolution2.mat');
% load('feasibleSolution3.mat');

XX   = x_cl;
UU   = u_cl;
Vfun = computeCost(x_cl, u_cl);
IterationCost = Vfun(1);


Iterations = 10;
Traillength = 50;


x0 = x_cl(:,1);
u0 = u_cl(:,1);

%%
% % case1: Set gammm = 0;
% % case2: Do not add constraints;
% % case3: add constraints;

% xmax = [inf;inf;inf]; xmin = [-inf;-inf;-inf]; deltaxmax = [inf;inf;inf]; deltaxmin = [-inf;-inf;-inf];
xmax = [350;50;50]; xmin = [-50;-50;-50]; deltaxmax = [30;30;30]; deltaxmin = [-30;-30;-30];

% umax = inf; umin = -inf; deltaumax = inf; deltaumin = -inf;
% umax = 100; umin = -100; deltaumax = inf; deltaumin = -inf;
umax = 50; umin = -50; deltaumax = 30; deltaumin = -30;
% emax = inf; emin = -inf;
emax = 30; emin = -30;


UBounds = [umax umin deltaumax deltaumin];
XBounds = [umax umin deltaumax deltaumin];

[Uset, Xset] = constraints(u0, x0, UBounds, XBounds);




N = 3;
% Now start iteration loop
disp('Please be patient! ')
% ================================= The Main algorithm ======================================================
j = 1;
while (j <= Iterations)
    t = 1;
    x_CLt = x0;
    ut = u0;
    tol = 10^(-8);
    
    disp(['Now, it runs for iteration: ', num2str(j)])
    
    while (t <= Traillength)
        disp(['Time step: ', num2str(t)])
        
        [xPred, uPred ] = solveOptimalUncertainControlProblem(@plant, t, x_CLt(:,t), ut, N, Vfun, XX, UU, UBounds, XBounds);
        %   [xPred, uPred ] = solveOptimalControlProblem(@plant, t, x_t, u_t, N, Qfun, XX, UU, UBounds, XBounds)
        
        u_CLt(:,t) = double(uPred{1});
        x_CLt(:,t+1) = plant(t, x_CLt(:,t), u_CLt(:,t));
        
        ut = u_CLt(:,end);
        t = t + 1;
    end
    
    
    XX   = [XX, x_CLt];
    UU   = [UU, u_CLt];
    Vfun = [Vfun, computeCost(x_CLt, u_CLt)];
    
    totalCostVector = computeCost(x_CLt, u_CLt);
    IterationCost(j+1) = totalCostVector(1);
    
    % increase Iteration index and restart
    j = j + 1;
    if j <= Iterations
        clear x_CLt
        clear u_CLt
    end
    
end


% =======================================================================================

% Plot the Results
for i = 1:(Iterations+1)
    fprintf('Total cost at iteration %d:  %13.5f\n', [i, IterationCost(i)]);
end

a0 = x_cl(:,1);
Iter_index_X = 1;
for ix = size(x_cl,2): size(XX,2)
    if XX(:,ix) == a0
        % if mod(ix, Traillength) == 0
        ix;
        Iter_index_X = [Iter_index_X, ix];
    end
end
Iter_index_X;

% It depends on the iterations;
n = size(x_cl,1);
m = size(u_cl,1);

X0 = XX(1:n,1:size(x_cl,2));
X1 = XX(1:n,Iter_index_X(2):Iter_index_X(2+1)-1);
X2 = XX(1:n,Iter_index_X(3):Iter_index_X(3+1)-1);
X3 = XX(1:n,Iter_index_X(4):Iter_index_X(4+1)-1);
X4 = XX(1:n,Iter_index_X(5):Iter_index_X(5+1)-1);
X5 = XX(1:n,Iter_index_X(6):Iter_index_X(6+1)-1);
X6 = XX(1:n,Iter_index_X(7):Iter_index_X(7+1)-1);
X7 = XX(1:n,Iter_index_X(8):Iter_index_X(8+1)-1);
X8 = XX(1:n,Iter_index_X(9):Iter_index_X(9+1)-1);
X9 = XX(1:n,Iter_index_X(10):Iter_index_X(10+1)-1);
X10 = XX(1:n,Iter_index_X(11):end);

% figure;
RMSE = [sqrt(sum(X0(3,:).^2))/(Traillength+1) sqrt(sum(X1(3,:).^2))/(Traillength+1) sqrt(sum(X2(3,:).^2))/(Traillength+1) sqrt(sum(X3(3,:).^2))/(Traillength+1) ...
    sqrt(sum(X4(3,:).^2))/(Traillength+1) sqrt(sum(X5(3,:).^2))/(Traillength+1) sqrt(sum(X6(3,:).^2))/(Traillength+1) ...
    sqrt(sum(X7(3,:).^2))/(Traillength+1) sqrt(sum(X8(3,:).^2))/(Traillength+1) sqrt(sum(X9(3,:).^2))/(Traillength+1) ...
    sqrt(sum(X10(3,:).^2))/(Traillength+1)];

%%
T = 0:size(X1(1,:),2)-1;

figure(1);
h1 = axes;
plot3(1:size(X0(1,:),2),(0)*ones(1,size(X0(1,:),2)),X0(1,:),'b--','LineWidth',2); hold on
plot3(T,(1)*ones(1,size(X1(1,:),2)),X1(1,:),'k-.','LineWidth',2); hold on
plot3(T,(2)*ones(1,size(X2(1,:),2)),X2(1,:),'--g','LineWidth',2); hold on
plot3(T,(3)*ones(1,size(X3(1,:),2)),X3(1,:),':m','LineWidth',2); hold on
plot3(T,(4)*ones(1,size(X4(1,:),2)),X4(1,:),'-.g','LineWidth',2); hold on
plot3(T,(5)*ones(1,size(X5(1,:),2)),X5(1,:),':y','LineWidth',2); hold on
plot3(T,(6)*ones(1,size(X6(1,:),2)),X6(1,:),'-.c','LineWidth',2); hold on
plot3(T,(7)*ones(1,size(X7(1,:),2)),X7(1,:),'--g','LineWidth',2); hold on
plot3(T,(8)*ones(1,size(X8(1,:),2)),X8(1,:),'-.m','LineWidth',2); hold on
plot3(T,(9)*ones(1,size(X9(1,:),2)),X9(1,:),'-.y','LineWidth',2); hold on
plot3(T,(10)*ones(1,size(X10(1,:),2)),X10(1,:),'-r','LineWidth',2); hold on
set(h1, 'Ydir', 'reverse')
yticks([0:Iterations])
% zlim([-40 40])
grid on
xlim([0 50]);
box on
xlabel('t (s)','FontSize',12)
ylabel('Batch (k)','FontSize',12)
zlabel('$\bf{\delta y_k ( t )}$','FontSize',14,'interpreter','latex')
hold off


%%
U0 = UU(1:m,1:size(u_cl,2));
U1 = UU(1:m,Traillength + 1:2*Traillength);
U2 = UU(1:m,2*Traillength + 1:3*Traillength);
U3 = UU(1:m,3*Traillength + 1:4*Traillength);
U4 = UU(1:m,4*Traillength + 1:5*Traillength);
U5 = UU(1:m,5*Traillength + 1:6*Traillength);
U6 = UU(1:m,6*Traillength + 1:7*Traillength);
U7 = UU(1:m,7*Traillength + 1:8*Traillength);
U8 = UU(1:m,8*Traillength + 1:9*Traillength);
U9 = UU(1:m,9*Traillength + 1:10*Traillength);
U10 = UU(1:m,10*Traillength + 1:end);

figure(2);
h1 = axes;
plot3(1:size(U0(1,:),2),(0)*ones(1,size(U0(1,:),2)),U0(1,:),'b--','LineWidth',2); hold on
plot3(1:size(U1(1,:),2),(1)*ones(1,size(U1(1,:),2)),U1(1,:),'k-.','LineWidth',2); hold on
plot3(1:size(U2(1,:),2),(2)*ones(1,size(U2(1,:),2)),U2(1,:),'--g','LineWidth',2); hold on
plot3(1:size(U3(1,:),2),(3)*ones(1,size(U3(1,:),2)),U3(1,:),':m','LineWidth',2); hold on
plot3(1:size(U4(1,:),2),(4)*ones(1,size(U4(1,:),2)),U4(1,:),'-.g','LineWidth',2); hold on
plot3(1:size(U5(1,:),2),(5)*ones(1,size(U5(1,:),2)),U5(1,:),':y','LineWidth',2); hold on
plot3(1:size(U6(1,:),2),(6)*ones(1,size(U6(1,:),2)),U6(1,:),'-.c','LineWidth',2); hold on
plot3(1:size(U7(1,:),2),(7)*ones(1,size(U7(1,:),2)),U7(1,:),'--g','LineWidth',2); hold on
plot3(1:size(U8(1,:),2),(8)*ones(1,size(U8(1,:),2)),U8(1,:),'-.m','LineWidth',2); hold on
plot3(1:size(U9(1,:),2),(9)*ones(1,size(U9(1,:),2)),U9(1,:),'-.y','LineWidth',2); hold on
plot3(1:size(U10(1,:),2),(10)*ones(1,size(U10(1,:),2)),U10(1,:),'-r','LineWidth',2); hold on
set(h1, 'Ydir', 'reverse')
yticks([0:Iterations])
grid on
xlim([0 50]);
box on
xlabel('t (s)','FontSize',12)
ylabel('Batch (k)','FontSize',12)
zlabel('$\bf{\delta u_k ( t )}$','FontSize',14,'interpreter','latex')
hold off



%% plot the real states;
Originalstate = x0(1:2,1);
Xi0 = get_OriginalData(X0,Originalstate);
Xi1 = get_OriginalData(X1,Originalstate);
Xi2 = get_OriginalData(X2,Originalstate);
Xi3 = get_OriginalData(X3,Originalstate);
Xi4 = get_OriginalData(X4,Originalstate);
Xi5 = get_OriginalData(X5,Originalstate);
Xi6 = get_OriginalData(X6,Originalstate);
Xi7 = get_OriginalData(X7,Originalstate);
Xi8 = get_OriginalData(X8,Originalstate);
Xi9 = get_OriginalData(X9,Originalstate);
Xi10 = get_OriginalData(X10,Originalstate);

Y_d = 0;
for j = 1:size(X1(1,:),2)-1
    Y_d = [Y_d reference(j)];
end


figure(3);
plot(T,Y_d,'c',...
    0:size(Xi0(1,:),2)-1,Xi0(1,:),'b-.',...
    0:size(Xi1(1,:),2)-1,Xi1(1,:),'c:',...
    0:size(Xi2(1,:),2)-1,Xi2(1,:),'--b',...
    0:size(Xi3(1,:),2)-1,Xi3(1,:),'-.g',...
    0:size(Xi4(1,:),2)-1,Xi4(1,:),':k',...
    0:size(Xi5(1,:),2)-1,Xi5(1,:),'-.c',...
    0:size(Xi6(1,:),2)-1,Xi6(1,:),'--g',...
    0:size(Xi7(1,:),2)-1,Xi7(1,:),'-.m',...
    0:size(Xi8(1,:),2)-1,Xi8(1,:),':y',...
    0:size(Xi9(1,:),2)-1,Xi9(1,:),'-.b',...
    0:size(Xi10(1,:),2)-1,Xi10(1,:),'-r','linewidth',1.5);
xlim([0 50]);
xlabel('t ( s )','FontSize',12);
ylabel('$\bf{y_k ( t )}$','FontSize',14,'interpreter','latex')
legend ('Reference','Initial points','The 1st iteration','The 2nd iteration','The 3rd iteration',...
    'The 4th iteration','The 5th iteration','The 6th iteration','The 7th iteration',...
    'The 8th iteration','The 9th iteration','The 10th iteration',...
    'location','southeast');
hold off



%% plot the inputs
Ui0 = get_OriginalData(U0,0);
Ui1 = get_OriginalData(U1,0);
Ui2 = get_OriginalData(U2,0);
Ui3 = get_OriginalData(U3,0);
Ui4 = get_OriginalData(U4,0);
Ui5 = get_OriginalData(U5,0);
Ui6 = get_OriginalData(U6,0);
Ui7 = get_OriginalData(U7,0);
Ui8 = get_OriginalData(U8,0);
Ui9 = get_OriginalData(U9,0);
Ui10 = get_OriginalData(U10,0);


figure(4);
plot(0:size(Ui0(1,:),2)-1,Ui0(1,:),'b--',...
    0:size(Ui1(1,:),2)-1,Ui1(1,:),'k-.',...
    0:size(Ui2(1,:),2)-1,Ui2(1,:),'--g',...
    0:size(Ui3(1,:),2)-1,Ui3(1,:),':m',...
    0:size(Ui4(1,:),2)-1,Ui4(1,:),'-.y',...
    0:size(Ui5(1,:),2)-1,Ui5(1,:),'-.c',...
    0:size(Ui6(1,:),2)-1,Ui6(1,:),'--g',...
    0:size(Ui7(1,:),2)-1,Ui7(1,:),'-.m',...
    0:size(Ui8(1,:),2)-1,Ui8(1,:),':y',...
    0:size(Ui9(1,:),2)-1,Ui9(1,:),'-.b',...
    0:size(Ui10(1,:),2)-1,Ui10(1,:),'-r','linewidth',1.5);
xlabel('t ( s )','FontSize',12);
ylabel('$\bf{u_k ( t )}$','FontSize',14,'interpreter','latex')
legend ('Initial points','The 1st iteration','The 2nd iteration','The 3rd iteration',...
    'The 4th iteration','The 5th iteration','The 6th iteration','The 7th iteration',...
    'The 8th iteration','The 9th iteration','The 10th iteration',...
    'location','southeast');
hold off

