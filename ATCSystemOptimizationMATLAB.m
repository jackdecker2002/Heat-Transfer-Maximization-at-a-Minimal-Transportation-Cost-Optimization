clear;clc;close;

% top-level system target
ProfitTarget = 100000000000; % TODO: Put a number here <-------- ADJUST

% run ATC 
[x_star,f_star,phi_star,exitflag] = transportation_system_atc_opt(ProfitTarget);

% display x*, f*, phi* after optimization
x_star
transportation_analysis(x_star(1),x_star(2))
phi_star

% re-calculate subsystem optimial solutions with optimal targets
subsys_target_geom = [x_star(1)];          % should be column vector
subsys_target_conv = [x_star(2)];          % should be column vector
x_geom_star = geometry_subsystem_atc_opt(subsys_target_geom);
x_conv_star = convection_subsystem_atc_opt(subsys_target_conv);

% display x_geom*
x_geom_star

% display x_conv*. where
x_conv_star 

%% --------------------------------------------------------------------------- DONE
% ATC otimization function for transportation system
%
function [x_star,f_star,phi_star,exitflag] = transportation_system_atc_opt(target)
    % Parameters
    
    weight = [1,1];

    % Intermediate Variables

    % linear inequiality constraits A x <= b
    A = [];
    b = [];
    
    % linear equality constraints Aeq x = beq
    Aeq = [];
    beq = [];

    % side constraints lb <= x <= ub (should be a column vector)
    % use Inf for a very (practically infintely) large values. 
    % L_fin
    lb(1) = 0.005;
    ub(1) = 0.01;
    
    % D_pipe_s
    lb(2) = 0.004;
    ub(2) = 0.01;

    % initial value for starting search
    for i = 1:2
    x0(i) = (ub(i)-lb(i))/2;    % just a random guess
    end

    % make column vecors
    lb = lb';
    ub = ub';
    x0 = x0'; 

    % optimize the system for ATC objective funciton (defined below), weight, 
    % and target; give a function handle to nonlinear inequality and equality 
    % constraints [c,ceq] = nonlcon(x) for the 2nd from the last input argument, 
    % and give true for last input argument to hide message from optimizer.
    [x_star,f_star,phi_star,exitflag] = atc_opt(@transportation_system_atc_obj,target,weight,x0,A,b,Aeq,beq,lb,ub,[],false);
end
%% --------------------------------------------------------------------------- DONE
% ATC objective function for transportation system
%
function[f,phi,phi_sub_star] = transportation_system_atc_obj(x,target)
    % parameters:
    weight = [1 1]; % weight for each subsystem target loss

    % system design variables and subsystem targets
    
    % Here x(1) means L_fin_s and x(2) means D_pipe_s
    
    subsys_target_geom = [x(1)];          % should be column vector
    subsys_target_conv = [x(2)];          % should be column vector

    % targets

    % intermediate variables
    [x_geom_star, f_geom_star, phi_geom_star,exitflag] = geometry_subsystem_atc_opt(subsys_target_geom);
    [x_conv_star, f_conv_star, phi_conv_star,exitflag] = convection_subsystem_atc_opt(subsys_target_conv);

    % system objective function: give zeo if undefined
    f = 0;

    % target loss: give zeo if undefined
    profit_s = transportation_analysis(x(1),x(2)); % Some function that will output the design space profit
    phi = (target-profit_s)^2;

    % optimal subsystem target loss: give zeo if undefined
    phi_sub_star = (weight(1)*phi_geom_star)+(weight(2)*phi_conv_star);
end

%% --------------------------------------------------------------------------- DONE
% ATC optimization function for geometry subsystem
%
function [x_star,f_star,phi_star,exitflag] = geometry_subsystem_atc_opt(target)
    % Parameters

    weight = [1,1]; % weights for phi and phi_sub_star relative to f

    % Intermediate Variables

    % linear inequiality constraits A x <= b
    A = [];
    b = [];
    
    % linear equality constraints Aeq x = beq
    Aeq = [];
    beq = [];

    % side constraints lb <= x <= ub (should be a column vector)
    % use Inf for a very (practically infintely) large values. 
    % L_fin
    lb(1) = 0.005;
    ub(1) = 0.01; 
    
    % t_fin
    lb(2) = 0.001;
    ub(2) = 0.01; 

    % N_fin
    lb(3) = 1;
    ub(3) = 25; 
   
    % initial value for starting search
    for i = 1:3
        x0(i) = (ub(i)-lb(i))/2;    % just a random guess
    end
    
    % make column vecors
    lb = lb';
    ub = ub';
    x0 = x0';

    % optimize the system for ATC objective funciton (defined below), weight, 
    % and target; give a function handle to nonlinear inequality and equality 
    % constraints [c,ceq] = nonlcon(x) for the 2nd from the last input argument, 
    % and give true for last input argument to hide message from optimizer.
    [x_star,f_star,phi_star,exitflag] = atc_opt(@geometry_subsystem_atc_obj,target,weight,x0,A,b,Aeq,beq,lb,ub,@geometry_constr,true);
end

%% --------------------------------------------------------------------------- DONE
% ATC objective function for geometry subsystem
%
function [f,phi,phi_sub_star] = geometry_subsystem_atc_obj(x,target)
    % Parameters

    % system design variables and subsystem targets
    % System design variables are L_fin, t_fin, and N_fin

    % targets
    % This system targets L_fin_s

    % intermediate variables
    L_fin_opt = geometry_analysis(x(1),x(2),x(3)); % I need to create a function here that handles geometric analysis based on L_fin, t_fin, and N_fin

    % system objective function: give zeo if undefined
    f = 0;

    % target loss: give zeo if undefined
    phi = (target-L_fin_opt)^2;

    % optimal subsystem target loss: give zeo if undefined
    phi_sub_star = 0;
end

%% --------------------------------------------------------------------------- DONE
% ATC optimization function for convection subsystem
%
function [x_star,f_star,phi_star,exitflag] = convection_subsystem_atc_opt(target)
    % parameters: 
    weight = [1, 1];   % weights for phi and phi_sub_star relative to f
    
    % intermediate variables

    % linear inequiality constraits A x <= b
    A = [];
    b = [];
    
    % linear equality constraints Aeq x = beq
    Aeq = [];
    beq = [];

    % side constraints lb <= x <= ub (should be a column vector)
    % use Inf for a very (practically infintely) large values.     
    % v_fluid
    lb(1) = 0;
    ub(1) = 3; 
    
    % D_pipe
    lb(2) = 0.004;
    ub(2) = 0.01;     

    % initial value for starting search
    for i = 1:2
        x0(i) = (ub(i)-lb(i))/2;    % just a random guess
    end
    
    % make column vecors
    lb = lb';
    ub = ub';
    x0 = x0';

    % optimize the system for ATC objective funciton (defined below), weight, 
    % and target; give a function handle to nonlinear inequality and equality 
    % constraints [c,ceq] = nonlcon(x) for the 2nd from the last input argument, 
    % and give true for last input argument to hide message from optimizer.
    [x_star,f_star,phi_star,exitflag] = atc_opt(@convection_subsystem_atc_obj,target,weight,x0,A,b,Aeq,beq,lb,ub,@convection_constr,true);
end

%% --------------------------------------------------------------------------- DONE
% ATC objective function for convection subsystem
%
function [f,phi,phi_sub_star] = convection_subsystem_atc_obj(x,target)

    % parameters
    
    % system design variables and subsystem targets
    % The design variables for this system is v_fluid and D_pipe
    
    % targets
    % The target is D_pipe_s

    % intermediate variables
    D_pipe_opt = convection_analysis(x(1),x(2)); % Come up with some function here that takes in v_fluid and D_pipe to optimize
    
    % system objective function: give zeo if undefined
    f = 0;
    
    % target loss: give zeo if undefined
    phi = (target-D_pipe_opt)^2;

    % optimal subsystem target loss: give zeo if undefined
    phi_sub_star = 0;
end

%% --------------------------------------------------------------------------- Andrew - Done I Think
% % TODO: Nonlinear constraints for geometry subsystem
%
function [c,ceq] = geometry_constr(x)

    % parameters
    w = 0.076;
    T_Inf = 294.26;
    T_S = 331.15;
    B = 0.00339;
    v = 0.000015;
    g = 9.81;
    k = 385;
    eff = 0.8;
    
    % system design variables and subsystem targets
    % Design Variables:
    % L_fin = x(1)
    % t_fin = x(2)
    % N_fin = x(3)
    
    % intermediate variables
    deltaT = T_S - T_Inf;
    Ra = g * B * (deltaT) * (x(1)^3) / (v^2);
    Nu = 0.59 * (Ra^0.25);
    h = Nu * k / x(1);
    t = (k/h)^0.5;
    S = 2.714 * w * (Ra^0.25);
    Q = x(3) * h * x(1) * w * deltaT * eff;

    % inequality constraints: should be be a column vector
	c = [-Q + 25000;   
        (x(3)-1)*S + (x(3)*t) - 0.1];

    % equality constraints: should be a column vector
    ceq = [];
end

%% --------------------------------------------------------------------------- MIGUEL - DONE
% %TODO: nonlinear constraints for convection subsystem
%
function [c,ceq] = convection_constr(x)

    % parameters
    
    
    % system design variables and subsystem targets
    % Design Variables:
    % v_fluid = x(1)
    % D_pipe = x(2)
    
    % intermediate variables

    % inequality constraints: should be be a column vector
	c = [];
      
    % equality constraints: should be a column vector
    ceq = [];
end

%% --------------------------------------------------------------------------- Andrew - DONE I Think
% geometry subsystem analysis function: returns a numerical solution of an implicit
% governing equation
%
function L_fin_opt = geometry_analysis(L_fin,t_fin,N_fin)

   % We need to do some work here to give the optimal L_fin based on the
   % inputs L_fin, t_fin, and N_fin

   % TODO
   % parameters
    w = 0.076;
    T_Inf = 294.26;
    T_S = 331.15;
    B = 0.00339;
    v = 0.000015;
    g = 9.81;
    k = 385;
    eff = 0.8;

    % intermediate variables 
    deltaT = T_S - T_Inf;
    Ra = g * B * (deltaT) * (L_fin^3) / (v^2);
    Nu = 0.59 * (Ra^0.25);
    h = Nu * k / L_fin;
    % t = (k/h)^0.5; Unnecessary
    S = 2.714 * w * (Ra^0.25);
    Q = N_fin * h * L_fin * w * deltaT * eff;

    % objective function
    L_fin_opt = L_fin;
    
end

%% --------------------------------------------------------------------------- MIGUEL - DONE
% convection subsystem analysis function
function D_pipe_opt = convection_analysis(v_fluid,D_pipe)

    % --- Economic Parameters ---
    c1 = 3e-6;  % Value per J of heat removed (formerly 1e-4, 3e-8)
    c2 = 0.3;    % Cost per J of energy consumed (formerly 0.3)

    % --- Physical Constants ---
    rho = 1000;       % kg/m^3 (density of water)
    mu = 0.001;       % Pa·s (dynamic viscosity)
    k = 0.606;        % W/m·K (thermal conductivity)
    cp = 4186;        % J/kg·K (specific heat of water)
    L = 0.1;          % m (pipe length)
    T_fluid = 298.15; % K (temperature difference)
    T_cpu = 313.15;
    deltaT = T_cpu - T_fluid;
    pi_val = pi;
    
    % --- Precompute constants ---
    Pr = cp * mu / k;
    Ch = 0.023 * k * Pr^0.3 * (rho / mu)^0.8;    % Heat transfer coefficient constant
    Cq = Ch * pi_val * L * deltaT;              % Q(v,D) coefficient
    Cp = (pi_val * rho) / 8;                    % P(v,D) coefficient
    
    % --- Discrete pipe diameters (in meters) ---
    D_values = [0.004, 0.006, 0.008, 0.010];
    
    % --- Cost Function Handle ---
    params = struct('c1', c1, 'c2', c2, 'Cq', Cq, 'Cp', Cp, 'D_values', D_values);
    cost_fn = @(x) coolingCost_New(x, params);
    
    % --- Simulated Annealing Settings ---
    opts = optimoptions('simulannealbnd', 'Display', 'off', ...
                        'MaxIterations', 100);
    
    % Variables: x(1) = flow velocity, x(2) = diameter index
    lb = [0, 1]; 
    ub = [2.12, length(D_values)]; % based on steady state temp of device 85 celsius and assuming qin for device is 39 W and max would be taking all that heat away as if air does nothing
    x0 = [v_fluid, D_pipe];
    
    % --- Run Optimization ---
    [x_opt, ~] = simulannealbnd(cost_fn, x0, lb, ub, opts);
    
    % --- Extract Optimal Values ---
    D_pipe_opt = D_values(round(x_opt(2)));
    
    % --- Cost Function Definition ---
    function cost = coolingCost_New(x, params)
        v = x(1);
        D_index = round(x(2));
        D_index = max(1, min(length(params.D_values), D_index));
        D = params.D_values(D_index);
    
        Q = params.Cq * v^0.8 * D^0.8;          % Heat removed [W = J/s]
        P = params.Cp * D^2 * v^3;              % Power consumption [W = J/s]
    
        cost = -(params.c1 * Q - params.c2 * P);  % Negative net benefit to minimize
    end
end

%% --------------------------------------------------------------------------- DONE

% transportation subsystem analysis function
%
function Profit_opt = transportation_analysis(L_fin,D_pipe)
   
   % Parameters:
   V_truck = 8*10*18; % ft^3
   d = 2463; % miles
   C_gal = 3.11; % $/gal
   n = 7; % Truck effiency, miles/gal
   RWT = 0.57; % $/mile
   E = 0.1; % g/(ton-mile)
   R_unit = 15.50; % $/unit
   C_carbontax = 0.00005; % $/gram
   rho = 0.279; % ton/ft^3
   C_gas = C_gal * d / n; % $
   C_WT = d * RWT; % $
   % Intermediate Variables:
   VolumeHS = 56*10^(-3)*76*10^(-3)*(0.002+D_pipe+L_fin); % m^3
   q = (V_truck/35.315)/VolumeHS; % quantity
   m = (VolumeHS*35.315) * q * rho; % output in tons
   m_emissions = d*m*E; % output in grams
   Revenue = R_unit*q; % $
   % Final deliverable function
   Profit_opt = Revenue - C_gas - C_WT^0.95 - (C_carbontax * m_emissions)^1.05;
   
end

%% --------------------------------------------------------------------------- DONE
% ATC subsystem optimization helper function 
%
function [x_star, f_star, phi_star, exitflag] = atc_opt(atc_obj,target,weight,x0,A,b,Aeq,beq,lb,ub,nonlcon,quiet)
    
    % define a dummy scalar function of x only 
    function f_atc = atc_obj_dummy(x)  
        [f, phi, phi_sub_star] = atc_obj(x,target);

        % ATC objective function: f + w*target_loss + w*subsys_target_loss*
        f_atc = f + weight(1)*phi + weight(2)*phi_sub_star;
    end

    if quiet == true
        % solve the problem: f_atc_star unused
        options = optimoptions('fmincon','Display','none');   % hide messages
        [x_star, f_atc_star, exitflag] = fmincon(@atc_obj_dummy,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
    else
        [x_star, f_atc_star, exitflag] = fmincon(@atc_obj_dummy,x0,A,b,Aeq,beq,lb,ub,nonlcon);
    end
  
    % evaluate again with x* to recover each term of ATC objective function
    % phi_sub_star is not passed to super system so unused here
    [f_star, phi_star, phi_sub_star] = atc_obj(x_star,target);
end
