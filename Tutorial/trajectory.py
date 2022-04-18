def trajectory(x, u, N, dt,nmpc_ref,u_ref,u_old):
    #Based on the initial condition and optimized trajectory u, computed the path as (x,y,z).
    #Calculate the dynamic costs based on selected weights  
    import math
    import numpy as np
    ns = 8
    p_hist = []
    x_hist = []
    cost = 0
    ## Weight matrices
    Qx = (0,0,0, 0, 0,0, 5, 5)    
    #P = 2*Qx; #final state weight
    Ru = (7,7,7); # input weights
    Rd = (3, 3, 3); # input rate weights
    #print(x, u, N, dt)
    for i in range(0,N):
####State costs
        x_ref = nmpc_ref[(ns*i):(ns*i+ns)]
        #print(x_ref)
    
        cost = cost + Qx[0]*(x[0]-x_ref[0])**2 + Qx[1]*(x[1]-x_ref[1])**2 + Qx[2]*(x[2]-x_ref[2])**2 + Qx[3]*(x[3]-x_ref[3])**2 + Qx[4]*(x[4]-x_ref[4])**2 + Qx[5]*(x[5]-x_ref[5])**2 + Qx[6]*(x[6]-x_ref[6])**2 + Qx[7]*(x[7]-x_ref[7])**2  #State weights
####Input Cost
        u_n = u[(3*i):3*i+3]
        cost += Ru[0]*(u_n[0] - u_ref[0])**2 + Ru[1]*(u_n[1] - u_ref[1])**2 + Ru[2]*(u_n[2] - u_ref[2])**2 #Input weights
        cost += Rd[0]*(u_n[0] - u_old[0])**2 + Rd[1]*(u_n[1] - u_old[1])**2 + Rd[2]*(u_n[2] - u_old[2])**2 #Input rate weights
        u_old = u_n
        x_hist = x_hist + [x]
        x[0] = x[0] + dt * x[3]
        x[1] = x[1] + dt * x[4] 
        x[2] = x[2] + dt * x[5] 
        x[3] = x[3] + dt * (math.sin(x[7]) * math.cos(x[6]) * u[3*i] - 0.1 * x[3]) 
        x[4] = x[4] + dt * (-math.sin(x[6]) * u[3*i] - 0.1*x[4])
        x[5] = x[5] + dt * (math.cos(x[7]) * math.cos(x[6]) * u[3*i] - 0.2 * x[5] - 9.81) 
        x[6] = x[6] + dt * ((1 / 0.3) * (u[3*i+1] - x[6])) 
        x[7] = x[7] + dt * ((1 / 0.3) * (u[3*i+2] - x[7]))
        p_hist = p_hist + [[x[0],x[1],x[2]]]
    #print(cost)
    #print(p_hist)
    return(p_hist, cost, x_hist)
