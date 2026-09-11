"""Short-horizon centroidal force preview, estimated state and scheduled contacts.

Linearized world-frame rigid-body dynamics. Position/attitude, linear/angular
velocity (12 states); four 3D contact forces. No contact truth. This is a
centroidal MPC approximation, not full-body horizon optimization or WBIC.
"""
import numpy as np
import osqp
from scipy import sparse
from position_wbc import skew
from support_shift import OFFSETS,swing_path


def solve_preview(state,mass,inertia,feet,center,phase,params,forward_speed,
                  friction=.5,horizon_steps=12,step_s=.08):
    N=horizon_steps;dt=step_s;nx=12;nu=12;nxall=nx*(N+1);n=nxall+nu*N
    H=np.eye(n)*1e-7;g=np.zeros(n);rows=[];low=[];high=[]
    initial=np.zeros((nx,n));initial[:,:nx]=np.eye(nx)
    rows.append(initial);low.append(state);high.append(state)
    A=np.eye(nx);A[:6,6:]=np.eye(6)*dt
    invI=np.linalg.inv(inertia)
    gravity=np.r_[0,0,-9.81,0,0,0.]
    c=np.r_[.5*dt*dt*gravity,dt*gravity]
    for k in range(N):
        fstart=nxall+k*nu;xs=(k+1)*nx
        predicted_feet=np.array(feet).copy()
        scheduled=[]
        for leg,offset in enumerate(OFFSETS):
            q=(phase+(k+.5)*dt/params[0]+offset)%1
            scheduled.append(q<params[1])
            old=swing_path((phase+offset)%1,params[1])[0]
            new=swing_path(q,params[1])[0]
            predicted_feet[leg,0]+=(new-old)*params[2]
        F=np.zeros((6,nu))
        for leg,p in enumerate(predicted_feet):
            F[:3,3*leg:3*leg+3]=np.eye(3)/mass
            F[3:,3*leg:3*leg+3]=invI@skew(p-center)
        B=np.vstack([.5*dt*dt*F,dt*F])
        row=np.zeros((nx,n));row[:,xs:xs+nx]=np.eye(nx)
        row[:,k*nx:(k+1)*nx]=-A;row[:,fstart:fstart+nu]=-B
        rows.append(row);low.append(c);high.append(c)
        weights=np.array([30,2000,5000,200,200,20,20,40,60,5,5,1.])
        ref=np.zeros(nx);ref[0]=forward_speed*(k+1)*dt;ref[6]=forward_speed
        H[xs:xs+nx,xs:xs+nx]+=np.diag(weights)
        g[xs:xs+nx]-=weights*ref
        H[fstart:fstart+nu,fstart:fstart+nu]+=np.eye(nu)*.0001
        # Permit the estimator's existing error in the first state, but future
        # planned sideways/height excursions must remain within 10 mm.
        bounds=np.zeros((2,n));bounds[0,xs+1]=1;bounds[1,xs+2]=1
        rows.append(bounds);low.append(np.array([-.01,-.01]));high.append(np.array([.01,.01]))
        for leg in range(4):
            fs=fstart+3*leg
            if not scheduled[leg]:
                row=np.zeros((3,n));row[:,fs:fs+3]=np.eye(3)
                rows.append(row);low.append(np.zeros(3));high.append(np.zeros(3));continue
            row=np.zeros((5,n));row[0,fs+2]=1
            for r,(sx,sy) in enumerate(((1,1),(1,-1),(-1,1),(-1,-1)),1):row[r,fs:fs+3]=[sx,sy,-friction]
            rows.append(row);low.append(np.array([0,-np.inf,-np.inf,-np.inf,-np.inf]))
            high.append(np.array([mass*9.81*2,0,0,0,0]))
    matrix=np.vstack(rows);lo=np.concatenate(low);hi=np.concatenate(high)
    # Scale metres/radians/newtons before ADMM. Otherwise the 10 mm hard
    # position bounds and force variables differ by several orders of magnitude.
    scale=np.r_[np.tile([.01,.01,.01,.1,.1,.1,.1,.1,.1,1.,1.,1.],N+1),np.full(nu*N,10.)]
    scaled_matrix=matrix*scale
    row_scale=1/np.maximum(np.max(abs(scaled_matrix),axis=1),1e-8)
    solver=osqp.OSQP();solver.setup(P=sparse.csc_matrix(np.triu(scale[:,None]*H*scale)),q=g*scale,
        A=sparse.csc_matrix(row_scale[:,None]*scaled_matrix),
        l=lo*row_scale,u=hi*row_scale,verbose=False,eps_abs=1e-5,eps_rel=1e-5,max_iter=3000,polishing=True)
    answer=solver.solve(raise_error=False)
    if answer.x is None or answer.info.status_val not in (1,2):return None,{'status':answer.info.status}
    x=answer.x*scale;violation=float(max(0,np.max(lo-matrix@x),np.max(matrix@x-hi)))
    if violation>2e-4:return None,{'status':'constraint-residual-rejected','violation':violation}
    states=x[:nxall].reshape(N+1,nx);forces=x[nxall:].reshape(N,4,3)
    return (states,forces),{'status':answer.info.status,'constraint_violation':violation,
        'horizon_s':N*dt,'lateral_peak_m':float(abs(states[1:,1]).max()),
        'height_error_peak_m':float(abs(states[1:,2]).max()),'iterations':int(answer.info.iter)}
