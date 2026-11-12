#!/usr/bin/env python3
"""Offline nominal dynamics scan; Panda URDF identified inertias, not libfranka.

Run after sourcing ROS and devel_diagnostics. No robot/network access.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, '/opt/ros/noetic/lib/python3.8/site-packages')
import numpy as np
import pinocchio as pin

K0 = np.diag([100., 100., 100., 40., 40., 40.])
D0 = np.diag([10., 10., 10., 4., 4., 4.])


def jacobian(model, data, q, fid):
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    return pin.getFrameJacobian(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).copy()


def matrices(model, data, q, fid):
    j = jacobian(model, data, q, fid)
    m = pin.crba(model, data, q).copy()
    return m, j


def linear_scan(m, j, ks, ds, dt):
    stiffness = j.T @ (ks*K0) @ j
    damping = j.T @ (ds*D0) @ j
    s = np.linalg.solve(m, stiffness)
    t = np.linalg.solve(m, damping)
    eye = np.eye(7)
    a = np.block([[eye-dt*dt*s, dt*eye-dt*dt*t], [-dt*s, eye-dt*t]])
    vals = np.linalg.eigvals(a)
    # Cartesian task leaves one redundant joint-space direction neutral.
    nonneutral = vals[np.abs(vals-1) > 1e-6]
    dom = nonneutral[np.argmax(np.abs(nonneutral))]
    return float(np.abs(dom)), float(dom.real), float(dom.imag)


def nonlinear_scan(model, q0, v0, fid, ks, ds, dt, duration=0.2, trace=None):
    data = model.createData()
    q = q0.copy(); v = v0.copy()
    jacobian(model, data, q, fid)
    target = data.oMf[fid].copy()
    last_sign = np.sign(v[6]); sign_changes = 0
    for n in range(int(round(duration / dt))):
        j = jacobian(model, data, q, fid)
        current = data.oMf[fid]
        relative = pin.Quaternion(current.rotation.T @ target.rotation)
        if relative.w < 0:
            relative.coeffs()[:] *= -1
        error = np.concatenate((target.translation-current.translation,
                                2*current.rotation @ relative.coeffs()[:3]))
        mass = pin.crba(model, data, q).copy()
        coriolis = pin.computeCoriolisMatrix(model, data, q, v).copy() @ v
        acceleration = np.linalg.solve(mass, j.T @ (ks*K0@error-ds*D0@(j@v))-coriolis)
        # Same semi-implicit Euler sequence used by the controller.
        v = v + dt*acceleration
        q = q + dt*v
        if trace is not None:
            trace.append([n+1, (n+1)*dt] + v.tolist() + acceleration.tolist())
        if not np.isfinite(v).all() or np.max(np.abs(v)) > 50:
            return (n+1)*dt, float(np.max(np.abs(v))), sign_changes, True
        sign = np.sign(v[6])
        if sign and last_sign and sign != last_sign: sign_changes += 1
        if sign: last_sign = sign
    return duration, float(np.max(np.abs(v))), sign_changes, False


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--csv', required=True)
    parser.add_argument('--output', default='analysis/nominal_dynamics/results')
    args=parser.parse_args()
    os.makedirs(args.output, exist_ok=True)
    with open(args.csv, newline='') as f:
        data=list(csv.DictReader(f))
    initial=data[0]
    q=np.array([float(initial[f'q_{i}']) for i in range(1,8)])
    v=np.array([float(initial[f'dq_{i}']) for i in range(1,8)])
    input_file='src/franka_ros/franka_description/robots/panda/panda.urdf.xacro'
    urdf=subprocess.check_output(['xacro',input_file,'hand:=false','gazebo:=true','parent:='],text=True)
    with tempfile.NamedTemporaryFile(mode='w',suffix='.urdf') as f:
        f.write(urdf); f.flush(); model=pin.buildModelFromUrdf(f.name)
    assert model.nq==7 and model.nv==7
    fid=model.getFrameId('panda_link8')
    mass,jac=matrices(model,model.createData(),q,fid)
    np.savetxt(os.path.join(args.output,'approx_mass.csv'), mass, delimiter=',', fmt='%.12g')
    np.savetxt(os.path.join(args.output,'approx_jacobian.csv'), jac, delimiter=',', fmt='%.12g')
    results=[]
    baseline_trace=[]
    for ks in [0.25,0.5,1.0,2.0,4.0]:
        for ds in [0.25,0.5,1.0,2.0,4.0]:
            for dt in [0.00025,0.0005,0.001,0.002,0.004]:
                rho,re,im=linear_scan(mass,jac,ks,ds,dt)
                trace = baseline_trace if (ks==1 and ds==1 and dt==0.001) else None
                elapsed,peak,flips,diverged=nonlinear_scan(model,q,v,fid,ks,ds,dt,trace=trace)
                results.append(dict(stiffness_scale=ks,damping_scale=ds,dt=dt,
                    linear_rho=rho,dominant_real=re,dominant_imag=im,
                    linear_unstable=rho>1+1e-6,nonlinear_time=elapsed,
                    final_or_stop_max_joint_speed=peak,joint7_sign_changes=flips,
                    nonlinear_speed_over_50=diverged))
    with open(os.path.join(args.output,'scan.csv'),'w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(results[0]));w.writeheader();w.writerows(results)
    with open(os.path.join(args.output,'baseline_trace.csv'),'w',newline='') as f:
        w=csv.writer(f); w.writerow(['step','time']+[f'nominal_dq_{i}' for i in range(1,8)]+[f'nominal_ddq_{i}' for i in range(1,8)]); w.writerows(baseline_trace)
    summary=dict(input_csv=os.path.abspath(args.csv),model='Panda xacro Gazebo identified inertias, panda_link8, hand=false, no tool load; NOT Franka internal model',
       source_initial_q=q.tolist(),source_initial_dq=v.tolist(),
       mass_eigenvalues=np.linalg.eigvalsh(mass).tolist(),
       generalized_damping_eigenvalues=np.sort(np.real(np.linalg.eigvals(np.linalg.solve(mass,jac.T@D0@jac)))).tolist(),
       warning='Approximation uses no measured F_T_EE or load from this run; does not validate hardware control.',
       count=len(results), default_case=next(r for r in results if r['stiffness_scale']==1 and r['damping_scale']==1 and r['dt']==.001))
    with open(os.path.join(args.output,'summary.json'),'w') as f: json.dump(summary,f,indent=2)
    print(json.dumps(summary,indent=2))

if __name__ == '__main__': main()
