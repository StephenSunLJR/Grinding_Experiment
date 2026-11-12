#!/usr/bin/env python3
"""Core claim figure: isotropic scalar damping (M2) vs anisotropic vector damping (M4)"""
import csv, argparse, os
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.size']=11; plt.rcParams['axes.grid']=True; plt.rcParams['grid.alpha']=0.2

def load(fn, lo=8.0, hi=10.0):
    with open(fn) as f:
        r=csv.reader(f); next(r); rows=[x for x in r if len(x)>92 and lo<float(x[0])<=hi]
    Lc=np.array([[float(x[87+i]) for i in range(6)] for x in rows])
    err=np.array([[float(x[20+i]) for i in range(6)] for x in rows])*1e3
    return Lc.mean(0), np.sqrt((err**2).mean(0))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--m2', required=True); ap.add_argument('--m4', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--cs', type=float, default=8.0); ap.add_argument('--ce', type=float, default=10.0)
    a=ap.parse_args(); os.makedirs(a.output, exist_ok=True)

    cart=['X','Y','Z','RX','RY','RZ']
    lc2,rms2=load(a.m2, a.cs, a.ce); lc4,rms4=load(a.m4, a.cs, a.ce)

    fig,(ax,bx)=plt.subplots(1,2,figsize=(12,4.8))
    fig.suptitle('Isotropic scalar damping (M2)  vs  Anisotropic vector damping (M4)',
                 fontweight='bold', fontsize=12)

    # (a) damping
    x=np.arange(6); w=0.35
    ax.bar(x-w/2,lc2,w,label='M2  scalar DLS',color='tab:orange',edgecolor='white',lw=0.5)
    ax.bar(x+w/2,lc4,w,label='M4  A-DLS',color='tab:red',edgecolor='white',lw=0.5)
    ax.axhline(1e-4,ls='--',c='gray',lw=0.8)
    ax.text(5.4,1.3e-4,r'$\lambda^2_{base}$',fontsize=8,color='gray',va='bottom')
    ax.set_yscale('log'); ax.set_xticks(x); ax.set_xticklabels(cart)
    ax.set_ylabel(r'Damping $\lambda^2_i$ (collision avg)'); ax.set_xlabel('Cartesian direction')
    ax.set_title('(a) Damping distribution', fontsize=11)
    ax.legend(fontsize=9, loc='upper right', framealpha=0.8)

    # (b) tracking error
    x3=np.arange(3)
    bx.bar(x3-w/2,rms2[:3],w,label='M2',color='tab:orange',edgecolor='white',lw=0.5)
    bx.bar(x3+w/2,rms4[:3],w,label='M4',color='tab:red',edgecolor='white',lw=0.5)
    bx.set_xticks(x3); bx.set_xticklabels(['X (healthy)','Y (healthy)','Z (collision)'])
    bx.set_ylabel('Tracking RMSE [mm]'); bx.set_xlabel('Cartesian direction')
    bx.set_title('(b) Tracking error', fontsize=11)
    bx.legend(fontsize=9, framealpha=0.8)
    for i,(v2,v4) in enumerate(zip(rms2[:3],rms4[:3])):
        bx.text(i-w/2,v2+0.8,f'{v2:.1f}',ha='center',fontsize=9)
        bx.text(i+w/2,v4+0.8,f'{v4:.1f}',ha='center',fontsize=9)

    fig.tight_layout(rect=[0,0,1,0.93])
    fig.savefig(a.output+'/fig_core_M2vsM4.png',dpi=150, bbox_inches='tight')
    fig.savefig(a.output+'/fig_core_M2vsM4.pdf', bbox_inches='tight')
    print(f'  -> {a.output}/fig_core_M2vsM4.png/pdf')
    print(f'  M2: Lc={np.round(lc2,4)}  RMSE_XYZ={np.round(rms2[:3],1)}')
    print(f'  M4: Lc={np.round(lc4,4)}  RMSE_XYZ={np.round(rms4[:3],1)}')

if __name__=='__main__':
    main()
