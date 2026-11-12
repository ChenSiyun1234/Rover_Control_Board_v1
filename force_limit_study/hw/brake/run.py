"""Brake driver model and clamp/supply/tolerance simulations."""
from itertools import product
import numpy as np
from hw.spice import simulate
OFF = .05

def netlist(kind='a',supply=24,corner=0,reset=False,mos_corner=None,rg_factor=1,pd_factor=1):
    z24,z43 = ((26.7,47.8) if corner<0 else (29.5,52.8) if corner>0 else (28.1,50.3))
    r24,r43 = (38.9-29.5)/15.4,(69.4-52.8)/8.6
    gate = ('Vcmd cmd 0 3.3\nVenable en 0 PWL(0 1 50m 1 50.0001m 0)\n'
            'Sreset cmd mcu en 0 disconnect\n.model disconnect SW(Ron=0.01 Roff=1e15 Vt=0.5 Vh=0.1)'
            if reset else 'Vcmd mcu 0 PWL(0 3.3 50m 3.3 50.0001m 0)')
    clamp = {'a':'Vcl drain ca 0\nD2 ca vs rect',
             'b':'Vcl drain ca 0\nD2 ca cz rect\nDZ2 vs cz tv24',
             'c':'Vcl drain ca 0\nDZ3 0 ca tv43'}[kind]
    mc = corner if mos_corner is None else mos_corner
    vto = (1 if mc<0 else 2 if mc>0 else 1.5)
    invkp = (.063-.044)/(1/(4-vto)-1/(10-vto))
    kp = 1/invkp
    rd = .044-invkp/(10-vto)
    return f'''24 V spring applied brake, clamp {kind}
Vs vs 0 {supply}
Rcoil vs top 72
Lcoil top drain 0.6
{gate}
Rg mcu gate {330*rg_factor}
Rpd gate 0 {10000*pd_factor}
M1 drain gate 0 0 power L=1u W=1u
Cgs gate 0 1.63n
Cgd gate drain 170p
Cds drain 0 180p
{clamp}
.model power NMOS(Level=1 Vto={vto} Kp={kp} Rd={rd} Lambda=0)
.model rect D(Is=1n N=1.6 Rs=0.1 BV=1000 IBV=5u)
.model tv24 D(Is=1p N=1 Rs={r24} BV={z24} IBV=1m)
.model tv43 D(Is=1p N=1 Rs={r43} BV={z43} IBV=1m)
.options reltol=1e-5 abstol=1e-10 method=gear
.tran 10u 120m 0 10u
.end
'''

def run(kind='a',supply=24,corner=0,reset=False,mos_corner=None,rg_factor=1,pd_factor=1):
    d = simulate(netlist(kind,supply,corner,reset,mos_corner,rg_factor,pd_factor))
    t,i,vd = d['time'],d['lcoil#branch'],d['drain']
    held = float(np.interp(OFF,t,i))
    after = t>=OFF
    k = np.flatnonzero(after & (i<=.3*held))[0]
    drop = float(np.interp(.3*held,[i[k],i[k-1]],[t[k],t[k-1]])-OFF)
    clamp_v = vd-(d['vs'] if kind in 'ab' else 0)
    power = np.maximum(clamp_v*d['vcl#branch'],0)
    energy = float(np.trapezoid(power[after],t[after]))
    return d,dict(kind=kind,supply=supply,corner=corner,reset=reset,mos_corner=mos_corner,rg_factor=rg_factor,pd_factor=pd_factor,held_a=held,
        delay_s=drop,peak_v=float(vd[after].max()),energy_j=energy,
        clamp_peak_w=float(power[after].max()),fall_mm=.5*9.80665*drop**2*1000,
        gate_end_v=float(d['gate'][-1]),current_end_a=float(i[-1]),
        rg_w=float(np.max((d['mcu']-d['gate'])**2/(330*rg_factor))),
        rpd_w=float(np.max(d['gate']**2/(10000*pd_factor))))

def run_corners():
    rows = [run(kind,v,0)[1] for kind,v in product('abc',(21.6,24,26.4))]
    rows += [run(kind,v,c,False,m,rg,pd)[1] for kind,v,c,m,rg,pd in
             product('abc',(21.6,24,26.4),(-1,1),(-1,1),(.99,1.01),(.99,1.01))]
    return rows

if __name__ == '__main__':
    for row in run_corners():
        print(row)
