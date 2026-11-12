"""Generate the optocoupler circuit and every independent tolerance corner."""
from itertools import product
import math
import numpy as np
from hw.spice import simulate

def netlist(r1=887,r2=887,shunt=453,pull=10000,vz=2.7,vf=1.25,ctr=2,transient=False):
    led_is = .01/math.expm1(vf/(2*.02585))
    source = 'PWL(0 0 1m 0 1.0001m 24 3m 24 3.0001m 0 5m 0)' if transient else '0'
    return f'''24 V isolated sinking input, TLP291 GB, low MCU level = ON
Vin inp 0 {source}
D1 inp a rect
R1 a b {r1}
R2 b z {r2}
DZ1 led z zener
R3 led 0 {shunt}
Vled led ld 0
DLED ld 0 optoled
Vcc vcc 0 3.3
R4 vcc out {pull}
Fphoto 0 photo Vled {ctr}
Rphoto photo 0 1k
Rdelay photo drive 1k
Cdelay drive 0 7n
Bsink out 0 I=max(v(drive)/1000,0)*max(v(out)-0.2,0)/(max(v(out)-0.2,0)+0.01)
Idark out 0 10u
Cout out 0 30p
.model rect D(Is=1n N=1.6 Rs=0.1 BV=1000 IBV=5u)
.model zener D(Is=1p N=1 Rs=10 BV={vz} IBV=5m)
.model optoled D(Is={led_is:.12g} N=2 Rs=0)
.options reltol=1e-5 abstol=1e-12
{'.tran 0.2u 5m 0 0.2u' if transient else '.dc Vin -30 30 0.02'}
.end
'''

def run_corners():
    rows,envelope = [],[]
    for factors in product((.99,1.01),repeat=4):
        r1,r2,shunt,pull = np.array([887,887,453,10000])*factors
        for vz,vf,ctr in product((2.5,2.9),(1.1,1.4),(1,4)):
            d = simulate(netlist(r1,r2,shunt,pull,vz,vf,ctr))
            v,out = d['inp'],d['out']
            current,led = -d['vin#branch'],d['vled#branch']
            on = v[np.flatnonzero(out<=.99)[0]]
            off = v[np.flatnonzero(out>=2.31)[-1]]
            at = lambda values,voltage: float(np.interp(voltage,v,values))
            i30 = at(current,30)
            rows.append(dict(on=float(on),off=float(off),i11=at(current,11),i30=i30,
                reverse_led=at(led,-30),reverse_i=at(current,-30),
                reverse_d1=at(d['inp']-d['a'],-30),reverse_led_v=at(d['ld'],-30),
                reverse_z=at(d['z']-d['led'],-30),reverse_out=at(out,-30),
                leakage_out=at(out,float(np.interp(.0015,current,v))),
                r1_w=i30*i30*r1,r2_w=i30*i30*r2,r3_w=at(d['led'],30)**2/shunt,
                r4_w=(3.3-at(out,30))**2/pull,z_w=i30*at(d['z']-d['led'],30)))
            envelope.append(current)
    return rows,v,np.array(envelope)

def transient(ctr):
    d = simulate(netlist(ctr=ctr,transient=True))
    t,out = d['time'],d['out']
    def crossing(start,level,rising):
        ids = np.flatnonzero((t>=start)&((out>=level) if rising else (out<=level)))
        k = ids[0]
        return float(np.interp(level,[out[k-1],out[k]] if rising else [out[k],out[k-1]],
                              [t[k-1],t[k]] if rising else [t[k],t[k-1]])-start)
    return d,dict(ctr=ctr,on_s=crossing(.00100005,.99,False),off_s=crossing(.00300005,2.31,True))

if __name__ == '__main__':
    rows,*_ = run_corners()
    print(f'{len(rows)} corners; ON <= {max(r["on"] for r in rows):.3f} V')
