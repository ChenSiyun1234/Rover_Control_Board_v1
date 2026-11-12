"""Regenerate all netlists, CSVs, report tables, schematics and waveform figures."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from hw.io24.run import netlist as input_netlist, run_corners as input_corners, transient
from hw.brake.run import netlist as brake_netlist, run_corners as brake_corners, run as brake_run, OFF
from hw.spice import simulate
from hw.draw import draw_all
ROOT = Path(__file__).resolve().parent

def csv_save(path,rows):
    with path.open('w',newline='',encoding='utf-8') as f:
        writer = csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+
                     ['| '+' | '.join(map(str,row))+' |' for row in rows])

def main():
    (ROOT/'out').mkdir(exist_ok=True)
    (ROOT/'io24/input.cir').write_text(input_netlist(),encoding='ascii')
    (ROOT/'io24/step.cir').write_text(input_netlist(transient=True),encoding='ascii')
    for kind in 'abc':
        (ROOT/f'brake/clamp_{kind}.cir').write_text(brake_netlist(kind),encoding='ascii')
    (ROOT/'brake/reset.cir').write_text(brake_netlist('c',26.4,1,True),encoding='ascii')
    rows,v,envelope = input_corners()
    csv_save(ROOT/'io24/corners.csv',rows)
    nominal = simulate(input_netlist())
    csv_save(ROOT/'io24/dc.csv',[dict(voltage=float(x),input_a=float(i),led_a=float(j),output_v=float(o))
        for x,i,j,o in zip(v,-nominal['vin#branch'],nominal['vled#branch'],nominal['out'])])
    fig,ax = plt.subplots(3,1,figsize=(10,10),sharex=True)
    # OFF is the union of the low-voltage and low-current arms, not a rectangle to 11 V / 15 mA.
    for xy,w,h,color,label in [((-3,0),8,15,'#c4e4ff','OFF: -3 to 5 V'),
                              ((5,0),6,1.5,'#a5d6f5','OFF: <=1.5 mA through 11 V'),
                              ((5,1.5),6,13.5,'#fff0b0','Transition'),
                              ((11,2),19,13,'#ccebc5','ON: 11-30 V, 2-15 mA')]:
        ax[0].add_patch(Rectangle(xy,w,h,facecolor=color,alpha=.6,label=label))
    ax[0].fill_between(v,envelope.min(axis=0)*1000,envelope.max(axis=0)*1000,color='black',alpha=.25,label='128 corners')
    ax[0].plot(v,-nominal['vin#branch']*1000,label='Nominal input current')
    ax[0].set(ylabel='Input current (mA)',ylim=(-.5,16),title='24 V sinking input: Type 3 regions and model envelope')
    ax[0].legend(fontsize=8,loc='upper left')
    ax[1].plot(v,nominal['vled#branch']*1000); ax[1].set(ylabel='LED current (mA)')
    ax[2].plot(v,nominal['out']); ax[2].axhline(.99,ls='--'); ax[2].axhline(2.31,ls='--')
    ax[2].set(xlabel='Input voltage (V)',ylabel='MCU voltage (V)',xlim=(-30,30))
    for a in ax: a.grid(alpha=.25)
    fig.tight_layout(); fig.savefig(ROOT/'figures/input_dc.png',dpi=160); plt.close(fig)
    delays = []
    fig,ax = plt.subplots(2,1,figsize=(10,7),sharex=True)
    for ctr in (1,4):
        d,r = transient(ctr); delays.append(r)
        csv_save(ROOT/f'out/step_ctr{ctr}.csv',[dict(time_s=float(t),input_v=float(i),output_v=float(o)) for t,i,o in zip(d['time'],d['inp'],d['out'])])
        ax[1].plot(d['time']*1000,d['out'],label=f'CTR {ctr*100}%')
    ax[0].plot(d['time']*1000,d['inp']); ax[0].set(ylabel='Input (V)')
    ax[1].set(xlabel='Time (ms)',ylabel='MCU (V)'); ax[1].legend()
    for a in ax: a.grid(alpha=.25)
    fig.tight_layout(); fig.savefig(ROOT/'figures/input_step.png',dpi=160); plt.close(fig)
    brake = brake_corners()
    csv_save(ROOT/'brake/corners.csv',brake)
    fig,ax = plt.subplots(3,1,figsize=(10,10),gridspec_kw={'height_ratios':[2,1.2,1]})
    supply_energy = {}
    for kind in 'abc':
        d,r = brake_run(kind)
        csv_save(ROOT/f'out/waveform_{kind}.csv',[dict(time_s=float(t),coil_a=float(i),drain_v=float(o),gate_v=float(g)) for t,i,o,g in zip(d['time'],d['lcoil#branch'],d['drain'],d['gate'])])
        for a,key in zip(ax[:2],('lcoil#branch','drain')):
            a.plot((d['time']-OFF)*1000,d[key],label=f'Clamp ({kind})')
        ax[2].plot((d['time']-OFF)*1000,d['drain'],label=f'Clamp ({kind})')
        ax[0].plot(r['delay_s']*1000,.3*r['held_a'],'o',color=ax[0].lines[-1].get_color())
        ax[0].annotate(f"({kind}) {r['delay_s']*1000:.2f} ms",(r['delay_s']*1000,.3*r['held_a']),xytext=(0,12 if kind != 'c' else -20),textcoords='offset points',ha='center',fontsize=8)
        t, current = d['time'], d['lcoil#branch']
        end = np.flatnonzero((t>OFF) & (current<=0))[0]
        interval = (t>=OFF) & (t<=t[end])
        supply_energy[kind] = float(np.trapezoid(-d['vs#branch'][interval]*d['vs'][interval],t[interval]))
    ax[0].axhline(.3*r['held_a'],color='gray',ls='--',label='30% held current')
    ax[0].set(ylabel='Coil current (A)',title='24 V turn-off; same time axis for all three clamps')
    ax[0].set_xlim(-1,12)
    ax[1].set(ylabel='Drain voltage (V)',xlabel='Time since gate-off command (ms)',xlim=(-1,12))
    ax[2].set(ylabel='Drain voltage (V)',xlabel='Time since gate-off command (ms)',xlim=(5,25),title='Post-current drain ringing')
    for a in ax: a.grid(alpha=.25); a.legend()
    fig.tight_layout(); fig.savefig(ROOT/'figures/brake_turnoff.png',dpi=160); plt.close(fig)
    resets = [brake_run(k,26.4,1,True,1,1.01,1.01)[1] for k in 'abc']
    csv_save(ROOT/'brake/reset.csv',resets)
    draw_all()
    bound = lambda key: (min(float(r[key]) for r in rows),max(float(r[key]) for r in rows))
    sections = ['# Simulation results',
        'Generated by `python hw/run_all.py` from fresh ngspice simulations. Do not edit tables manually.',
        'These are conditional model-envelope results at 25 C, not certified IEC compliance or hardware timing guarantees. See DESIGN.md for source test conditions and model limits.',
        '## Input: all 128 independent corners',
        f'Guaranteed within this model: ON at or above {max(r["on"] for r in rows):.2f} V; OFF at or below {min(r["off"] for r in rows):.2f} V (0.02 V grid, conservative sampled boundaries). Logic LOW <=0.99 V means ON; HIGH >=2.31 V means OFF.',
        table(['Metric','Minimum','Maximum'],[[k,f'{bound(k)[0]*scale:.6g}',f'{bound(k)[1]*scale:.6g}'] for k,scale in [('i11',1000),('i30',1000),('leakage_out',1),('reverse_led',1),('reverse_i',1),('reverse_d1',1)]]),
        'i11 and i30 are mA; leakage_out is MCU V at 1.5 mA forced sensor leakage; reverse currents are A and reverse_d1 is V at -30 V.',
        table(['Part at 30 V','Worst power (W)','Rating (W)','Rating / power'],[[k,f'{bound(k)[1]:.6f}',rating,f'{rating/bound(k)[1]:.2f}'] for k,rating in [('r1_w',.5),('r2_w',.5),('r3_w',.5),('r4_w',.5),('z_w',.5)]]),
        '## Input propagation from 50% input edge to guaranteed MCU logic level',
        table(['CTR (%)','ON (us)','OFF (us)'],[[r['ctr']*100,f'{r["on_s"]*1e6:.3f}',f'{r["off_s"]*1e6:.3f}'] for r in delays]),
        '## Brake nominal parts at each supply',
        'Energy integrates all clamp branch power from gate-off to the end of the simulation, including the series diode in (b). The live supply remains present. Drop distance includes electrical delay only.',
        f"At 24 V, supply energy from gate-off to the first coil-current zero is {supply_energy['b']*1000:.3f} mJ for (b) and {supply_energy['c']*1000:.3f} mJ for (c); in (c) the supply drives coil current into the drain clamp, increasing its absorbed energy.",
        'After coil current first reaches zero, the 0.6 H coil resonates with the MOSFET output capacitances; the ideal clamp diode models have no explicit junction capacitance. Using 180 pF Cds and 170 pF Cgd gives an approximate 11.0 kHz resonance; the model omits core loss, and the sub-clamp-voltage ringing may decay more slowly than in a real coil, where an RC snubber across the coil could damp it.',
        table(['Clamp','Supply (V)','Held (A)','Delay (ms)','Peak drain (V)','Clamp energy (mJ)','Peak clamp (W)','Fall (mm)'],
              [[r['kind'],r['supply'],f'{r["held_a"]:.5f}',f'{r["delay_s"]*1000:.4f}',f'{r["peak_v"]:.3f}',f'{r["energy_j"]*1000:.3f}',f'{r["clamp_peak_w"]:.3f}',f'{r["fall_mm"]:.5f}'] for r in brake if r['corner']==0]),
        f'## Brake envelope: {len(brake)} simulations',
        'Each supply has nominal parts plus independent TVS breakdown endpoints, MOS threshold endpoints, and both gate-resistor/pull-down tolerance endpoints. Coil R/L and capacitances are fixed assumptions.',
        table(['Clamp','Supply','Delay min/max (ms)','Max drain (V)','Max energy (mJ)','Max fall (mm)'],
              [[k,vv,f'{min(r["delay_s"] for r in group)*1000:.4f} / {max(r["delay_s"] for r in group)*1000:.4f}',f'{max(r["peak_v"] for r in group):.3f}',f'{max(r["energy_j"] for r in group)*1000:.3f}',f'{max(r["fall_mm"] for r in group):.5f}'] for k in 'abc' for vv in (21.6,24,26.4) for group in [[r for r in brake if r['kind']==k and r['supply']==vv]]]),
        '## Single-pulse clamp rating margins',
        table(['Clamp','Peak branch power (W)','Conservative allowed power (W)','Power margin','Max energy (mJ)'],
              [[k,f'{max(r["clamp_peak_w"] for r in group):.3f}',limit,f'{limit/max(r["clamp_peak_w"] for r in group):.2f}x',f'{max(r["energy_j"] for r in group)*1000:.3f}'] for k,limit in [('a',1.1),('b',100),('c',100)] for group in [[r for r in brake if r['kind']==k]]]),
        'For (a), 1.1 W = 1 A x 1.1 V rectifier reference; the relevant current margin is >2.7x. For TVS clamps, 100 W over 10 ms is a conservative single-pulse design envelope from the pulse-duration curve at 25 C; 1 J rectangular envelope exceeds simulated energy. This is not a repetitive energy rating. Check pad area, temperature and repetition rate in hardware.',
        '## Reset: MCU pin disconnected, 26.4 V, high TVS/Vth and +1% resistors',
        table(['Clamp','Delay (ms)','Peak drain (V)','Final gate (V)','Final coil current (A)'],[[r['kind'],f'{r["delay_s"]*1000:.4f}',f'{r["peak_v"]:.3f}',f'{r["gate_end_v"]:.3g}',f'{r["current_end_a"]:.3g}'] for r in resets]),
        '## Limits',
        'Temperature, coil heating, mechanical brake travel, ageing, wiring parasitics, EMC/surge tests, isolation layout, and production distributions are not modelled. A back-driven ball screw with reflected inertia falls more slowly than free fall. No mechanical engage time is assumed; the electrical delay is only part of the total. PWM hold reduction was intentionally skipped.']
    (ROOT/'RESULTS.md').write_text('\n\n'.join(sections)+'\n',encoding='utf-8')
    summary = dict(input_on_v=max(r['on'] for r in rows),input_off_v=min(r['off'] for r in rows),input_i11_ma=[x*1000 for x in bound('i11')],input_i30_ma=[x*1000 for x in bound('i30')],input_delays=delays,brake_runs=len(brake))
    (ROOT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    import hw.spice as spice_module
    engine = spice_module._engine
    engine.log.clear()
    engine.lib.ngSpice_Command(b'version')
    print('DLL: '+str(engine.path))
    print('\n'.join(engine.log))
    print(json.dumps(summary,indent=2))
    print('Regenerated RESULTS.md, CSVs, netlists and all figures.')

if __name__ == '__main__':
    main()
