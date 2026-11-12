"""Schemdraw physical schematics; virtual SPICE fit elements are described in DESIGN."""
from pathlib import Path
import schemdraw
import schemdraw.elements as e

OUT = Path(__file__).resolve().parent/'figures'

def save(d,name):
    d.save(str(OUT/f'{name}.svg'))
    d.save(str(OUT/f'{name}.png'),dpi=160)

def draw_all():
    with schemdraw.Drawing(show=False) as d:
        d.config(fontsize=10,unit=2)
        d += e.Dot().at((0,0)).label('PNP input',loc='left')
        d += e.Diode().right().length(2).label('D1 1N4007')
        d += e.Resistor().right().length(2.2).label('R1 887 ohm')
        d += e.Resistor().right().length(2.2).label('R2 887 ohm')
        d += e.Zener().left().at((8.6,0)).length(2.2).label('DZ1\nBZX55C2V7')
        opt = e.Optocoupler(box=False).right().at((9.4,0)).anchor('anode')
        d += opt
        for start, end in [((9.1,0.4),(12,0.4)), ((12,0.4),(12,-1.8)),
                           ((12,-1.8),(9.1,-1.8)), ((9.1,-1.8),(9.1,0.4))]:
            d += e.Line().at(start).to(end).linestyle('--')
        d += e.Line().at((10.75,-1.65)).to((10.75,0.15)).linestyle('--')
        d += e.Label().at((10.5,1)).label('U1 TLP291 (GB)')
        d += e.Line().at((8.6,0)).to(opt.anode)
        d += e.Dot().at((8.6,0))
        d += e.Resistor().at((8.6,0)).down().length(3)
        d += e.Label().at((7,-1.5)).label('R3 453 ohm')
        d += e.Line().at(opt.cathode).to((9.4,-3))
        d += e.Line().at((9.4,-3)).to((8.6,-3))
        d += e.Ground().at((8.6,-3)).label('Field 0 V',loc='bottom')
        cx,cy = opt.collector
        d += e.Resistor().at((cx,cy)).up().length(2.3)
        d += e.Label().at((13.3,1.3)).label('R4 10 kohm')
        d += e.Dot().label('+3.3 V',loc='top')
        d += e.Line().at(opt.collector).right().length(2).label('MCU input\nLOW = ON',loc='right')
        d += e.Dot().at(opt.collector)
        d += e.Line().at(opt.emitter).down().length(0.8)
        d += e.Ground().label('MCU GND',loc='bottom')
        d += e.Label().at((3,-3)).label('R1-R4: 1%, 0.5 W\nIsolated field and MCU returns')
        save(d,'io24_schematic')
    for kind in 'abc':
        with schemdraw.Drawing(show=False) as d:
            d.config(fontsize=10,unit=2)
            d += e.Dot().at((6,5)).label('+24 V +/-10%',loc='top')
            d += e.Resistor().down().length(2).label('Rcoil 72 ohm')
            d += e.Inductor().down().length(2).label('Lcoil 0.6 H')
            mos = e.NMos().right().at((6,1)).anchor('drain').label('M1 IRL540N\n100 V',loc='right')
            d += mos
            d += e.Ground().at(mos.source)
            gx,gy = mos.gate
            d += e.Resistor().at((0,gy)).right().to((gx,gy)).label('Rg 330 ohm')
            d += e.Dot().at((0,gy)).label('MCU 3.3 V',loc='left')
            d += e.Dot().at((gx,gy))
            d += e.Resistor().at((gx,gy)).down().length(2.5)
            d += e.Label().at((3.4,-1.4)).label('Rpd 10 kohm')
            d += e.Ground()
            d += e.Line().at((6,1)).to((10,1))
            d += e.Dot().at((6,1))
            if kind in 'ab':
                d += e.Line().at((6,5)).to((10,5))
                d += e.Diode().at((10,1)).up().length(4 if kind=='a' else 2)
                d += e.Label().at((12.2,3 if kind=='a' else 2)).label('D2 1N4007')
                if kind=='b':
                    d += e.Zener().at((10,5)).down().length(2)
                    d += e.Label().at((12.2,4)).label('DZ2 SMBJ24A')
            else:
                d += e.Zener().at((10,-2)).up().length(3)
                d += e.Label().at((12.2,-.5)).label('DZ3 SMBJ43A')
                d += e.Ground().at((10,-2))
            d += e.Label().at((3,5)).label(f'Clamp ({kind})\nSpring applied brake\nPower ON = released')
            d += e.Label().at((2,-3)).label('Rg, Rpd: 1%, 0.5 W\nReset leaves MCU pin floating')
            save(d,f'brake_{kind}_schematic')
