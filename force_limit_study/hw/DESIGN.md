# 24 V axis I/O design

## Scope and requirements

Low MCU output means sensor ON. The field return and MCU ground are isolated in hardware; their DC references share SPICE node 0 only to simplify numerical analysis. No isolation-barrier layout or certification is implied.

Type 3 limits are taken from Analog Devices, [Industrial Digital Inputs with the MAX22191](https://www.analog.com/en/resources/design-notes/industrial-digital-inputs-with-the-max22191.html), section **Types of Digital Inputs**, Table 1, and cross-checked against Beckhoff [DK9222-0909-0008](https://download.beckhoff.com/download/Document/Application_Notes/DK9222-0909-0008.pdf), page 3, Figures 2 and 3. For 24 V sinking Type 3 inputs: ON is 11-30 V and 2-15 mA; transition is 5-11 V and 1.5-15 mA. OFF is the union of the low-voltage arm (-3 to 5 V, up to 15 mA) and the low-current arm (up to 1.5 mA through 11 V). These are two-dimensional regions; the table's OFF upper value of 11 V must not be interpreted as OFF at arbitrary current through 11 V. This circuit additionally studies -30 V reverse polarity.

A two-wire sensor can deliver current while OFF. A voltage threshold alone does not reject this leakage: the receiving circuit must consume the leakage without enough LED emission to pull the MCU pin low. We simulate a forced 1.5 mA point on every corner's I-V curve, as well as the voltage/current operating band. The plot draws both arms of the OFF region.

The MCU is assumed to have VIL(max)=0.3 VDD=0.99 V and VIH(min)=0.7 VDD=2.31 V, VDD exactly 3.3 V, negligible input leakage, and 30 pF total collector/pin/wiring capacitance. No specific MCU is claimed.

## Parts and sources

| Reference | Selection | Datasheet and relevant section |
| --- | --- | --- |
| U1 / DLED | Toshiba TLP291, **GB rank**, SO4 | [TLP291](https://toshiba-semicon-storage.com/info/TLP291_datasheet_en_20190527.pdf?did=12884&prodName=TLP291), pages 2-4: GB CTR, LED, coupled and switching characteristics |
| D1, D2 | Vishay 1N4007, 1 A, 1000 V | [1N4001-1N4007](https://www.vishay.com/docs/88503/1n4001.pdf), pages 1-2, maximum ratings/electrical characteristics |
| DZ1 | Vishay BZX55C2V7, nominal 2.7 V, 0.5 W | [BZX55](https://www.vishay.com/docs/85604/bzx55.pdf), maximum ratings and electrical characteristics: 2.5-2.9 V at 5 mA |
| R1, R2 | Vishay CRCW1210, 887 ohm, 1%, 0.5 W each | [D/CRCW e3](https://www.vishay.com/docs/20035/dcrcwe3.pdf), technical specifications, CRCW1210 size |
| R3, R4 | Same resistor series: 453 ohm and 10 kohm, 1%, 0.5 W | Same resistor datasheet |
| M1 | Infineon IRL540NPbF, 100 V, logic-level TO220 | [IRL540N](https://www.infineon.com/dgdl/Infineon-IRL540N-DataSheet-v01_01-EN.pdf?fileId=5546d462533600a40153565fc2a62567), pages 1-2, absolute ratings and electrical characteristics |
| Rg, Rpd | CRCW1210, 330 ohm and 10 kohm, 1%, 0.5 W | Same resistor datasheet |
| DZ2 | Vishay SMBJ24A, unidirectional TVS used as avalanche Zener across coil, in series with D2 | [SMBJ](https://www.vishay.com/docs/88392/smbj.pdf), pages 1-2 and page 4 Figures 1-3 |
| DZ3 | Vishay SMBJ43A, unidirectional TVS across drain-source | Same SMBJ datasheet |
| Rcoil, Lcoil | Assumed brake winding, 72 ohm and 0.6 H | No physical brake part selected |

All resistors operate below 50% of their 0.5 W rating in the evaluated conditions. Capacitors, current sources and the reset switch in netlists are model/test-fixture elements, not extra physical BOM parts. The schematics show the physical circuits; U1's LED and detector are expanded in SPICE.

## Input sizing and hand calculations

U1 GB has CTR 100-400% at IF=5 mA, VCE=5 V, 25 C. LED VF is 1.1-1.4 V at 10 mA. GB also specifies at least 30% saturated CTR at IF=1 mA, VCE=0.4 V. These are different operating points. The simulation applies 100-400% across current and uses a simple saturation taper; it does **not** turn those test-point limits into a guaranteed CTR curve. That extrapolation is an explicit model assumption. Consequently, 'guaranteed ON/OFF' in RESULTS means guaranteed over the stated simulation envelope, not a production hardware guarantee.

R3 <= 0.7 V / 1.5 mA = 466.7 ohm. Select 453 ohm, 1%; its maximum 457.53 ohm gives at most 0.6863 V from 1.5 mA even if all leakage goes through R3. U1's OFF collector-current limit at VF=0.7 V is 10 uA; a 10 kohm pull-up loses only 0.10 V at that limit. The model includes a conservative fixed 10 uA output leakage in addition to LED-dependent sink current.

R4 is specified as 10 kohm. At VIL=0.99 V it supplies (3.3-0.99)/10000 = 0.231 mA; its -1% limit requires 0.2333 mA. At a 0.3 V saturated collector it supplies 0.30 mA nominal. The shunt and LED must be included together when computing the required field current.

DZ1 is 2.7 V, rather than a larger threshold Zener, because R3 consumes several mA around LED turn-on. The total series resistance is R1+R2=1774 ohm. A screening bound at 11 V using diode 0.8 V, Zener 2.9 V and LED 1.4 V gives (11-0.8-2.9-1.4)/(1774*1.01)=3.293 mA. R3 can consume 1.4/(453*.99)=3.122 mA in this deliberately constant-VF bound. This bound alone is insufficient to prove ON; the diode I-V curve at low LED current and the CTR assumption are material, and ngspice evaluates them explicitly. This is why no unconditional hardware compliance claim is made.

At 30 V, using 0.6 V rectifier drop, 2.5 V Zener and 1.1 V LED gives (30-0.6-2.5-1.1)/(1774*.99)=14.69 mA, below 15 mA. Each 887 ohm resistor dissipates about (14.69 mA)^2*887=0.191 W. Splitting the series resistance yields comfortable voltage stress and <0.25 W per part. The simulated dissipations, including independent resistor extremes, are in RESULTS. R3's loose screening power bound is 1.4^2/(453*.99)=4.37 mW; R4's is 3.3^2/9900=1.10 mW. DZ1 is approximately 2.9*14.69 mA=42.6 mW (its model includes dynamic resistance, so use the actual simulated maximum). D1 sees approximately 30 V reverse, far below 1000 V. At its datasheet 5 uA maximum reverse leakage the shunt drop is under 2.3 mV and series resistor loss is negligible; the simulated nominal reverse leakage is much lower. LED reverse limit is 5 V and is protected by D1 and R3.

The 128 corners are the Cartesian product of independent R1/R2/R3/R4 +/-1%, DZ1 2.5/2.9 V, LED 1.1/1.4 V calibrated at 10 mA, and CTR 1/4. No Monte Carlo is needed. DC sweep resolution is 20 mV from -30 to +30 V; ON is the maximum first sampled Vout<=0.99 V, OFF is the minimum last sampled Vout>=2.31 V. Monotonicity is assumed by this passive threshold circuit and checked by the swept band. The current limit is checked across the entire 11-30 V interval.

The LED uses an exponential diode with N=2 and Is=10 mA/(exp(VF/(2*25.85 mV))-1). DZ1 uses BV at 5 mA and an assumed 10 ohm series resistance. Its soft knee at currents below the test point, LED slope, and rectifier Is=1 nA/N=1.6/Rs=0.1 ohm are simplified fits, not vendor worst-case models. Temperature is not modelled. Photocurrent is generated with a CCCS; a 1 kohm transimpedance and 1 kohm/7 nF low-pass give an assumed 14 us response constant (both 1 kohm resistances contribute). The output sink tapers toward a 0.2 V saturation floor over a 10 mV scale. This is not a charge-storage model: actual turn-off under saturation may differ substantially. Datasheet timing is typical under other loads; our 10 kohm pull-up delays are illustrative, not guaranteed maxima.

The nominal simulated field input draws 260.6 mW at 24 V and 426.0 mW at 30 V per active channel. For comparison, the [ISO1211 datasheet](https://www.ti.com/lit/ds/symlink/iso1211.pdf), section 8.2.1.2.1, specifies a typical 2.25 mA limit with 562 ohm Rsense for Type 3; this corresponds to approximately 54.0 mW at 24 V and 67.5 mW at 30 V per channel at the field input, including external resistors and excluding logic-side power. This lower field-side heat favors a current-limited receiver on a many-channel board.

Integrated receivers such as [TI ISO1211](https://www.ti.com/lit/ds/symlink/iso1211.pdf), section 8.1 and Figure 8-1, integrate isolation, thresholding and current limiting with fewer external parts. For a product I would select ISO1211: its documented current-limited Type 3 behavior avoids the discrete circuit's CTR/knee qualification burden and reduces heat. This discrete design is useful for understanding and bench qualification.

The SPICE input model shares node 0 between field and MCU sides for numerical convenience; it does not simulate isolation. The physical schematic has separate Field 0 V and MCU GND returns.

## Brake calculations and models

The assumed brake has 24 V, 8 W, R=24^2/8=72 ohm, L=0.6 H. Held currents without switch loss are 0.300/0.333/0.367 A at 21.6/24/26.4 V. Stored energy is L*I^2/2=27.0/33.3/40.3 mJ. Drop-out is 30% of the actual simulated held current immediately before turn-off. The circuit starts energized at its DC operating point, so it represents an already-released brake, not startup from zero current.

No downloadable vendor plain-device SPICE model was located on the consulted IRL540N pages/search. This study therefore uses the requested level-1 fitting alternative; it does not claim a vendor model was found incompatible. Fit Rds(Vgs)=Rd+1/[Kp*(Vgs-Vto)] to the datasheet 63 milliohm at 4 V and 44 milliohm at 10 V. For nominal Vto=1.5 V: 1/Kp=(.063-.044)/(1/2.5-1/8.5)=0.06729, Kp=14.86 A/V^2, Rd=0.03608 ohm. Repeat the fit independently at Vto=1 and 2 V. Ciss=1800 pF, Crss=170 pF and Coss=350 pF give Cgs=1630 pF, Cgd=170 pF, Cds=180 pF. These fixed capacitances are typical values at one bias. The model extrapolates resistance to 3.3 V; Rds is not guaranteed by the manufacturer at 3.3 V. This is a bench qualification item, even though 0.37 A is a light load.

Rg=330 ohm limits an ideal 3.3 V step to 10 mA; its worst possible pulse power is 3.3^2/330=33 mW. Rpd=10 kohm gives Vgate=3.3*10000/(10000+330)=3.195 V while driven high; it consumes 1.02 mW. Nominal reset RC is 10 kohm*1.8 nF=18 us, with additional Miller charge during drain rise. The reset fixture opens a switch between the still-high source and MCU pin; the MCU pin is truly disconnected, not driven low. Rpd must remove the charge. No pull-down can protect against a shorted MOSFET.

For (a), D2 has anode at drain, cathode at supply. For (b), D2 anode is at drain and cathode at the TVS cathode; DZ2 anode is at supply. For (c), DZ3 cathode is at drain, anode at source/ground. Never place a plain diode forward across the energized coil or reverse the avalanche clamp connection.

For an approximately constant opposing coil voltage Vc, di/dt=-(R*i+Vc)/L and t30=(L/R)*ln[(R*I0+Vc)/(0.3*R*I0+Vc)]. At 24 V, Vc approximately 0.85 V gives 9.4 ms for (a); about 29.3 V gives 3.16 ms for (b). For (c), Vc is Vdrain-Vs, about 51.1-24=27.1 V, giving 3.34 ms. The different supply reference explains why a drain TVS with a larger nominal voltage can still decay more slowly than a lower-voltage coil TVS.

SMBJ24A has breakdown 26.7-29.5 V at 1 mA, maximum 38.9 V at 15.4 A. SMBJ43A has breakdown 47.8-52.8 V at 1 mA, maximum 69.4 V at 8.6 A. Their linear-fit resistances are (38.9-29.5)/15.4=0.6104 ohm and (69.4-52.8)/8.6=1.9302 ohm. The fit includes an exponential avalanche knee and those resistances. Maximum datasheet screening drain voltages are 26.4+38.9+1.1=66.4 V for (b) and 69.4 V for (c), both below 80 V (80% of M1's 100 V rating). Real wiring overshoot is not modelled.

The brake suite has 9 nominal supply/clamp cases and 144 endpoint cases: three clamps, three supplies, independent two-point TVS breakdown, Vto, Rg and Rpd. The diode clamp has no TVS, so some of its cases intentionally duplicate the TVS dimension. Coil R/L have no given tolerances and remain exact assumed values. TVS dynamic resistance and capacitance tolerances are not specified as production bounds and are not swept.

Clamp energy is the numerical integral of positive branch voltage times branch current, from 50 ms gate-off through 120 ms. For (b) the branch includes both diode and TVS; this conservatively assigns their combined loss to the TVS rating. Coil resistance absorbs the remaining energy in (a)/(b). In (c), the live supply continues supplying energy during current decay, so clamp energy may exceed the initial magnetic energy. A 600 W 10/1000 us label is not a generic joule rating. We use a conservative 100 W / 10 ms single-pulse envelope below the SMBJ page 4 Figure 1 curve at 25 C, corresponding to a 1 J rectangular screening envelope; results report peak-power and energy margins. This assumes the stated copper pads and a cool isolated pulse. Repetition rate and hot thermal derating need separate qualification. D2 has >2.7x current margin to its 1 A rating at the largest held current.

After coil current first reaches zero, the 0.6 H coil resonates with the MOSFET output capacitances; the clamp diode models have no explicit junction capacitance. Using Cds = 180 pF and Cgd = 170 pF gives an approximate 11.0 kHz resonance; the model omits core loss, and the sub-clamp-voltage ringing may decay more slowly than in a real coil, where an RC snubber across the coil could damp it.

## Vertical-axis interpretation

If torque disappears at gate-off, the electrical unsupported interval has a free-fall upper bound d=0.5*9.80665*t30^2. Each case is tabulated. A back-driven ball screw with reflected inertia falls more slowly. The simulation keeps the supply live, which is relevant to a safety stop and a conservative electrical timing scenario relative to an ideal bus forced to zero for the drain-clamped case; it does not simulate every power-loss supply impedance. No mechanical engage time was found for the assumed R/L winding, so none is invented. Mechanical motion and brake torque buildup add to electrical delay; the tabulated distance is only the electrical interval's free-fall bound, not a total stopping-distance guarantee.

## Assumptions, deviations and unchecked items

- Brake: assumed 72 ohm, 0.6 H, 8 W at 24 V, fixed R/L, 30% current threshold; supply 21.6/24/26.4 V; initially held at DC equilibrium; no mechanical delay assumed.
- MCU: ideal 3.3 V, 0.99/2.31 V logic limits, negligible leakage, 30 pF total pin/output capacitance; ideal command edges with 100 ns ramp; driver source resistance represented by Rg only.
- Input model: 25 C only; LED N=2, datasheet VF endpoints at 10 mA extrapolated; constant GB CTR 100-400%; 10 uA detector leakage; 0.2 V saturation floor and 10 mV taper; 14 us low-pass time constant; no saturation charge storage.
- Zener model: datasheet voltage endpoints at test current, 10 ohm assumed series resistance for DZ1; exponential knee is a fit. Rectifiers: Is=1 nA, N=1.6, Rs=0.1 ohm, BV=1000 V, IBV=5 uA; no reverse recovery or production forward-voltage spread.
- MOS model: level-1 fit with Vto=1/1.5/2 V, Rds endpoints at 4/10 V, extrapolated to 3.3 V; fixed 1800/350/170 pF capacitances; no temperature, avalanche self-heating or package/wire inductance. No suitable vendor model was located; using the fit without asserting incompatibility is the model-selection deviation.
- TVS: endpoint breakdown with fixed fitted resistance, no production curve/temperature guarantee; conservative 100 W / 10 ms single-pulse envelope at 25 C with specified mounting; no repetition rate assumed.
- Transient timing is from command/input 50% edge to the stated logic/current threshold. Input corner thresholds use a 20 mV sweep grid; integration uses actual adaptive ngspice time points. Brake max step is 10 us; input max step is 0.2 us.
- Gravity 9.80665 m/s^2; zero initial downward velocity for the electrical free-fall calculation; mechanical brake delay omitted, not zero in the real machine.
- The delivered checks establish the stated SPICE envelope, not full hardware IEC compliance. Low-current/saturated CTR, Zener knee bounds, 3.3 V MOS conduction and timing require bench/vendor qualification. No physical build, insulation/EMC/surge, thermal, mechanical or safety acceptance tests were performed. Optional PWM hold reduction was skipped.
