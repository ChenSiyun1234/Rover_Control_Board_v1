"""Fast selected-fixture integration checks against the unchanged controller DLL."""

import math

import mujoco as mj
import numpy as np
import pytest

from sim.params import Params
from sim.sensors import Sensors
from sim.trace import TraceColumn as T
from sim3d.cases import run_case
from sim3d.mj_plant import MjPlant
from sim3d.scene import ARM


def test_rack_rest_and_static_rover():
    plant = MjPlant(Params(), case="rack")
    for _ in range(1000):
        plant.step(0, False)
    np.testing.assert_array_equal(plant.data.qpos, [0, 0])
    np.testing.assert_array_equal(plant.data.qvel, [0, 0])
    assert plant.force == plant.current == plant.jam_force() == 0
    assert plant.model.njnt == 2
    assert plant.model.opt.timestep == 1e-4
    assert plant.model.body("control_board").id > 0
    assert all(plant.model.mesh(f"board_c{i:02}").id >= 0 for i in range(12))
    assert plant.model.nhfield == 0
    assert plant.model.camera("overview").id >= 0
    assert plant.model.camera("rack_view").id >= 0
    assert len(plant.qpos_log) == 101
    assert plant.qpos_log[-1][0] == pytest.approx(.1)


@pytest.mark.parametrize("case", ["rack", "arm"])
def test_nominal_seats(case):
    sim, result = run_case(f"{case}_nominal")
    assert result["ok"], result
    assert result["fault_flags"] == 0
    assert result["seat_position_mm"] == pytest.approx(295 if case == "rack" else 40, abs=.2)
    log = sim.plant.qpos_log
    assert len(log) == sim.now_ms+1
    np.testing.assert_allclose(log[-1][1], sim.plant.data.qpos)
    assert sim.plant.data.time == pytest.approx(sim.now_ms/1000)
    if case == "arm":
        assert sim.plant.counts_per_m == pytest.approx(8192*100/(2*math.pi*ARM.radius))
        assert Sensors(Params()).encoder(sim.plant) == round(
            sim.plant.carriage_position_m*ARM.counts_per_m)
        # Verify the physical static patch stiffness, including both native
        # contact points and MuJoCo's impedance regularization.
        d = sim.plant.data
        patch_k = sum(d.efc_KBIP[c.efc_address, 0]*d.efc_KBIP[c.efc_address, 2]
                      / d.efc_R[c.efc_address] for c in d.contact)
        assert patch_k == pytest.approx(ARM.contact_k, rel=.01)
        normal = np.zeros(6)
        force = 0
        for i in range(d.ncon):
            mj.mj_contactForce(sim.plant.model, d, i, normal)
            force += normal[0]
        assert sim.plant.force == pytest.approx(force)


@pytest.mark.parametrize("case", ["rack", "arm"])
def test_jam_trips_and_backs_off(case):
    sim, result = run_case(f"{case}_jam")
    assert result["ok"], result
    assert result["fault_flags"] == 1
    assert result["backoff_mm"] == pytest.approx(2, abs=.15)
    assert result["residual_force_N"] < 1
    assert sim.plant.jam_force() >= 0
    assert np.max(sim.trace[:, T.RACK_FORCE_N]) > 0
    trip_row = sim.trace[result["fault_ms"]-1]
    assert trip_row[T.MEASURED_FORCE_N] == pytest.approx(result["detection_force_N"])
