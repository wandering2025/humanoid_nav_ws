"""Reject models that violate the frozen upstream constructor/slice contract."""
import numpy as np
import mujoco


class ModelIncompatible(ValueError):
    pass


def joint_map(model, names):
    def require(condition, message):
        if not condition:
            raise ModelIncompatible(message)
    require((model.nq, model.nv, model.nu, model.njnt) == (30, 29, 23, 24),
            'expected nq/nv/nu/njnt=30/29/23/24; extra degrees of freedom are unsupported')
    try:
        root = model.joint('world_joint').id
        require(model.jnt_type[root] == mujoco.mjtJoint.mjJNT_FREE and
                model.jnt_bodyid[root] == model.body('trunk').id and
                model.jnt_qposadr[root] == 0 and model.jnt_dofadr[root] == 0,
                'world_joint must be the trunk free joint at address zero')
        rows = []
        require(len(names) == 23 and len(set(names)) == 23, 'expected 23 unique joint names')
        for i, name in enumerate(names):
            j = model.joint(name).id
            actuators = np.flatnonzero(model.actuator_trnid[:, 0] == j)
            require(model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE, f'{name}: not hinge')
            require(model.jnt_qposadr[j] == i + 7 and model.jnt_dofadr[j] == i + 6,
                    f'{name}: order incompatible with upstream initialization')
            require(actuators.tolist() == [i], f'{name}: actuator order mismatch')
            require(model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT and
                    np.array_equal(model.actuator_gear[i], [1, 0, 0, 0, 0, 0]),
                    f'{name}: expected unit-gear joint actuator')
            rows.append((i + 7, i + 6, i))
        return np.asarray(rows, dtype=int).T
    except KeyError as exc:
        raise ModelIncompatible(str(exc)) from exc
