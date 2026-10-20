"""Dialog to create / edit a motor preset. Kept apart from main.py so the logic in main.py can be tested without a display."""
from PySide6 import QtWidgets

import presets as PR


def run_preset_dialog(parent, initial=None):
    """Return a preset dict (line-line values, firmware units) or None if cancelled."""
    init = dict(initial or {})
    dlg = QtWidgets.QDialog(parent)
    dlg.setWindowTitle("Motor preset" if initial else "New motor preset")
    lay = QtWidgets.QVBoxLayout(dlg)
    form = QtWidgets.QFormLayout()
    lay.addLayout(form)

    name = QtWidgets.QLineEdit(init.get("name", ""))
    name.setMaxLength(PR.NAME_MAX)
    name.setToolTip("Up to %d characters, letters/digits/spaces. Stored on the board as the 'current preset'." % PR.NAME_MAX)
    form.addRow("Name:", name)

    mode_pn = QtWidgets.QRadioButton("Datasheet gives phase-to-neutral values (R and L are doubled)")
    mode_ll = QtWidgets.QRadioButton("Values are line-to-line (as measured between two motor wires)")
    mode_ll.setChecked(True)
    lay.insertWidget(0, mode_ll)
    lay.insertWidget(0, mode_pn)

    def dspin(lo, hi, val, dec, suffix="", step=None):
        s = QtWidgets.QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(dec)
        s.setValue(val)
        if suffix:
            s.setSuffix(" " + suffix)
        if step:
            s.setSingleStep(step)
        return s

    pp = QtWidgets.QSpinBox()
    pp.setRange(1, 50)
    pp.setValue(int(init.get("pole_pairs", 7)))
    kv = dspin(1, 5000, float(init.get("kv", 100)), 2, "rpm/V")
    r = dspin(1, 10000, float(init.get("r_ll_mohm", 100)), 2, "mOhm")
    l_ = dspin(0, 10000, float(init.get("l_ll_uh", 50)), 2, "uH")
    kt = dspin(0.001, 10, float(init.get("kt_nm_a", 8.27 / float(init.get("kv", 100)))), 4, "Nm/A", 0.001)
    rmax = dspin(1, 20000, float(init.get("rpm_max", 1500)), 0, "rpm", 10)
    ilim = dspin(0.5, 100, float(init.get("ilim", 8)), 1, "A", 0.5)
    idr = dspin(0.1, 100, float(init.get("i_drive", 3)), 2, "A", 0.5)
    acc = dspin(1, 10000, float(init.get("accel", 60)), 0, "rpm/s", 10)
    form.addRow("Pole pairs:", pp)
    form.addRow("Kv:", kv)
    form.addRow("Resistance:", r)
    form.addRow("Inductance:", l_)
    form.addRow("Torque constant:", kt)
    form.addRow("Speed limit:", rmax)
    form.addRow("Overcurrent trip:", ilim)
    form.addRow("Drive current:", idr)
    form.addRow("Acceleration:", acc)
    notes = QtWidgets.QLineEdit(init.get("notes", ""))
    form.addRow("Notes:", notes)
    hint = QtWidgets.QLabel("Kv is rpm per volt (peak line-line). Kt defaults to 8.27 / Kv if unknown. "
                            "Start with a low overcurrent trip and drive current for a new motor.")
    hint.setWordWrap(True)
    lay.addWidget(hint)

    buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Ok |
                                         QtWidgets.QDialogButtonBox.StandardButton.Cancel)
    buttons.accepted.connect(dlg.accept)
    buttons.rejected.connect(dlg.reject)
    lay.addWidget(buttons)

    while dlg.exec() == QtWidgets.QDialog.DialogCode.Accepted:
        k = 2.0 if mode_pn.isChecked() else 1.0
        d = {"name": name.text().strip(), "notes": notes.text().strip(),
             "pole_pairs": pp.value(), "kv": kv.value(), "r_ll_mohm": r.value() * k, "l_ll_uh": l_.value() * k,
             "kt_nm_a": kt.value(), "rpm_max": rmax.value(), "ilim": ilim.value(), "i_drive": idr.value(), "accel": acc.value()}
        for k, v in init.items():                     # keep tuning values the dialog has no field for
            if not k.startswith("_") and k not in d:
                d[k] = v
        err = PR.validate_name(d["name"])
        if err:
            QtWidgets.QMessageBox.warning(dlg, "Motor preset", err)
            continue
        return d
    return None
