"""Motor presets: bundled ones live in ./presets, your own ones in %APPDATA%/ESC_Control/presets."""
import json
import os
import sys
from pathlib import Path

from protocol import PARAM_SPEC_BY_NAME

# preset key -> firmware parameter name
PRESET_TO_PARAM = {
    "pole_pairs": "pp", "kv": "kv", "r_ll_mohm": "rl", "l_ll_uh": "ll", "kt_nm_a": "kt",
    "rpm_max": "rpmmax", "ilim": "ilim", "i_drive": "idrive", "accel": "accel",
    # optional values from the calibration / tuning (stage 2)
    "enc_dir": "encdir",
    "i_max": "imax", "cur_bw": "curbw", "spd_kp": "spdkp", "spd_ki": "spdki",
    "pos_kp": "poskp", "pos_ki": "poski", "pos_kd": "poskd", "pos_vmax": "posvmax", "pos_acc": "posacc",
}
PARAM_TO_PRESET = {v: k for k, v in PRESET_TO_PARAM.items()}


NAME_MAX = 23         # the board stores the name in 24 bytes


def validate_name(name):
    """Return an error text, or None if the name can be stored on the board."""
    if not name or not name.strip():
        return "Please enter a name."
    if len(name) > NAME_MAX:
        return "The name can have at most %d characters." % NAME_MAX
    if any(ord(c) < 0x20 or ord(c) > 0x7E or c == "," for c in name):
        return "Use plain ASCII characters only, no commas."
    return None


def values_match(preset, params):
    """True if the board's current parameters equal the preset's values."""
    for key, pname in PRESET_TO_PARAM.items():
        if key in preset and pname in params and abs(float(preset[key]) - float(params[pname])) > 1e-3 * max(1.0, abs(float(preset[key]))):
            return False
    return True


def bundled_dir():
    base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    return base / "presets"


def user_dir():
    root = os.environ.get("APPDATA") or str(Path.home() / ".config")
    return Path(root) / "ESC_Control" / "presets"


def _read(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        if isinstance(d, dict) and "name" in d:
            return d
    except (OSError, ValueError):
        pass
    return None


def load_presets():
    """name -> preset dict (user presets override bundled ones with the same name)."""
    out = {}
    for d, user in ((bundled_dir(), False), (user_dir(), True)):
        if d.is_dir():
            for p in sorted(d.glob("*.json")):
                pr = _read(p)
                if pr:
                    pr = dict(pr)
                    pr["_user"] = user
                    pr["_path"] = str(p)
                    out[pr["name"]] = pr
    return out


def preset_commands(preset):
    """List of 'set ...' commands that apply the preset (values outside the firmware limits are skipped)."""
    cmds, skipped = [], []
    for key, pname in PRESET_TO_PARAM.items():
        if key not in preset:
            continue
        v = float(preset[key])
        spec = PARAM_SPEC_BY_NAME[pname]
        if not (spec[3] <= v <= spec[4]):
            skipped.append("%s=%s" % (key, v))
            continue
        cmds.append("set %s %s" % (pname, ("%d" % round(v)) if spec[2] == "int" else ("%.6g" % v)))
    return cmds, skipped


def preset_from_params(name, params, notes=""):
    d = {"name": name, "notes": notes}
    for pname, key in PARAM_TO_PRESET.items():
        if pname in params:
            v = params[pname]
            d[key] = int(round(v)) if key == "pole_pairs" else round(float(v), 6)
    return d


def save_user_preset(preset):
    folder = user_dir()
    folder.mkdir(parents=True, exist_ok=True)
    safe = "".join(c if (c.isalnum() or c in "-_ .") else "_" for c in preset["name"]).strip() or "preset"
    path = folder / (safe + ".json")
    clean = {k: v for k, v in preset.items() if not k.startswith("_")}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(clean, f, indent=2)
    return path


def delete_user_preset(preset):
    if preset.get("_user") and preset.get("_path"):
        try:
            os.remove(preset["_path"])
            return True
        except OSError:
            return False
    return False
