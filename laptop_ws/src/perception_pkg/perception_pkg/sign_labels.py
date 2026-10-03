"""Pure helpers that map detector classes to the labels the robot acts on."""

# Signs present on the current track. Any other class the detector reports is
# treated as "no sign", so an unused class can never trigger a behaviour.
ACTIVE_SIGN_LABELS = frozenset({'turn_left', 'speed_limit_20', 'stop'})

# The production detectors (yolov8n.onnx, viet.pt) store Vietnamese class
# names; the rest of the system uses these English labels.
LABEL_ALIASES = {
    'cam_re_phai': 'no_right_turn',
    'di_cham': 'slow_down',
    'dung_lai': 'stop',
    're_trai': 'turn_left',
    'toc_do_toi_da_20': 'speed_limit_20',
}


def normalize_label(name):
    label = str(name).strip().lower().replace('-', '_').replace(' ', '_')
    return LABEL_ALIASES.get(label, label)


def build_class_map(model_names, fallback, allowed):
    """Return ``(class_map, source)`` mapping class id to an allowed label.

    Names stored in the loaded model take priority, so the mapping always
    follows the model on disk. The hard-coded ``fallback`` is used only when
    the model reports no name the robot recognises (e.g. ``class0``).
    """
    if model_names:
        items = (model_names.items() if isinstance(model_names, dict)
                 else enumerate(model_names))
        from_model = {int(i): normalize_label(n) for i, n in items}
        if any(label in allowed for label in from_model.values()):
            return (
                {i: lb for i, lb in from_model.items() if lb in allowed},
                'model')
    return {i: lb for i, lb in fallback.items() if lb in allowed}, 'fallback'


def pick_detection(classes, confs, y_bottoms, class_map, conf_threshold):
    """Return ``(label, conf, y_bottom)`` of the most confident mapped box."""
    best = ('none', 0.0, None)
    for cls, conf, y_bottom in zip(classes, confs, y_bottoms):
        label = class_map.get(int(cls))
        conf = float(conf)
        if label is None or conf < conf_threshold or conf <= best[1]:
            continue
        best = (label, conf, float(y_bottom))
    return best
