"""Unit tests for mapping detector classes to active track signs."""

from perception_pkg.sign_labels import (
    ACTIVE_SIGN_LABELS, build_class_map, pick_detection)

FIVE_CLASS_ORDER = {
    0: 'no_right_turn',
    1: 'slow_down',
    2: 'stop',
    3: 'turn_left',
    4: 'speed_limit_20',
}


def test_model_names_take_priority_over_hard_coded_order():
    model_names = {0: 'turn_left', 1: 'Speed-Limit 20', 2: 'STOP'}
    class_map, source = build_class_map(
        model_names, FIVE_CLASS_ORDER, ACTIVE_SIGN_LABELS)
    assert source == 'model'
    assert class_map == {0: 'turn_left', 1: 'speed_limit_20', 2: 'stop'}


def test_vietnamese_names_of_production_model_are_mapped():
    production_names = {
        0: 'cam_re_phai', 1: 'di_cham', 2: 'dung_lai', 3: 're_trai',
        4: 'toc_do_toi_da_20',
    }
    class_map, source = build_class_map(
        production_names, {}, ACTIVE_SIGN_LABELS)
    assert source == 'model'
    assert class_map == {2: 'stop', 3: 'turn_left', 4: 'speed_limit_20'}


def test_unused_classes_are_dropped():
    class_map, source = build_class_map(
        FIVE_CLASS_ORDER, {}, ACTIVE_SIGN_LABELS)
    assert source == 'model'
    assert class_map == {2: 'stop', 3: 'turn_left', 4: 'speed_limit_20'}


def test_fallback_when_model_names_are_unrecognised():
    for model_names in (None, {}, {0: 'class0', 1: 'class1'}):
        class_map, source = build_class_map(
            model_names, FIVE_CLASS_ORDER, ACTIVE_SIGN_LABELS)
        assert source == 'fallback'
        assert class_map == {2: 'stop', 3: 'turn_left', 4: 'speed_limit_20'}


def test_unused_class_never_wins_even_with_higher_confidence():
    class_map = {2: 'stop', 3: 'turn_left'}
    label, conf, y_bottom = pick_detection(
        [0, 3], [0.95, 0.7], [100.0, 120.0], class_map, 0.5)
    assert (label, conf, y_bottom) == ('turn_left', 0.7, 120.0)


def test_only_unused_or_weak_boxes_mean_no_sign():
    class_map = {2: 'stop'}
    assert pick_detection([0, 1], [0.9, 0.9], [1.0, 2.0], class_map, 0.5) == (
        'none', 0.0, None)
    assert pick_detection([2], [0.4], [1.0], class_map, 0.5) == (
        'none', 0.0, None)
