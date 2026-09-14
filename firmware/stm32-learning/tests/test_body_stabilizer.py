"""Host-only regression: real pure C PD and strict generated configuration.

No HAL, servo, physical timing or mounted IMU-axis validation is performed.
"""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess

import pytest

PROJECT = Path(__file__).resolve().parents[1]
ROOT = PROJECT.parents[1]
GENERATOR = ROOT / 'tools/generate_body_stabilization_config.py'
spec = importlib.util.spec_from_file_location('body_config_generator_test', GENERATOR)
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


def test_nine_standalone_controller_groups(tmp_path):
    cc = shutil.which('cc')
    assert cc, 'A C11 host compiler is required'
    binary = tmp_path / 'body-pd'
    subprocess.run([cc, '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    '-I'+str(PROJECT/'Inc'), str(PROJECT/'tests/test_body_stabilizer.c'),
                    '-lm', '-o', str(binary)], check=True, capture_output=True, text=True)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    assert '9 standalone groups passed' in result.stdout


@pytest.fixture
def config():
    return generator.load_config(ROOT/'config/body_stabilization.json')


def test_generated_header_and_named_order_are_current(config):
    generated = generator.render(config)
    assert generated == (PROJECT/'Inc/body_stabilization_config.h').read_text()
    assert generated == generator.render(dict(reversed(list(config.items()))))


@pytest.mark.parametrize('name', generator.FIELDS)
def test_every_field_is_required(config, name):
    del config[name]
    with pytest.raises(ValueError, match='missing='):
        generator.validate_config(config)


@pytest.mark.parametrize('bad', [None, True, '0.2', float('nan'), float('inf'), -1., 3.])
def test_invalid_numeric_values_rejected(config, bad):
    config['kp_roll'] = bad
    with pytest.raises(ValueError):
        generator.validate_config(config)


@pytest.mark.parametrize('bad', [True, 100., -1, 0, 1001, 2**32+100])
def test_timeout_rejects_wrong_type_or_unsigned_wrap(config, bad):
    config['timeout_ms'] = bad
    with pytest.raises(ValueError):
        generator.validate_config(config)


@pytest.mark.parametrize('field', ['angle_alpha', 'gyro_alpha'])
def test_old_value_lpf_weight_allows_zero_but_not_one(config, field):
    config[field] = 0
    generator.validate_config(config)
    config[field] = 1
    with pytest.raises(ValueError):
        generator.validate_config(config)


@pytest.mark.parametrize('version', [True, 1., 0, 2, '1'])
def test_schema_version_is_exact(config, version):
    config['schema_version'] = version
    with pytest.raises(ValueError):
        generator.validate_config(config)


def test_unknown_fields_rejected(config):
    config['kp_rol'] = .2
    with pytest.raises(ValueError, match='unknown='):
        generator.validate_config(config)


def test_duplicate_fields_are_rejected_before_json_overwrite(tmp_path, config):
    path = tmp_path/'duplicate.json'
    content = json.dumps(config)
    path.write_text(content[:-1]+', "kp_roll": 0.1}')
    with pytest.raises(ValueError, match='Duplicate'):
        generator.load_config(path)
