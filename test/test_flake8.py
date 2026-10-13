# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ament flake8 check of our code: the package without old/, test/ and scripts/."""

from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest


#: Our code: the package without the old monitor, plus tests and scripts. external/ and data/ are not ours.
ROOT = Path(__file__).resolve().parents[1]
PATHS = [str(ROOT / d) for d in ("husky_assembly_teleop", "test", "scripts")]
EXCLUDE = [str(ROOT / "husky_assembly_teleop" / "old")]


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """No flake8 findings, with lines up to 120 characters."""
    rc, errors = main_with_errors(argv=['--linelength', '120', '--exclude', *EXCLUDE, '--', *PATHS,
                                        str(ROOT / 'setup.py')])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
