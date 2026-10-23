# Copyright 2015 Open Source Robotics Foundation, Inc.
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

"""ament pep257 check of our docstrings (Google style): the package without old/, test/ and scripts/."""


from ament_pep257.main import main
import pytest
from repo_paths import REPO


#: Our code: the package without the old monitor, plus tests and scripts. external/ and data/ are not ours.
ROOT = REPO
PATHS = [str(ROOT / d) for d in ("husky_assembly_teleop", "test", "scripts")]
#: The old monitor, and OptiTrack's NatNet SDK vendored next to the relay.
EXCLUDE = [str(ROOT / "husky_assembly_teleop" / d) for d in ("old", "mocap_relay/natnet")]


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """No docstring findings for Google style; the summary may start on either line (D212 off)."""
    rc = main(argv=['--convention', 'google', '--add-ignore', 'D212', '--exclude', *EXCLUDE, '--', *PATHS])
    assert rc == 0, 'Found code style errors / warnings'
