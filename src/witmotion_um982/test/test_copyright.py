# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
from ament_copyright.main import main
import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'
