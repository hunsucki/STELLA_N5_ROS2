"""PEP 257 linter test."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Check source files for docstring errors."""
    return_code = main(argv=['.', 'test'])
    assert return_code == 0, 'Found code style errors or warnings'
