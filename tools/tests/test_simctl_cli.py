from pathlib import Path

from typer.testing import CliRunner

from simtools.simctl.cli import app

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_validate_accepts_the_ci_session() -> None:
    result = CliRunner().invoke(
        app,
        [
            "validate",
            str(REPO_ROOT / "configs/sessions/ci_hover.yaml"),
            "--base-dir",
            str(REPO_ROOT),
        ],
    )
    assert result.exit_code == 0, result.output
    assert "ok: ci_hover: 4 motors" in result.output


def test_validate_reports_errors_with_exit_code_2(tmp_path: Path) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text("schema_version: 1\nname: x\ndrone: nope.yaml\nenvironment: nope.yaml\n")
    result = CliRunner().invoke(app, ["validate", str(bad), "--base-dir", str(REPO_ROOT)])
    assert result.exit_code == 2
    assert "nope.yaml" in result.output
