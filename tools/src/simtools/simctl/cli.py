import json
import sys
from pathlib import Path
from typing import Annotated

import typer

from simtools.config.resolver import (
    ConfigError,
    load_drone,
    resolve_session,
    write_run_directory,
)
from simtools.modelc import compile_drone, export_glb, report, to_json
from simtools.propfit import ThrustTableError, as_yaml, fit_prop, read_thrust_table
from simtools.simctl.launcher import (
    LaunchError,
    find_simcore,
    install_signal_handlers,
    run_headless,
)
from simtools.simctl.serve import serve_forever

app = typer.Typer(add_completion=False, no_args_is_help=True, help="fpvsim launcher")

BaseDirOption = Annotated[
    Path, typer.Option("--base-dir", help="Directory that paths inside YAML files are relative to")
]


@app.command()
def validate(session: Path, base_dir: BaseDirOption = Path()) -> None:
    """Resolve a session and everything it references without running anything."""
    try:
        resolved = resolve_session(session, base_dir)
    except ConfigError as error:
        typer.echo(str(error), err=True)
        raise typer.Exit(code=2) from error
    typer.echo(
        f"ok: {resolved.name}: {len(resolved.drone['motors'])} motors, "
        f"{len(resolved.cli_lines)} CLI lines"
    )


@app.command()
def run(
    session: Path,
    base_dir: BaseDirOption = Path(),
    headless: Annotated[
        bool, typer.Option("--headless", help="No app (the only mode so far)")
    ] = False,
    run_root: Annotated[
        Path | None, typer.Option("--run-root", help="Overrides logging.root")
    ] = None,
) -> None:
    """Resolve a session into a run directory and run it against Betaflight SITL."""
    _ = headless
    try:
        resolved = resolve_session(session, base_dir)
        root = run_root if run_root is not None else base_dir / resolved.session_logging_root()
        run_dir = write_run_directory(resolved, root, base_dir, sys.argv)
        typer.echo(f"run directory: {run_dir}")
        install_signal_handlers()
        result = run_headless(resolved, run_dir, find_simcore(base_dir))
    except (ConfigError, LaunchError) as error:
        typer.echo(str(error), err=True)
        raise typer.Exit(code=2) from error
    typer.echo(f"simcore exit code {result.exit_code}; logs in {result.run_dir / 'logs'}")
    raise typer.Exit(code=result.exit_code)


@app.command()
def model(
    drone: Path,
    base_dir: BaseDirOption = Path(),
    out: Annotated[
        Path | None, typer.Option("--out", help="Directory for drone.json and drone.glb")
    ] = None,
    show_report: Annotated[bool, typer.Option("--report", help="Print the model report")] = False,
) -> None:
    """Compile a drone parts list into mass properties, geometry and a glb."""
    try:
        compiled = compile_drone(load_drone(drone, base_dir))
    except ConfigError as error:
        typer.echo(str(error), err=True)
        raise typer.Exit(code=2) from error
    if out is not None:
        out.mkdir(parents=True, exist_ok=True)
        (out / "drone.json").write_text(json.dumps(to_json(compiled), indent=2))
        (out / "drone.glb").write_bytes(export_glb(compiled))
        typer.echo(f"wrote {out / 'drone.json'} and {out / 'drone.glb'}")
    if show_report or out is None:
        typer.echo(report(compiled), nl=False)


@app.command()
def propfit(
    table: Path,
    kv: Annotated[float, typer.Option("--kv", help="Motor Kv in rpm per volt")],
    rho_ref: Annotated[float, typer.Option("--rho-ref", help="Reference air density")] = 1.225,
) -> None:
    """Fit prop and motor constants from a thrust table CSV and print them as YAML."""
    try:
        fit = fit_prop(read_thrust_table(table), kv, rho_ref)
    except (ThrustTableError, OSError, ValueError) as error:
        typer.echo(str(error), err=True)
        raise typer.Exit(code=2) from error
    typer.echo(as_yaml(fit), nl=False)


@app.command()
def serve(
    base_dir: BaseDirOption = Path(),
    port: Annotated[int, typer.Option("--port", help="TCP port of the backend API")] = 7740,
) -> None:
    """Run the backend API the app connects to (sessions, start, stop, status, logs)."""
    install_signal_handlers()
    serve_forever(base_dir.resolve(), port=port)


def main() -> None:
    app()


if __name__ == "__main__":
    main()
