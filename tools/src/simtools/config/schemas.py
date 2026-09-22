from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


class Strict(BaseModel):
    model_config = ConfigDict(extra="forbid")


class Ports(Strict):
    pwm_raw: int = 9001
    pwm: int = 9002
    fdm: int = 9003
    rc: int = 9004
    uart_base: int = 5761


class BetaflightConfig(Strict):
    binary: str = "build/betaflight/betaflight_SITL.elf"
    protocol_version: Literal["2026.6.2"] = "2026.6.2"
    cli_script: str = "configs/betaflight/baseline_cli.txt"
    cli_extra: list[str] = Field(default_factory=list)
    host: str = "127.0.0.1"
    ports: Ports = Field(default_factory=Ports)


class OriginConfig(Strict):
    lat_deg: float = 56.0
    lon_deg: float = 24.0
    altitude_m: float = 0.0


class SpawnConfig(Strict):
    north_m: float = 0.0
    east_m: float = 0.0
    height_agl_m: float = 0.0
    heading_deg: float = 0.0


class AltitudeHoldConfig(Strict):
    target_height_m: float = 5.0
    climb_rate_mps: float = 1.0
    kp_us_per_m: float = 100.0
    ki_us_per_m_s: float = 20.0
    kd_us_per_mps: float = 80.0
    integral_limit_us: float = 300.0
    arm_delay_s: float = 6.0


class InputConfig(Strict):
    source: Literal["altitude_hold"] = "altitude_hold"
    rc_rate_hz: int = 250
    altitude_hold: AltitudeHoldConfig = Field(default_factory=AltitudeHoldConfig)


class LoggingConfig(Strict):
    rate_hz: int = 200
    root: str = "runs"


class SessionConfig(Strict):
    schema_version: Literal[1]
    name: str
    seed: int = 42
    mode: Literal["realtime"] = "realtime"
    physics_rate_hz: int = 1000
    duration_s: float = 0.0
    betaflight: BetaflightConfig = Field(default_factory=BetaflightConfig)
    drone: str
    environment: str
    terrain: Literal["flat"] = "flat"
    origin: OriginConfig = Field(default_factory=OriginConfig)
    spawn: SpawnConfig = Field(default_factory=SpawnConfig)
    input: InputConfig = Field(default_factory=InputConfig)
    logging: LoggingConfig = Field(default_factory=LoggingConfig)

    @model_validator(mode="after")
    def rates_are_consistent(self) -> "SessionConfig":
        if self.physics_rate_hz <= 0 or self.physics_rate_hz % 1000 != 0:
            raise ValueError("physics_rate_hz must be a positive multiple of 1000")
        if 1_000_000_000 % self.physics_rate_hz != 0:
            raise ValueError("physics_rate_hz must divide 1e9 evenly")
        for name, rate in (
            ("input.rc_rate_hz", self.input.rc_rate_hz),
            ("logging.rate_hz", self.logging.rate_hz),
        ):
            if rate <= 0 or self.physics_rate_hz % rate != 0:
                raise ValueError(f"{name} must divide physics_rate_hz")
        if self.duration_s < 0.0:
            raise ValueError("duration_s must not be negative")
        return self


class LayoutConfig(Strict):
    type: Literal["quad_x"] = "quad_x"
    motor_spacing_mm: float
    props_out: bool = False


class FirstOrderMotorConfig(Strict):
    model: Literal["first_order"] = "first_order"
    max_rpm: float
    time_constant_s: float
    k_t: float
    k_q: float
    rotor_inertia_kg_m2: float


class ImuConfig(Strict):
    offset_mm: Vector3 = (0.0, 0.0, 0.0)


class AeroConfig(Strict):
    cda_m2: Vector3
    cop_mm: Vector3 = (0.0, 0.0, 0.0)
    k_omega: Vector3 = (5e-4, 5e-4, 5e-4)


class ContactConfig(Strict):
    leg_drop_mm: float = 20.0
    stiffness_n_per_m: float = 3000.0
    damping_n_s_per_m: float = 30.0
    friction: float = 0.6
    friction_regularization_mps: float = 0.01
    crash_speed_mps: float = 6.0


class DroneConfig(Strict):
    schema_version: Literal[1]
    name: str
    mass_g: float = Field(gt=0.0)
    inertia_frd_kg_m2: Matrix3
    layout: LayoutConfig
    motor: FirstOrderMotorConfig
    imu: ImuConfig = Field(default_factory=ImuConfig)
    aero: AeroConfig
    contact: ContactConfig = Field(default_factory=ContactConfig)


class AtmosphereConfig(Strict):
    ground_temperature_c: float = 15.0
    ground_pressure_hpa: float = 1013.25


class EnvironmentConfig(Strict):
    schema_version: Literal[1]
    atmosphere: AtmosphereConfig = Field(default_factory=AtmosphereConfig)
