import re
from typing import Annotated, Literal

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
    esc_request: int = 9005
    esc_uart: int = 4
    uart_base: int = 5761


class BetaflightConfig(Strict):
    binary: str = "build/betaflight/betaflight_SITL.elf"
    protocol_version: Literal["2026.6.2"] = "2026.6.2"
    cli_script: str = "configs/betaflight/baseline_cli.txt"
    cli_extra: list[str] = Field(default_factory=list)
    host: str = "127.0.0.1"
    ports: Ports = Field(default_factory=Ports)
    virtual_esc: bool = True


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
    source: Literal["altitude_hold", "gamepad"] = "altitude_hold"
    rc_rate_hz: int = 250
    altitude_hold: AltitudeHoldConfig = Field(default_factory=AltitudeHoldConfig)
    mapping: str | None = None

    @model_validator(mode="after")
    def mapping_matches_source(self) -> "InputConfig":
        if self.source == "gamepad" and self.mapping is None:
            raise ValueError("input.mapping is required with source: gamepad")
        return self


CHANNEL_NAMES = ("roll", "pitch", "throttle", "yaw", *(f"aux{i}" for i in range(1, 13)))


class ChannelSourceConfig(Strict):
    axis: int | None = Field(default=None, ge=0)
    button: int | None = Field(default=None, ge=0)
    inverted: bool = False
    deadband: float = Field(default=0.0, ge=0.0, lt=1.0)

    @model_validator(mode="after")
    def exactly_one_source(self) -> "ChannelSourceConfig":
        if (self.axis is None) == (self.button is None):
            raise ValueError("exactly one of axis or button")
        return self


class DeviceConfig(Strict):
    name_contains: str


class InputMappingConfig(Strict):
    schema_version: Literal[1]
    name: str
    device: DeviceConfig
    channels: dict[str, ChannelSourceConfig]
    arm_channel: str = "aux1"

    @model_validator(mode="after")
    def channel_names_are_known(self) -> "InputMappingConfig":
        for name in (*self.channels, self.arm_channel):
            if name not in CHANNEL_NAMES:
                raise ValueError(
                    f"unknown channel '{name}'; use roll, pitch, throttle, yaw, aux1..aux12"
                )
        return self


class ControlApiConfig(Strict):
    host: str = "127.0.0.1"
    port: int = 7700


class AppConfig(Strict):
    enabled: bool = True
    host: str = "127.0.0.1"
    port: int = 7710
    state_rate_hz: int = 100


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
    control_api: ControlApiConfig = Field(default_factory=ControlApiConfig)
    app: AppConfig = Field(default_factory=AppConfig)
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
            ("app.state_rate_hz", self.app.state_rate_hz),
        ):
            if rate <= 0 or self.physics_rate_hz % rate != 0:
                raise ValueError(f"{name} must divide physics_rate_hz")
        if self.duration_s < 0.0:
            raise ValueError("duration_s must not be negative")
        return self


GENERATED_PART = re.compile(r"^(arm|motor|prop)\d+$")


class PartBase(Strict):
    mass_g: float = Field(gt=0.0)
    pos_mm: Vector3 = (0.0, 0.0, 0.0)
    rot_deg: Vector3 = (0.0, 0.0, 0.0)
    color: str | None = Field(default=None, pattern=r"^#[0-9a-fA-F]{6}$")


class BoxPart(PartBase):
    shape: Literal["box"]
    size_mm: Vector3


class CylinderPart(PartBase):
    shape: Literal["cylinder"]
    diameter_mm: float = Field(gt=0.0)
    height_mm: float = Field(gt=0.0)


class SpherePart(PartBase):
    shape: Literal["sphere"]
    diameter_mm: float = Field(gt=0.0)


class MeshPart(PartBase):
    shape: Literal["mesh"]
    file: str

    @model_validator(mode="after")
    def not_supported(self) -> "MeshPart":
        raise ValueError("mesh parts are reserved but not supported yet")


Part = Annotated[BoxPart | CylinderPart | SpherePart | MeshPart, Field(discriminator="shape")]


class ArmConfig(Strict):
    width_mm: float = Field(default=12.0, gt=0.0)
    thickness_mm: float = Field(default=5.0, gt=0.0)
    mass_g: float = Field(default=11.0, gt=0.0)
    color: str | None = None


class MotorBodyConfig(Strict):
    diameter_mm: float = Field(default=28.0, gt=0.0)
    height_mm: float = Field(default=30.0, gt=0.0)
    mass_g: float = Field(default=30.0, gt=0.0)
    color: str | None = None


class PropBodyConfig(Strict):
    diameter_mm: float = Field(default=127.0, gt=0.0)
    thickness_mm: float = Field(default=5.0, gt=0.0)
    mass_g: float = Field(default=5.0, gt=0.0)
    color: str | None = None


class CustomMotorConfig(Strict):
    bf_index: int = Field(ge=1)
    pos_mm: Vector3
    axis: Vector3 = (0.0, 0.0, -1.0)
    spin: Literal["cw", "ccw"]


class LayoutConfig(Strict):
    type: Literal["quad_x", "stretched_x", "custom"] = "quad_x"
    motor_spacing_mm: float | None = Field(default=None, gt=0.0)
    length_mm: float | None = Field(default=None, gt=0.0)
    width_mm: float | None = Field(default=None, gt=0.0)
    motors: list[CustomMotorConfig] = Field(default_factory=list[CustomMotorConfig])
    props_out: bool = False
    arm: ArmConfig = Field(default_factory=ArmConfig)
    motor_body: MotorBodyConfig = Field(default_factory=MotorBodyConfig)
    prop_body: PropBodyConfig = Field(default_factory=PropBodyConfig)

    @model_validator(mode="after")
    def fields_match_type(self) -> "LayoutConfig":
        if self.type == "quad_x" and self.motor_spacing_mm is None:
            raise ValueError("quad_x needs motor_spacing_mm")
        if self.type == "stretched_x" and (self.length_mm is None or self.width_mm is None):
            raise ValueError("stretched_x needs length_mm and width_mm")
        if self.type == "custom":
            if self.props_out:
                raise ValueError("custom layouts give each motor its spin; props_out is not used")
            indices = sorted(m.bf_index for m in self.motors)
            if not indices or indices != list(range(1, len(indices) + 1)):
                raise ValueError("custom motors need bf_index 1..N without gaps")
            if len(self.motors) > 8:
                raise ValueError("at most 8 motors")
        return self


class MountConfig(Strict):
    part: str | None = None
    offset_mm: Vector3 = (0.0, 0.0, 0.0)
    rot_deg: Vector3 = (0.0, 0.0, 0.0)


class CameraMountConfig(MountConfig):
    name: str


class AeroOverrideConfig(Strict):
    cda_m2: Vector3
    cop_mm: Vector3 = (0.0, 0.0, 0.0)


class AeroConfig(Strict):
    cd: float = Field(default=1.0, gt=0.0)
    override: AeroOverrideConfig | None = None
    k_omega: Vector3 = (5e-4, 5e-4, 5e-4)


class ContactConfig(Strict):
    leg_drop_mm: float = 20.0
    stiffness_n_per_m: float = 3000.0
    damping_n_s_per_m: float = 30.0
    friction: float = 0.6
    friction_regularization_mps: float = 0.01
    crash_speed_mps: float = 6.0


LIPO_OCV_V = (3.30, 3.60, 3.70, 3.75, 3.79, 3.83, 3.87, 3.93, 4.00, 4.10, 4.20)
LIPO_TEMPERATURE_FACTOR = (
    (-10.0, 2.0),
    (0.0, 1.6),
    (10.0, 1.3),
    (25.0, 1.0),
    (40.0, 0.9),
    (60.0, 0.85),
)


class MotorConfig(Strict):
    schema_version: Literal[1] = 1
    model: Literal["dc", "first_order"] = "dc"
    kv_rpm_per_v: float = Field(gt=0.0)
    winding_resistance_ohm: float = Field(gt=0.0)  # DC-equivalent value, fitted, not the datasheet
    no_load_current_a: float = Field(default=0.0, ge=0.0)
    brake_current_a: float = Field(default=10.0, ge=0.0)
    poles: int = Field(default=14, ge=2)
    rotor_inertia_kg_m2: float = Field(gt=0.0)
    reference_voltage_v: float = Field(default=24.0, gt=0.0)
    max_rpm: float | None = Field(
        default=None, gt=0.0
    )  # first_order: at full throttle and reference voltage
    time_constant_s: float = Field(default=0.02, gt=0.0)
    mass_g: float | None = None
    thrust_table: str | None = None

    @model_validator(mode="after")
    def first_order_needs_max_rpm(self) -> "MotorConfig":
        if self.model == "first_order" and self.max_rpm is None:
            raise ValueError("first_order motors need max_rpm")
        if self.poles % 2 != 0:
            raise ValueError("poles must be even")
        return self


class PropConfig(Strict):
    schema_version: Literal[1] = 1
    diameter_mm: float = Field(gt=0.0)
    pitch_mm: float = Field(gt=0.0)
    blades: int = Field(default=3, ge=2)
    mass_g: float | None = None
    k_t: float = Field(gt=0.0)
    k_q: float = Field(gt=0.0)
    rho_ref_kg_m3: float = Field(default=1.225, gt=0.0)
    inflow_coefficient: float = Field(default=1.0, ge=0.0)
    k_h: float = Field(default=0.0, ge=0.0)


class BatteryConfig(Strict):
    schema_version: Literal[1] = 1
    cells: int = Field(ge=1)
    capacity_mah: float = Field(gt=0.0)
    cell_resistance_mohm: float = Field(gt=0.0)
    connector_resistance_mohm: float = Field(default=0.0, ge=0.0)
    temperature_c: float = 25.0
    avionics_current_a: float = Field(default=0.5, ge=0.0)
    esc_cutoff_v: float = Field(default=0.0, ge=0.0)
    initial_soc: float = Field(default=1.0, ge=0.0, le=1.0)
    rc_resistance_mohm: float = Field(default=0.0, ge=0.0)
    rc_capacitance_f: float = Field(default=0.0, ge=0.0)
    ocv_curve_v: tuple[float, ...] = LIPO_OCV_V
    temperature_factor: tuple[tuple[float, float], ...] = LIPO_TEMPERATURE_FACTOR

    @model_validator(mode="after")
    def tables_are_well_formed(self) -> "BatteryConfig":
        if len(self.ocv_curve_v) != 11:
            raise ValueError("ocv_curve_v needs 11 values, per cell at SOC 0, 0.1 .. 1")
        if any(b <= a for a, b in zip(self.ocv_curve_v, self.ocv_curve_v[1:], strict=False)):
            raise ValueError("ocv_curve_v must increase with SOC")
        temps = [t for t, _ in self.temperature_factor]
        if len(temps) < 2 or temps != sorted(temps):
            raise ValueError("temperature_factor must list at least two rising temperatures")
        return self


class ImuNoiseConfig(Strict):
    gyro_noise_density_dps_rthz: float = Field(default=0.005, ge=0.0)
    gyro_bias_walk_dps2_rthz: float = Field(default=0.001, ge=0.0)
    gyro_range_dps: float = Field(default=2000.0, ge=0.0)
    accel_noise_density_mps2_rthz: float = Field(default=0.003, ge=0.0)
    accel_bias_walk_mps3_rthz: float = Field(default=0.0005, ge=0.0)
    accel_range_g: float = Field(default=16.0, ge=0.0)
    vibration_imbalance_mps2_per_radps2: float = Field(default=2e-7, ge=0.0)
    vibration_harmonic2: float = Field(default=0.3, ge=0.0)
    vibration_blade_pass: float = Field(default=0.2, ge=0.0)
    vibration_gyro_gain_radps_per_mps2: float = Field(default=0.02, ge=0.0)


class BaroConfig(Strict):
    noise_pa: float = Field(default=3.0, ge=0.0)
    bias_pa: float = 0.0


class SensorsConfig(Strict):
    imu: ImuNoiseConfig = Field(default_factory=ImuNoiseConfig)
    baro: BaroConfig = Field(default_factory=BaroConfig)


class DroneConfig(Strict):
    schema_version: Literal[1]
    name: str
    layout: LayoutConfig
    motor: MotorConfig
    prop: PropConfig
    battery: BatteryConfig
    sensors: SensorsConfig = Field(default_factory=SensorsConfig)
    parts: dict[str, Part]
    imu: MountConfig = Field(default_factory=MountConfig)
    cameras: list[CameraMountConfig] = Field(default_factory=list[CameraMountConfig])
    aero: AeroConfig = Field(default_factory=AeroConfig)
    contact: ContactConfig = Field(default_factory=ContactConfig)

    @model_validator(mode="after")
    def parts_and_mounts_are_consistent(self) -> "DroneConfig":
        if not self.parts:
            raise ValueError("parts must not be empty")
        for name in self.parts:
            if GENERATED_PART.match(name):
                raise ValueError(f"part name '{name}' is reserved for generated parts")
        for label, part in (("imu", self.imu.part), *((c.name, c.part) for c in self.cameras)):
            if part is not None and part not in self.parts:
                raise ValueError(f"{label} refers to unknown part '{part}'")
        names = [c.name for c in self.cameras]
        if len(set(names)) != len(names):
            raise ValueError("camera names must be unique")
        return self


class AtmosphereConfig(Strict):
    ground_temperature_c: float = 15.0
    ground_pressure_hpa: float = 1013.25


class EnvironmentConfig(Strict):
    schema_version: Literal[1]
    atmosphere: AtmosphereConfig = Field(default_factory=AtmosphereConfig)
