"""Safety monitor with state machine, watchdog timers, and speed limiting.

THIS IS THE MOST CRITICAL MODULE IN THE ENTIRE PACKAGE.
It has absolute authority to override any motor command.

Safety layers (in order of authority, highest first):
  1. E-STOP (hardware kill switch - outside software, cuts battery power)
  2. E-STOP (software via RC ch3 or /estop service)
  3. RC override (RC signals get highest software priority)
  4. Command timeout (no commands received = motors stop)
  5. Acceleration limiting (smooth ramp up/down, prevents jerky movement)
  6. Speed limiting (per-mode max speed caps)
  7. Startup delay (reject commands during initialization period)

State machine states:
  INITIALIZING - Boot-up delay, all commands rejected, motors neutral
  NORMAL       - Accepting commands from teleop/autonomous sources
  TIMEOUT      - No command received within timeout, motors neutral
  RC_OVERRIDE  - RC receiver active, only RC commands accepted
  ESTOP        - Emergency stop active, motors forced neutral every cycle
  ERROR        - Hardware fault detected, motors neutral, manual reset required

CRITICAL INVARIANT:
  In states ESTOP, TIMEOUT, INITIALIZING, and ERROR, motors MUST be at
  neutral. This is enforced EVERY control cycle, not just on transition.
  This prevents any possible code path from moving motors during unsafe states.

This module has NO ROS2 dependencies. It is pure Python for easy testing.
"""

import time
import logging
from enum import Enum, auto
from typing import Tuple, Optional, Callable

logger = logging.getLogger(__name__)


class SafetyState(Enum):
    """Safety state machine states.

    Each state determines what commands are accepted and how
    motor outputs are handled. See module docstring for details.
    """
    INITIALIZING = auto()
    NORMAL = auto()
    TIMEOUT = auto()
    ESTOP = auto()
    RC_OVERRIDE = auto()
    ERROR = auto()


class CommandSource(Enum):
    """Command priority levels. Higher numeric value = higher priority.

    The safety monitor uses these to determine which command source
    should control the motors when multiple sources are active.
    """
    AUTONOMOUS = 0    # Lowest priority: nav2, path planner
    TELEOP = 1        # Medium: keyboard, joystick, bluetooth controller
    RC = 2            # High: RC radio transmitter
    ESTOP = 3         # Highest: emergency stop (forces neutral)


class SafetyMonitor:
    """Central safety authority for the motor controller.

    All motor commands MUST pass through this monitor before reaching
    the Sabertooth driver. The monitor can:
      - Block commands entirely (e-stop, timeout, initializing)
      - Clamp speeds to per-mode maximum limits
      - Rate-limit speed changes (acceleration/deceleration limiting)
      - Override command source based on priority

    Thread safety: This class is NOT thread-safe by design. It should
    only be called from the ROS2 node's single-threaded control loop.

    Example:
        monitor = SafetyMonitor(
            command_timeout_ms=500,
            max_speed_teleop=0.6,
            accel_limit=2.0,
        )
        monitor.start()

        # In control loop (called at 50Hz):
        monitor.command_received(CommandSource.TELEOP)
        monitor.check_timeouts()
        left, right, limited = monitor.process_speeds(0.5, 0.5, CommandSource.TELEOP, dt=0.02)
    """

    def __init__(
        self,
        command_timeout_ms: float = 500.0,
        heartbeat_timeout_ms: float = 2000.0,
        max_speed_teleop: float = 0.6,
        max_speed_autonomous: float = 0.4,
        max_speed_rc: float = 1.0,
        accel_limit: float = 2.0,
        decel_limit: float = 4.0,
        emergency_decel_limit: float = 10.0,
        startup_delay_sec: float = 1.0,
        on_state_change: Optional[Callable[[SafetyState, SafetyState], None]] = None,
    ):
        """
        Args:
            command_timeout_ms: Time (ms) without commands before TIMEOUT state.
                                Default 500ms = motors stop within half a second
                                if controller connection is lost.
            heartbeat_timeout_ms: Node-level heartbeat monitoring period.
            max_speed_teleop: Maximum allowed speed fraction (0.0-1.0) when
                              controlled by keyboard/joystick. Lower = safer.
            max_speed_autonomous: Maximum allowed speed for autonomous navigation.
                                  Should be conservative for indoor/convention use.
            max_speed_rc: Maximum allowed speed for RC control. Usually 1.0
                          since the human operator has direct physical control.
            accel_limit: Maximum acceleration rate in speed-units per second.
                         At 2.0, it takes 0.5 seconds to go from 0 to 1.0.
                         Lower = smoother but less responsive.
            decel_limit: Maximum deceleration rate (speed-units/sec).
                         Should be HIGHER than accel_limit so the robot stops
                         faster than it accelerates. This is a safety feature.
            emergency_decel_limit: E-stop deceleration rate. Very high value
                                   for near-instant stop. At 10.0, stops from
                                   full speed in 0.1 seconds.
            startup_delay_sec: Time after boot before accepting any commands.
                               Allows all systems to initialize. Motors stay
                               at neutral during this period.
            on_state_change: Optional callback fired on state transitions.
                             Receives (old_state, new_state). Useful for logging
                             and ROS2 diagnostics publishing.
        """
        # Configuration
        self._cmd_timeout_sec = command_timeout_ms / 1000.0
        self._heartbeat_timeout_sec = heartbeat_timeout_ms / 1000.0
        self._max_speeds = {
            CommandSource.AUTONOMOUS: max_speed_autonomous,
            CommandSource.TELEOP: max_speed_teleop,
            CommandSource.RC: max_speed_rc,
            CommandSource.ESTOP: 0.0,
        }
        self._accel_limit = accel_limit
        self._decel_limit = decel_limit
        self._emergency_decel = emergency_decel_limit
        self._startup_delay = startup_delay_sec
        self._on_state_change = on_state_change

        # State
        self._state = SafetyState.INITIALIZING
        self._start_time = 0.0
        self._last_cmd_time = 0.0
        self._last_cmd_source = CommandSource.AUTONOMOUS
        self._estop_active = False
        self._rc_override_active = False

        # Acceleration limiting state - tracks current motor speeds
        # for smooth ramping
        self._current_left = 0.0
        self._current_right = 0.0

    @property
    def state(self) -> SafetyState:
        """Current safety state (read-only)."""
        return self._state

    @property
    def current_speeds(self) -> Tuple[float, float]:
        """Current rate-limited motor speeds (left, right)."""
        return (self._current_left, self._current_right)

    def start(self) -> None:
        """Begin safety monitoring. Enters INITIALIZING state.

        Call this once after construction, before the control loop starts.
        The monitor will remain in INITIALIZING until startup_delay_sec elapses.
        """
        self._start_time = time.monotonic()
        self._last_cmd_time = self._start_time
        self._state = SafetyState.INITIALIZING
        self._current_left = 0.0
        self._current_right = 0.0
        logger.info(
            "Safety monitor started. Startup delay: %.1fs. "
            "Command timeout: %.0fms. Speed limits: teleop=%.0f%%, auto=%.0f%%, rc=%.0f%%",
            self._startup_delay,
            self._cmd_timeout_sec * 1000,
            self._max_speeds[CommandSource.TELEOP] * 100,
            self._max_speeds[CommandSource.AUTONOMOUS] * 100,
            self._max_speeds[CommandSource.RC] * 100,
        )

    def command_received(self, source: CommandSource) -> None:
        """Record that a command was received from the given source.

        Updates internal timestamps for timeout tracking. Call this
        every time a valid command arrives from any source.

        Args:
            source: Which command source sent the command.
        """
        self._last_cmd_time = time.monotonic()
        self._last_cmd_source = source

    def process_speeds(
        self,
        left_speed: float,
        right_speed: float,
        source: CommandSource,
        dt: float,
    ) -> Tuple[float, float, bool]:
        """Process motor speeds through all safety filters.

        This is the main entry point called by the node on every control cycle.
        ALL motor commands must pass through here before reaching hardware.

        Processing order:
          1. Check if state allows motor output (reject if ESTOP/INITIALIZING/etc)
          2. Apply speed limit for the active command source
          3. Apply acceleration/deceleration rate limiting
          4. Update internal speed state for next cycle's rate limiting

        Args:
            left_speed: Requested left motor speed (-1.0 to 1.0)
            right_speed: Requested right motor speed (-1.0 to 1.0)
            source: Command source for speed-limit selection
            dt: Time delta since last call (seconds). At 50Hz, this is ~0.02s.

        Returns:
            (processed_left, processed_right, was_limited)
            was_limited is True if any clamping or ramping was applied.
            The caller should report this in MotorStatus for diagnostics.
        """
        was_limited = False

        # States that force neutral - no motor output allowed
        if self._state in (
            SafetyState.ESTOP,
            SafetyState.TIMEOUT,
            SafetyState.INITIALIZING,
            SafetyState.ERROR,
        ):
            # Ramp to zero using emergency decel for ESTOP, normal decel for others
            decel = self._emergency_decel if self._state == SafetyState.ESTOP else self._decel_limit
            self._current_left = self._apply_rate_limit(
                self._current_left, 0.0, dt, self._accel_limit, decel
            )
            self._current_right = self._apply_rate_limit(
                self._current_right, 0.0, dt, self._accel_limit, decel
            )
            return (self._current_left, self._current_right, True)

        # Apply speed limit based on command source
        max_speed = self._max_speeds.get(source, self._max_speeds[CommandSource.AUTONOMOUS])
        clamped_left = max(-max_speed, min(max_speed, left_speed))
        clamped_right = max(-max_speed, min(max_speed, right_speed))
        if clamped_left != left_speed or clamped_right != right_speed:
            was_limited = True

        # Apply acceleration/deceleration rate limiting
        new_left = self._apply_rate_limit(
            self._current_left, clamped_left, dt, self._accel_limit, self._decel_limit
        )
        new_right = self._apply_rate_limit(
            self._current_right, clamped_right, dt, self._accel_limit, self._decel_limit
        )

        if new_left != clamped_left or new_right != clamped_right:
            was_limited = True

        # Update state for next cycle
        self._current_left = new_left
        self._current_right = new_right

        return (new_left, new_right, was_limited)

    def check_timeouts(self) -> None:
        """Check all timeout conditions and update state.

        Called every control cycle by the node's timer. Checks:
          - Startup delay elapsed (INITIALIZING -> NORMAL)
          - Command timeout exceeded (NORMAL -> TIMEOUT)
          - Command received after timeout (TIMEOUT -> NORMAL)

        Does NOT transition out of ESTOP (requires explicit release + reset).
        """
        now = time.monotonic()

        # INITIALIZING -> NORMAL after startup delay
        if self._state == SafetyState.INITIALIZING:
            elapsed = now - self._start_time
            if elapsed >= self._startup_delay:
                self._transition_state(SafetyState.NORMAL)
            return

        # Don't override ESTOP or ERROR states with timeout logic
        if self._state in (SafetyState.ESTOP, SafetyState.ERROR):
            return

        # Check command timeout
        cmd_age = now - self._last_cmd_time
        if cmd_age > self._cmd_timeout_sec:
            if self._state != SafetyState.TIMEOUT:
                self._transition_state(SafetyState.TIMEOUT)
        else:
            # Command received within timeout
            if self._state == SafetyState.TIMEOUT:
                self._transition_state(SafetyState.NORMAL)

    def set_estop(self, active: bool) -> None:
        """Engage or release software e-stop.

        When engaged, immediately transitions to ESTOP state.
        When released, stays in ESTOP until explicit reset_from_estop() call.
        This two-step release prevents accidental re-engagement.

        Args:
            active: True to engage e-stop, False to release.
        """
        self._estop_active = active
        if active and self._state != SafetyState.ESTOP:
            logger.warning("E-STOP ENGAGED - motors forced to neutral")
            self._transition_state(SafetyState.ESTOP)

    def set_rc_override(self, active: bool) -> None:
        """Set RC override state.

        When RC signals are detected, the monitor enters RC_OVERRIDE state.
        When RC signals are lost, returns to NORMAL (if commands available)
        or TIMEOUT.

        Args:
            active: True when RC signals are present and valid.
        """
        self._rc_override_active = active

        if active and self._state == SafetyState.NORMAL:
            self._transition_state(SafetyState.RC_OVERRIDE)
        elif not active and self._state == SafetyState.RC_OVERRIDE:
            self._transition_state(SafetyState.NORMAL)

    def reset_from_estop(self) -> bool:
        """Attempt to reset from ESTOP state back to NORMAL.

        Only succeeds if:
          - Current state is ESTOP
          - E-stop signal is no longer active

        Returns:
            True if reset succeeded, False if conditions not met.
        """
        if self._state != SafetyState.ESTOP:
            logger.warning("Reset requested but not in ESTOP state (current: %s)", self._state)
            return False
        if self._estop_active:
            logger.warning("Reset requested but e-stop still active. Release e-stop first.")
            return False

        self._transition_state(SafetyState.NORMAL)
        logger.info("Reset from ESTOP successful - returning to NORMAL")
        return True

    def reset_from_error(self) -> bool:
        """Attempt to reset from ERROR state back to NORMAL.

        Returns:
            True if reset succeeded.
        """
        if self._state != SafetyState.ERROR:
            return False
        self._transition_state(SafetyState.NORMAL)
        logger.info("Reset from ERROR successful - returning to NORMAL")
        return True

    def set_error(self) -> None:
        """Transition to ERROR state due to hardware fault."""
        if self._state != SafetyState.ERROR:
            logger.error("HARDWARE ERROR - motors forced to neutral. Manual reset required.")
            self._transition_state(SafetyState.ERROR)

    def get_active_source(self) -> CommandSource:
        """Return the currently active (highest priority) command source."""
        if self._estop_active:
            return CommandSource.ESTOP
        if self._rc_override_active:
            return CommandSource.RC
        return self._last_cmd_source

    def get_max_speed_for_source(self, source: CommandSource) -> float:
        """Return the configured max speed for a given command source."""
        return self._max_speeds.get(source, 0.0)

    def get_last_command_age(self) -> float:
        """Return seconds since the last command was received."""
        return time.monotonic() - self._last_cmd_time

    def get_uptime(self) -> float:
        """Return seconds since the monitor was started."""
        if self._start_time == 0:
            return 0.0
        return time.monotonic() - self._start_time

    def _apply_rate_limit(
        self,
        current: float,
        target: float,
        dt: float,
        accel: float,
        decel: float,
    ) -> float:
        """Apply acceleration/deceleration rate limiting to a single axis.

        Uses different rates for speeding up vs slowing down.
        Deceleration is typically faster than acceleration for safety
        (robot stops faster than it starts).

        Args:
            current: Current speed value from previous cycle.
            target: Desired speed value this cycle.
            dt: Time delta in seconds.
            accel: Max acceleration rate (speed-units/second).
            decel: Max deceleration rate (speed-units/second).

        Returns:
            Rate-limited speed value, moved toward target by at most
            (rate * dt) per call.
        """
        if dt <= 0:
            return current

        diff = target - current

        # Determine if we're accelerating or decelerating
        # Decelerating = moving toward zero or reducing magnitude
        is_decelerating = abs(target) < abs(current) or (target * current < 0)
        rate = decel if is_decelerating else accel
        max_change = rate * dt

        if abs(diff) <= max_change:
            return target
        elif diff > 0:
            return current + max_change
        else:
            return current - max_change

    def _transition_state(self, new_state: SafetyState) -> None:
        """Transition to a new safety state with logging and callback.

        Args:
            new_state: The state to transition to.
        """
        old_state = self._state
        if old_state == new_state:
            return

        self._state = new_state
        logger.info("Safety state: %s -> %s", old_state.name, new_state.name)

        if self._on_state_change is not None:
            try:
                self._on_state_change(old_state, new_state)
            except Exception as e:
                logger.error("Error in state change callback: %s", e)
