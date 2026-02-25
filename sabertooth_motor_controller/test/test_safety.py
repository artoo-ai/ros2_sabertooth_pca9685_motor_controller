"""Unit tests for the safety monitor state machine.

Tests the SafetyMonitor class that enforces all safety constraints
on motor commands. This is the most critical module in the package.

Tests cover:
  - State machine transitions
  - Speed limiting per command source
  - Acceleration/deceleration limiting
  - E-stop behavior
  - Timeout behavior
  - Safety invariants (motors neutral in unsafe states)

Run with: pytest test/test_safety.py -v
"""

import time
import pytest
from sabertooth_motor_controller.safety_monitor import (
    SafetyMonitor,
    SafetyState,
    CommandSource,
)


@pytest.fixture
def monitor():
    """Create a safety monitor with short timeouts for testing."""
    m = SafetyMonitor(
        command_timeout_ms=100.0,       # 100ms for fast test cycles
        heartbeat_timeout_ms=500.0,
        max_speed_teleop=0.6,
        max_speed_autonomous=0.4,
        max_speed_rc=1.0,
        accel_limit=2.0,
        decel_limit=4.0,
        emergency_decel_limit=100.0,    # Near-instant for testing
        startup_delay_sec=0.0,          # No startup delay for tests
    )
    m.start()
    return m


@pytest.fixture
def monitor_with_delay():
    """Create a safety monitor WITH startup delay for testing init state."""
    m = SafetyMonitor(
        command_timeout_ms=100.0,
        startup_delay_sec=0.5,          # Half second startup delay
    )
    m.start()
    return m


# ===========================================================================
# State Machine Transitions
# ===========================================================================

class TestStateTransitions:
    """Test safety state machine transitions."""

    def test_starts_normal_without_delay(self, monitor):
        """With zero startup delay, should immediately be NORMAL."""
        monitor.check_timeouts()
        assert monitor.state == SafetyState.NORMAL

    def test_starts_initializing_with_delay(self, monitor_with_delay):
        """With startup delay, should start in INITIALIZING."""
        assert monitor_with_delay.state == SafetyState.INITIALIZING

    def test_initializing_to_normal_after_delay(self, monitor_with_delay):
        """Should transition to NORMAL after startup delay elapses."""
        time.sleep(0.6)  # Wait for startup delay
        monitor_with_delay.check_timeouts()
        assert monitor_with_delay.state == SafetyState.NORMAL

    def test_normal_to_timeout(self, monitor):
        """Should transition to TIMEOUT when no commands received."""
        monitor.check_timeouts()
        assert monitor.state == SafetyState.NORMAL
        time.sleep(0.15)  # Exceed 100ms timeout
        monitor.check_timeouts()
        assert monitor.state == SafetyState.TIMEOUT

    def test_timeout_to_normal_on_command(self, monitor):
        """Should return to NORMAL when command received after timeout."""
        monitor.check_timeouts()
        time.sleep(0.15)
        monitor.check_timeouts()
        assert monitor.state == SafetyState.TIMEOUT

        # Receive a new command
        monitor.command_received(CommandSource.TELEOP)
        monitor.check_timeouts()
        assert monitor.state == SafetyState.NORMAL

    def test_normal_to_estop(self, monitor):
        """E-stop should immediately transition to ESTOP."""
        monitor.check_timeouts()
        monitor.set_estop(True)
        assert monitor.state == SafetyState.ESTOP

    def test_estop_requires_release_and_reset(self, monitor):
        """Cannot leave ESTOP without both release AND reset."""
        monitor.check_timeouts()
        monitor.set_estop(True)
        assert monitor.state == SafetyState.ESTOP

        # Just releasing e-stop is not enough
        monitor.set_estop(False)
        assert monitor.state == SafetyState.ESTOP

        # Must also call reset
        result = monitor.reset_from_estop()
        assert result is True
        assert monitor.state == SafetyState.NORMAL

    def test_reset_fails_if_estop_still_active(self, monitor):
        """Reset should fail if e-stop signal is still active."""
        monitor.check_timeouts()
        monitor.set_estop(True)

        # Try to reset without releasing e-stop
        result = monitor.reset_from_estop()
        assert result is False
        assert monitor.state == SafetyState.ESTOP

    def test_normal_to_rc_override(self, monitor):
        """RC signals should transition to RC_OVERRIDE."""
        monitor.check_timeouts()
        monitor.set_rc_override(True)
        assert monitor.state == SafetyState.RC_OVERRIDE

    def test_rc_override_to_normal_on_signal_lost(self, monitor):
        """Should return to NORMAL when RC signal lost."""
        monitor.check_timeouts()
        monitor.set_rc_override(True)
        assert monitor.state == SafetyState.RC_OVERRIDE

        monitor.set_rc_override(False)
        assert monitor.state == SafetyState.NORMAL

    def test_estop_from_rc_override(self, monitor):
        """E-stop should work even during RC override."""
        monitor.check_timeouts()
        monitor.set_rc_override(True)
        monitor.set_estop(True)
        assert monitor.state == SafetyState.ESTOP

    def test_estop_from_timeout(self, monitor):
        """E-stop should work even during timeout."""
        monitor.check_timeouts()
        time.sleep(0.15)
        monitor.check_timeouts()
        assert monitor.state == SafetyState.TIMEOUT

        monitor.set_estop(True)
        assert monitor.state == SafetyState.ESTOP

    def test_error_state(self, monitor):
        """Hardware error should transition to ERROR."""
        monitor.check_timeouts()
        monitor.set_error()
        assert monitor.state == SafetyState.ERROR

    def test_reset_from_error(self, monitor):
        """Should be able to reset from ERROR state."""
        monitor.check_timeouts()
        monitor.set_error()
        result = monitor.reset_from_error()
        assert result is True
        assert monitor.state == SafetyState.NORMAL


# ===========================================================================
# Speed Limiting
# ===========================================================================

class TestSpeedLimiting:
    """Test per-source speed limiting."""

    def test_teleop_speed_limited(self, monitor):
        """Teleop commands should be clamped to max_speed_teleop."""
        monitor.check_timeouts()
        # Request 1.0, should be clamped to 0.6
        left, right, limited = monitor.process_speeds(
            1.0, 1.0, CommandSource.TELEOP, dt=1.0  # Large dt to skip accel limit
        )
        assert abs(left) <= 0.6 + 0.01
        assert abs(right) <= 0.6 + 0.01
        assert limited is True

    def test_autonomous_speed_limited(self, monitor):
        """Autonomous commands should be clamped to max_speed_autonomous."""
        monitor.check_timeouts()
        left, right, limited = monitor.process_speeds(
            1.0, 1.0, CommandSource.AUTONOMOUS, dt=1.0
        )
        assert abs(left) <= 0.4 + 0.01
        assert abs(right) <= 0.4 + 0.01
        assert limited is True

    def test_rc_full_speed(self, monitor):
        """RC commands should allow full speed (max_speed_rc=1.0)."""
        monitor.check_timeouts()
        left, right, limited = monitor.process_speeds(
            1.0, 1.0, CommandSource.RC, dt=1.0
        )
        assert left == pytest.approx(1.0, abs=0.01)
        assert right == pytest.approx(1.0, abs=0.01)

    def test_within_limit_not_limited(self, monitor):
        """Commands within limit should not be flagged as limited."""
        monitor.check_timeouts()
        left, right, limited = monitor.process_speeds(
            0.3, 0.3, CommandSource.TELEOP, dt=1.0
        )
        # 0.3 is within teleop limit of 0.6, but may be limited by accel
        assert abs(left) <= 0.6


# ===========================================================================
# Acceleration Limiting
# ===========================================================================

class TestAccelerationLimiting:
    """Test acceleration and deceleration rate limiting."""

    def test_acceleration_ramp(self, monitor):
        """Speed should ramp up gradually, not jump instantly."""
        monitor.check_timeouts()
        # From 0 to 0.6 with accel_limit=2.0 and dt=0.02 (50Hz)
        # max_change = 2.0 * 0.02 = 0.04 per cycle
        left, right, _ = monitor.process_speeds(
            0.6, 0.6, CommandSource.TELEOP, dt=0.02
        )
        assert left < 0.6  # Should not reach 0.6 in one step
        assert left == pytest.approx(0.04, abs=0.01)

    def test_deceleration_faster_than_acceleration(self, monitor):
        """Deceleration should be faster than acceleration."""
        monitor.check_timeouts()

        # Ramp up to some speed
        current = 0.0
        for _ in range(25):  # 25 cycles at 50Hz = 0.5 sec
            left, _, _ = monitor.process_speeds(
                0.6, 0.6, CommandSource.TELEOP, dt=0.02
            )
            current = left

        speed_before_decel = current

        # Now decelerate to 0
        left, _, _ = monitor.process_speeds(
            0.0, 0.0, CommandSource.TELEOP, dt=0.02
        )
        decel_delta = abs(speed_before_decel - left)

        # Deceleration delta should be larger than a single accel step
        single_accel_step = 2.0 * 0.02  # accel_limit * dt
        single_decel_step = 4.0 * 0.02  # decel_limit * dt
        assert decel_delta >= single_accel_step

    def test_gradual_ramp_to_target(self, monitor):
        """Speed should gradually reach target over multiple cycles."""
        monitor.check_timeouts()

        # Run 50 cycles at 50Hz (1 second)
        left = 0.0
        for _ in range(50):
            left, _, _ = monitor.process_speeds(
                0.5, 0.5, CommandSource.TELEOP, dt=0.02
            )

        # After 1 second with accel_limit=2.0, should reach 0.5
        assert left == pytest.approx(0.5, abs=0.05)


# ===========================================================================
# Safety Invariants (CRITICAL)
# ===========================================================================

class TestSafetyInvariants:
    """Test that motors are ALWAYS neutral in unsafe states.

    This is the most critical test class. These invariants must NEVER
    be violated - they protect people (including children) from the robot.
    """

    def test_neutral_in_estop(self, monitor):
        """Motors ALWAYS neutral when state is ESTOP."""
        monitor.check_timeouts()
        # Build up some speed
        for _ in range(50):
            monitor.process_speeds(0.6, 0.6, CommandSource.TELEOP, dt=0.02)

        # Engage e-stop
        monitor.set_estop(True)

        # Process should ramp toward zero
        for _ in range(100):
            left, right, _ = monitor.process_speeds(
                1.0, 1.0, CommandSource.TELEOP, dt=0.02
            )

        # After enough cycles, should be at zero
        assert left == pytest.approx(0.0, abs=0.01)
        assert right == pytest.approx(0.0, abs=0.01)

    def test_neutral_in_timeout(self, monitor):
        """Motors ALWAYS neutral when state is TIMEOUT."""
        monitor.check_timeouts()
        time.sleep(0.15)
        monitor.check_timeouts()
        assert monitor.state == SafetyState.TIMEOUT

        left, right, limited = monitor.process_speeds(
            1.0, 1.0, CommandSource.TELEOP, dt=0.02
        )
        # Should be ramping to zero, not accepting commands
        assert limited is True

    def test_neutral_in_initializing(self, monitor_with_delay):
        """Motors ALWAYS neutral when state is INITIALIZING."""
        assert monitor_with_delay.state == SafetyState.INITIALIZING

        left, right, limited = monitor_with_delay.process_speeds(
            1.0, 1.0, CommandSource.TELEOP, dt=0.02
        )
        assert limited is True
        # Should be at zero (no speed built up in INITIALIZING)
        assert left == pytest.approx(0.0, abs=0.01)
        assert right == pytest.approx(0.0, abs=0.01)

    def test_neutral_in_error(self, monitor):
        """Motors ALWAYS neutral when state is ERROR."""
        monitor.check_timeouts()
        monitor.set_error()

        left, right, limited = monitor.process_speeds(
            1.0, 1.0, CommandSource.TELEOP, dt=0.02
        )
        assert limited is True

    def test_commands_rejected_during_initializing(self, monitor_with_delay):
        """No commands should produce motor output during initialization."""
        for _ in range(10):
            left, right, _ = monitor_with_delay.process_speeds(
                1.0, 1.0, CommandSource.TELEOP, dt=0.02
            )
        # Speed should still be zero
        assert left == pytest.approx(0.0, abs=0.01)
        assert right == pytest.approx(0.0, abs=0.01)


# ===========================================================================
# Command Source Priority
# ===========================================================================

class TestCommandSourcePriority:
    """Test command source priority reporting."""

    def test_estop_highest_priority(self, monitor):
        """E-stop should be reported as active source."""
        monitor.set_estop(True)
        assert monitor.get_active_source() == CommandSource.ESTOP

    def test_rc_override_priority(self, monitor):
        """RC should be reported as active source when override active."""
        monitor.set_rc_override(True)
        assert monitor.get_active_source() == CommandSource.RC

    def test_teleop_source(self, monitor):
        """Teleop should be reported when last command was teleop."""
        monitor.command_received(CommandSource.TELEOP)
        assert monitor.get_active_source() == CommandSource.TELEOP

    def test_autonomous_source(self, monitor):
        """Autonomous should be reported when last command was autonomous."""
        monitor.command_received(CommandSource.AUTONOMOUS)
        assert monitor.get_active_source() == CommandSource.AUTONOMOUS


# ===========================================================================
# Utility Methods
# ===========================================================================

class TestUtilities:
    """Test utility methods."""

    def test_uptime(self, monitor):
        """Uptime should increase over time."""
        time.sleep(0.1)
        assert monitor.get_uptime() >= 0.1

    def test_command_age(self, monitor):
        """Command age should reflect time since last command."""
        monitor.command_received(CommandSource.TELEOP)
        time.sleep(0.1)
        assert monitor.get_last_command_age() >= 0.1

    def test_max_speed_for_source(self, monitor):
        """Should return correct max speed for each source."""
        assert monitor.get_max_speed_for_source(CommandSource.TELEOP) == 0.6
        assert monitor.get_max_speed_for_source(CommandSource.AUTONOMOUS) == 0.4
        assert monitor.get_max_speed_for_source(CommandSource.RC) == 1.0
        assert monitor.get_max_speed_for_source(CommandSource.ESTOP) == 0.0
