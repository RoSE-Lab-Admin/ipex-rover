"""Motion primitive base class and failure codes (ROS-free).

A primitive declares its parameters (PARAMS) and implements run(ctx).
Everything common is handled by the primitive server, not the primitive:
parameter validation, timing, energy (t and E), feedback, cancel, all stop,
operator override, timeouts, safe stop on exit, and run logging.
"""


class Codes:
    """failure_code values in the Primitive action result."""
    INVALID_PARAMS = 'INVALID_PARAMS'          # unknown / missing / out-of-range parameter
    UNSUPPORTED = 'UNSUPPORTED'                # primitive or parameter not implemented on this rover yet
    PRECONDITION = 'PRECONDITION'              # e.g. arm not homed, no odometry, energy meter down
    BUSY = 'BUSY'                              # another primitive is already running
    TIMEOUT = 'TIMEOUT'                        # did not finish within its time limit
    STALL = 'STALL'                            # drive current at the stall threshold
    ESTOP = 'ESTOP'                            # all stop (B / /ipex/all_stop)
    OPERATOR_OVERRIDE = 'OPERATOR_OVERRIDE'    # operator used the hand controller
    CANCELED = 'CANCELED'                      # client canceled the goal
    HARDWARE = 'HARDWARE'                      # required telemetry missing mid-run
    INTERNAL = 'INTERNAL'                      # unexpected software error

    ALL = (INVALID_PARAMS, UNSUPPORTED, PRECONDITION, BUSY, TIMEOUT, STALL,
           ESTOP, OPERATOR_OVERRIDE, CANCELED, HARDWARE, INTERNAL)


class PrimitiveFailure(Exception):
    """Raise from run() (or ctx) to end the primitive with a failure code."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.message = message


class Primitive:
    """Base class. Subclasses set NAME, SUMMARY, PARAMS and implement run()."""

    NAME = ''
    SUMMARY = ''
    PARAMS = []               # list of params.Param
    CAPABLE = True            # False -> always fails with UNSUPPORTED
    STOPS_WHEN = ''           # documentation: normal completion condition
    FAILS_WHEN = ''           # documentation: failure conditions

    def __init__(self, params, config):
        self.params = params
        self.config = config

    def run(self, ctx):
        """Do the motion. Return a dict of outputs (stored in outputs_yaml).

        Use ctx.sleep()/ctx.check() in every loop so cancel / all stop /
        override / timeout take effect promptly.
        """
        raise NotImplementedError

    def cleanup(self, ctx):
        """Always called after run() (success or failure). Stop anything the
        primitive started. Drive is stopped by the server regardless."""

    def reject_unsupported_params(self):
        """Fail with UNSUPPORTED if an unimplemented parameter was given."""
        for spec in self.PARAMS:
            if spec.unsupported and self.params.get(spec.name) is not None:
                raise PrimitiveFailure(
                    Codes.UNSUPPORTED,
                    f"parameter '{spec.name}' is not supported yet: {spec.help}")
