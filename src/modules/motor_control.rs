use pid::Pid;

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum Drive {
    Coast,
    Forward,
    Reverse,
}

#[derive(Copy, Clone)]
pub struct MotorCommand {
    pub drive: Drive,
    pub duty: u16,
}

impl MotorCommand {
    pub fn coast() -> Self {
        Self {
            drive: Drive::Coast,
            duty: 0,
        }
    }
}

/// H-bridge driver for Zynq-7000 motor channels.
///
/// Uses memory-mapped AXI registers (in the PL) or TTC channels to control
/// forward and reverse duty cycles.
pub struct HBridgePwm {
    ch1_duty_addr: *mut u32,
    ch2_duty_addr: *mut u32,
    max: u16,
}

// Safety: The register addresses point to valid memory-mapped hardware peripheral registers.
unsafe impl Send for HBridgePwm {}

impl HBridgePwm {
    /// Create an H-bridge driver with forward (ch1) and reverse (ch2) duty register addresses.
    ///
    /// # Safety
    /// The caller must ensure the register addresses are valid MMIO pointers.
    pub unsafe fn new(ch1_duty_addr: *mut u32, ch2_duty_addr: *mut u32, max_duty: u16) -> Self {
        let mut dev = Self {
            ch1_duty_addr,
            ch2_duty_addr,
            max: max_duty,
        };
        dev.apply(MotorCommand::coast());
        dev
    }

    pub fn max_duty(&self) -> u16 {
        self.max
    }

    pub fn apply(&mut self, cmd: MotorCommand) {
        unsafe {
            match cmd.drive {
                Drive::Coast => {
                    core::ptr::write_volatile(self.ch1_duty_addr, 0);
                    core::ptr::write_volatile(self.ch2_duty_addr, 0);
                }
                Drive::Forward => {
                    core::ptr::write_volatile(self.ch1_duty_addr, cmd.duty as u32);
                    core::ptr::write_volatile(self.ch2_duty_addr, 0);
                }
                Drive::Reverse => {
                    core::ptr::write_volatile(self.ch1_duty_addr, 0);
                    core::ptr::write_volatile(self.ch2_duty_addr, cmd.duty as u32);
                }
            }
        }
    }
}

/// PID-based speed controller producing an H-bridge command.
pub struct MotorPid {
    pid: Pid<f32>,
    min_duty: u16,
    max_duty: u16,
    deadband: f32,
}

impl MotorPid {
    pub fn new(setpoint_cps: f32, min_duty: u16, max_duty: u16) -> Self {
        let mut pid = Pid::new(setpoint_cps, 1.0_f32);
        pid.p(0.0004_f32, 1.0_f32).i(0.00002_f32, 1.0_f32).d(0.0_f32, 1.0_f32);

        Self {
            pid,
            min_duty,
            max_duty,
            deadband: 0.02,
        }
    }

    pub fn set_setpoint(&mut self, setpoint_cps: f32) {
        self.pid.setpoint(setpoint_cps);
    }

    pub fn update(&mut self, measured_cps: f32) -> MotorCommand {
        let out = self.pid.next_control_output(measured_cps).output;

        if out.abs() < self.deadband {
            return MotorCommand::coast();
        }

        let duty_span = self.max_duty.saturating_sub(self.min_duty) as f32;
        let duty = (self.min_duty as f32 + out.abs().min(1.0) * duty_span) as u16;

        let drive = if out >= 0.0 { Drive::Forward } else { Drive::Reverse };
        MotorCommand { drive, duty }
    }
}
