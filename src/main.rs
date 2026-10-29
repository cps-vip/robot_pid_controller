#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use panic_halt as _;

use zynq7000_hal::{
    Slcr,
    clocks::{
        Clocks,
        fpga::{self, FpgaClockConfig},
    },
    gpio::GpioPins,
    gtc::GlobalTimerCounter,
    pl,
    time::Hertz,
    uart::{ClockConfig, Config, Uart},
};
use zynq7000_rt as _;

mod modules;

// PYNQ-Z2 onboard PS reference crystal oscillator is 50 MHz (PCW_CRYSTAL_PERIPHERAL_FREQMHZ = 50).
const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(50_000_000);
// PYNQ-Z2 PL master fabric clock (FCLK_CLK0..3 sourced from IO PLL) is 100 MHz.
const PL_MASTER_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(100_000_000);

const CONTROL_PERIOD_MS: u64 = 10;
const TELEMETRY_PERIOD_MS: u64 = 100;

const TARGET_CPS: f32 = 4000.0;
const MIN_DUTY: u16 = 10_000;
const MAX_DUTY: u16 = 65_535;

// TODO: [HARDWARE/VIVADO INTEGRATION REMINDER]
// The AXI addresses below are placeholder offsets in the PL (Programmable Logic) address space (M_AXI_GP0).
// When your Vivado block design is finalized:
// 1. Open your Vivado project -> Block Design -> "Address Editor" tab.
// 2. Note the assigned base addresses for your motor PWM IP core (e.g. AXI Timer / custom PWM)
//    and encoder counter IP core (e.g. AXI Quadrature Encoder / custom counter).
// 3. Update the base addresses and channel offsets below to match the memory map:
//    - Left/Right Encoder counter registers
//    - Left/Right H-Bridge Forward/Reverse duty cycle registers
pub const AXI_ENCODER_LEFT_ADDR: *const u32 = 0x4000_0000 as *const u32;
pub const AXI_ENCODER_RIGHT_ADDR: *const u32 = 0x4000_0004 as *const u32;

pub const AXI_PWM_LEFT_FWD_ADDR: *mut u32 = 0x4000_0010 as *mut u32;
pub const AXI_PWM_LEFT_REV_ADDR: *mut u32 = 0x4000_0014 as *mut u32;
pub const AXI_PWM_RIGHT_FWD_ADDR: *mut u32 = 0x4000_0018 as *mut u32;
pub const AXI_PWM_RIGHT_REV_ADDR: *mut u32 = 0x4000_001C as *mut u32;

/// Entry point which calls the embassy main method on Zynq-7000 Cortex-A9.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    // Initialize MMU, L2 cache, GIC, and PS-to-PL level shifters while keeping PL in reset
    // until the 100 MHz FPGA fabric clocks (FCLK0..3) are configured.
    let mut hal_config = zynq7000_hal::Config::default();
    hal_config.deassert_pl_reset = false;
    let periphs = zynq7000_hal::init(hal_config).unwrap();

    // Hold PL in reset while configuring FCLK0..3 to 100 MHz, then release PL reset
    // so AXI MMIO peripherals in the FPGA fabric have an active clock before any access.
    pl::assert_reset();
    let fpga_clk_cfg =
        FpgaClockConfig::calculate(PS_CLOCK_FREQUENCY, [PL_MASTER_CLOCK_FREQUENCY; 4]).unwrap();
    unsafe {
        Slcr::with(|_| {
            fpga::configure(fpga_clk_cfg);
        });
    }
    pl::deassert_reset();

    let clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    let gpio_pins = GpioPins::new(periphs.gpio);

    // Set up global timer counter and embassy time driver (GTC).
    let gtc = GlobalTimerCounter::new(periphs.gtc, clocks.arm_clocks());
    zynq7000_hal::time_driver_gtc::init(clocks.arm_clocks(), gtc);

    // QEI encoders (memory-mapped registers from FPGA logic)
    let mut enc_left = unsafe { modules::encoder::Encoder::new(AXI_ENCODER_LEFT_ADDR) };
    let mut enc_right = unsafe { modules::encoder::Encoder::new(AXI_ENCODER_RIGHT_ADDR) };

    // Setup UART0 on MIO 15 (TX) and MIO 14 (RX), wired to the onboard FTDI USB-UART on PYNQ-Z2.
    let uart_clk_config = ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115_200)
        .unwrap()
        .0;
    let uart = Uart::new_with_mio_for_uart_0(
        periphs.uart_0,
        Config::new_with_clk_config(uart_clk_config),
        (gpio_pins.mio.mio15, gpio_pins.mio.mio14),
    )
    .unwrap();
    let (tx, rx) = uart.split();
    let mut serial = modules::serial::SerialInterface::new(tx, rx);

    // Left and Right motor PWM H-Bridges
    let mut left_hbridge = unsafe {
        modules::motor_control::HBridgePwm::new(
            AXI_PWM_LEFT_FWD_ADDR,
            AXI_PWM_LEFT_REV_ADDR,
            MAX_DUTY,
        )
    };
    let mut right_hbridge = unsafe {
        modules::motor_control::HBridgePwm::new(
            AXI_PWM_RIGHT_FWD_ADDR,
            AXI_PWM_RIGHT_REV_ADDR,
            MAX_DUTY,
        )
    };

    let mut left_pid =
        modules::motor_control::MotorPid::new(TARGET_CPS, MIN_DUTY, left_hbridge.max_duty());
    let mut right_pid =
        modules::motor_control::MotorPid::new(TARGET_CPS, MIN_DUTY, right_hbridge.max_duty());

    let dt_s = (CONTROL_PERIOD_MS as f32) / 1000.0;
    let mut tick = Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS));
    let mut next_telemetry = Instant::now();
    let mut line = LineBuffer::new();

    loop {
        tick.next().await;

        // Apply any pending setpoint updates from UART
        while let Some(b) = serial.try_read() {
            if let Some(cmd) = line.push(b) {
                match cmd {
                    Command::SetTargetCpsAll(t) => {
                        left_pid.set_setpoint(t as f32);
                        right_pid.set_setpoint(t as f32);
                    }
                    Command::SetTargetCpsLeft(t) => left_pid.set_setpoint(t as f32),
                    Command::SetTargetCpsRight(t) => right_pid.set_setpoint(t as f32),
                }
            }
        }

        // Convert delta counts per period into counts per second
        let left_cps = (enc_left.delta_counts() as f32) / dt_s;
        let right_cps = (enc_right.delta_counts() as f32) / dt_s;

        // Compute drive commands
        let left_cmd = left_pid.update(left_cps);
        let right_cmd = right_pid.update(right_cps);

        left_hbridge.apply(left_cmd);
        right_hbridge.apply(right_cmd);

        if Instant::now() >= next_telemetry {
            next_telemetry += Duration::from_millis(TELEMETRY_PERIOD_MS);

            let mut buf = [0u8; 64];
            let n = build_telemetry_line(&mut buf, left_cps as i32, right_cps as i32);
            serial.write(&buf[..n]);
        }
    }
}

fn build_telemetry_line(buf: &mut [u8; 64], left_cps: i32, right_cps: i32) -> usize {
    let mut i = 0usize;

    i = push_bytes(buf, i, b"L:");
    i = push_i32(buf, i, left_cps);
    i = push_bytes(buf, i, b" R:");
    i = push_i32(buf, i, right_cps);
    i = push_bytes(buf, i, b"\r\n");

    i
}

fn push_bytes<const N: usize>(buf: &mut [u8; N], i: usize, s: &[u8]) -> usize {
    let copy_len = s.len().min(N.saturating_sub(i));
    buf[i..i + copy_len].copy_from_slice(&s[..copy_len]);
    i + copy_len
}

fn push_i32<const N: usize>(buf: &mut [u8; N], mut i: usize, mut v: i32) -> usize {
    if i >= N {
        return i;
    }
    if v == 0 {
        buf[i] = b'0';
        return i + 1;
    }

    if v < 0 {
        buf[i] = b'-';
        i += 1;
        v = -v;
        if i >= N {
            return i;
        }
    }

    // write digits to a small stack buffer in reverse
    let mut tmp = [0u8; 11];
    let mut n = 0usize;
    let mut x = v as u32;
    while x != 0 && n < tmp.len() {
        tmp[n] = b'0' + (x % 10) as u8;
        x /= 10;
        n += 1;
    }

    // copy reversed into output
    while n != 0 && i < N {
        n -= 1;
        buf[i] = tmp[n];
        i += 1;
    }

    i
}

enum Command {
    SetTargetCpsAll(i32),
    SetTargetCpsLeft(i32),
    SetTargetCpsRight(i32),
}

struct LineBuffer {
    buf: [u8; 48],
    len: usize,
}

impl LineBuffer {
    fn new() -> Self {
        Self { buf: [0; 48], len: 0 }
    }

    fn push(&mut self, b: u8) -> Option<Command> {
        match b {
            b'\r' => None,
            b'\n' => {
                let cmd = parse_command(&self.buf[..self.len]);
                self.len = 0;
                cmd
            }
            _ => {
                if self.len < self.buf.len() {
                    self.buf[self.len] = b;
                    self.len += 1;
                }
                None
            }
        }
    }
}

// Supported commands (ASCII, newline-terminated):
// - "T <int>"  : set target CPS for both motors
// - "TL <int>" : set target CPS for left motor
// - "TR <int>" : set target CPS for right motor
fn parse_command(line: &[u8]) -> Option<Command> {
    let line = trim_ascii(line);
    if line.is_empty() {
        return None;
    }

    let (head, rest) = split_once_space(line);
    let value = parse_i32(trim_ascii(rest))?;

    match head {
        b"T" => Some(Command::SetTargetCpsAll(value)),
        b"TL" => Some(Command::SetTargetCpsLeft(value)),
        b"TR" => Some(Command::SetTargetCpsRight(value)),
        _ => None,
    }
}

fn trim_ascii(mut s: &[u8]) -> &[u8] {
    while let Some((&b, tail)) = s.split_first() {
        if b == b' ' || b == b'\t' {
            s = tail;
        } else {
            break;
        }
    }
    while let Some((&b, head)) = s.split_last() {
        if b == b' ' || b == b'\t' {
            s = head;
        } else {
            break;
        }
    }
    s
}

fn split_once_space(s: &[u8]) -> (&[u8], &[u8]) {
    for (i, &b) in s.iter().enumerate() {
        if b == b' ' || b == b'\t' {
            return (&s[..i], &s[i + 1..]);
        }
    }
    (s, &[])
}

fn parse_i32(s: &[u8]) -> Option<i32> {
    if s.is_empty() {
        return None;
    }

    let (neg, s) = match s[0] {
        b'-' => (true, &s[1..]),
        b'+' => (false, &s[1..]),
        _ => (false, s),
    };

    if s.is_empty() {
        return None;
    }

    let mut val: i32 = 0;
    for &b in s {
        if !b.is_ascii_digit() {
            return None;
        }
        let digit = (b - b'0') as i32;
        val = val.checked_mul(10)?.checked_add(digit)?;
    }

    if neg {
        Some(-val)
    } else {
        Some(val)
    }
}
