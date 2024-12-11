#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_hal::i2c::I2c;
use hal::pac;
use panic_probe as _;
use rp2040_hal::{self as hal, fugit::HertzU32, Clock};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Register Address
const GPIOCFG: u8 = 0x1D; // Vol Pin Mode Selection, CH Pin Mode Selection

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin = pins
        .gpio14
        .into_pull_up_input()
        .into_function::<hal::gpio::FunctionI2C>();
    let scl_pin = pins
        .gpio15
        .into_pull_up_input()
        .into_function::<hal::gpio::FunctionI2C>();

    let i2c_clock: HertzU32 = HertzU32::from_raw(400_000).into();
    let peripheral_clock = clocks.peripheral_clock.freq();

    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        i2c_clock,
        &mut pac.RESETS,
        peripheral_clock,
    );

    // I2C Device Address
    let i2c_address = 0x35u8;

    // Transmission Data(3 bytes)
    let mut data = [0u8; 3];
    // Vol Pin Mode Selection, CH Pin Mode Selection
    data[0..1].copy_from_slice(&GPIOCFG.to_be_bytes());
    // VOL = 10 / CH = 10
    data[1..3].copy_from_slice(&0b0000000000001010u16.to_be_bytes());
    // Send Data
    i2c.write(i2c_address, &data).unwrap();

    loop {
        cortex_m::asm::wfi();
    }
}
