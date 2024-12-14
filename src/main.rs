#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::i2c::I2c;
use hal::pac;
use panic_probe as _;
use rp2040_hal::{self as hal, fugit::HertzU32, Clock};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000;

// I2C Device Address
const I2C_ADDRESS: u8 = 0x35;

// Register Address
const AMSYSCFG: u8 = 0x16; // AM/FM mode Control / Audio Gain Selection
const GPIOCFG: u8 = 0x1D; // Vol Pin Mode Selection, CH Pin Mode Selection
const USERSTARTCH: u8 = 0x2F; // User band start channel, only effect when USERBAND=1
const USERGUARD: u8 = 0x30; // User band guard number, only effect when USERBAND=1
const USERCHANNUM: u8 = 0x31; // User band channel number, only effect when USERBAND=1

#[rp2040_hal::entry]
fn main() -> ! {
    info!("Program start!");
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

    // FM
    fm_mode(&mut i2c, I2C_ADDRESS);

    // Dial mode VOL amd CH
    // VOL = 10 / CH = 10
    i2c_send_multibyte(&mut i2c, I2C_ADDRESS, GPIOCFG, 0b0000_0000_0000_1010u16);

    loop {
        cortex_m::asm::wfi();
    }
}

fn i2c_send_multibyte<T>(i2c: &mut T, i2c_address: u8, register_address: u8, write_data: u16)
where
    T: I2c,
{
    // Transmission Data(3 bytes)
    let mut transmisson_data = [0u8; 3];
    // Vol Pin Mode Selection, CH Pin Mode Selection
    transmisson_data[0..1].copy_from_slice(&register_address.to_be_bytes());
    // VOL = 10 / CH = 10
    transmisson_data[1..3].copy_from_slice(&write_data.to_be_bytes());
    // Send Data
    i2c.write(i2c_address, &transmisson_data).unwrap();
}

fn i2c_read_multibyte<T>(i2c: &mut T, i2c_address: u8, register_address: u8, read_data: &mut u16)
where
    T: I2c,
{
    // Received data( 2 bytes)
    let mut received_data = [0u8; 2];
    // Transmission Data(1 bytes)
    let mut transmisson_data = [0u8; 1];
    transmisson_data[0..1].copy_from_slice(&register_address.to_be_bytes());
    // Read Data
    i2c.write_read(i2c_address, &transmisson_data, &mut received_data)
        .unwrap();
    // Set received data
    *read_data = u16::from(received_data[0]) << 8 | u16::from(received_data[1]);
}

fn fm_mode<T>(i2c: &mut T, i2c_address: u8)
where
    T: I2c,
{
    // Read AMSYSCFG
    let mut amsyscfg_mode: u16 = 0;
    i2c_read_multibyte(i2c, i2c_address, AMSYSCFG, &mut amsyscfg_mode);
    debug!("Current AMSYSCFG 0b{:016b}", amsyscfg_mode);
    // AM_FM(15 Bit) 0 = FM Mode
    amsyscfg_mode = amsyscfg_mode & !(1u16 << 15);
    // USERBAND(14 Bit) 1 = Use user-defined band
    amsyscfg_mode = amsyscfg_mode | 0b0100_0000_0000_0000u16;
    debug!("Send AMSYSCFG 0b{:016b}", amsyscfg_mode);
    i2c_send_multibyte(i2c, i2c_address, AMSYSCFG, amsyscfg_mode);
    // 76MHz = 1520 * 50kHZ (1520= 5F0h)
    i2c_send_multibyte(i2c, i2c_address, USERSTARTCH, 0b0000_0101_1111_0000u16);
    // 23ch  2kohm
    i2c_send_multibyte(i2c, i2c_address, USERGUARD, 0b0000_0000_0001_0111u16);
    // 181ch  76MHz - 94MHz  100KHz/step  8kohm
    i2c_send_multibyte(i2c, i2c_address, USERCHANNUM, 0b0000_0000_1011_0101u16);
}
