#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::{delay::DelayNs, i2c::I2c};
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
const CHIP_ID: u8 = 0x01;
const TUNE: u8 = 0x03; // FM Channel
const STATUSA: u8 = 0x12; // RSSI
const STATUSC: u8 = 0x14; // SNR
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

    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

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

    // wait chip ready
    wait_chip_ready(&mut i2c, I2C_ADDRESS, &mut timer);

    // FM
    fm_mode(&mut i2c, I2C_ADDRESS);

    // Dial mode VOL amd CH
    // VOL = 10 / CH = 10
    i2c_send_multibyte(&mut i2c, I2C_ADDRESS, GPIOCFG, 0b0000_0000_0000_1010u16);

    loop {
        // Read chipID
        let mut chip_id = 0u16;
        i2c_read_multibyte(&mut i2c, I2C_ADDRESS, CHIP_ID, &mut chip_id);
        info!("chipID: 0x{:04X}", chip_id);
        // Read STATUSA
        let mut statusa = 0u16;
        i2c_read_multibyte(&mut i2c, I2C_ADDRESS, STATUSA, &mut statusa);
        // RSSI
        let fm_rssi: u16 = (statusa & 0b0000_0001_1111_000u16) >> 3;
        let fm_rssi: i16 = -100 + (fm_rssi as i16) * 3;
        info!("RSSI: {}", fm_rssi);
        // SNR
        let mut statusc = 0u16;
        i2c_read_multibyte(&mut i2c, I2C_ADDRESS, STATUSC, &mut statusc);
        let snr: u16 = (statusc & 0b0001_1111_1100_0000u16) >> 6;
        info!("SNR: {}", snr);
        // FM freq
        let mut tune = 0u16;
        i2c_read_multibyte(&mut i2c, I2C_ADDRESS, TUNE, &mut tune);
        let fm_freq: u16 = tune & 0b0000_1111_1111_1111u16;
        let fm_freq = (fm_freq as f32) / 1000.0 * 50.0;
        let fm_freq_scaled = (fm_freq * 10.0) as u16;
        info!(
            "FM_FREQ: {}.{} MHz",
            fm_freq_scaled / 10,
            fm_freq_scaled % 10
        );
        // wait 1000ms
        timer.delay_ms(1000);
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

fn wait_chip_ready<T>(i2c: &mut T, i2c_address: u8, timer: &mut rp2040_hal::Timer)
where
    T: I2c,
{
    let mut ready = false;
    while !ready {
        let mut statusc = 0u16;
        i2c_read_multibyte(i2c, i2c_address, STATUSC, &mut statusc);
        let chiprdy: u16 = (statusc & 0b0010_0000_0000_0000u16) >> 13;
        info!("CHIPRDY: {}", chiprdy);
        if chiprdy == 1 {
            ready = true;
            info!("Chip is ready, calibration done.");
        } else {
            info!("Chip is not ready.");
            // wait 100ms
            timer.delay_ms(100);
        }
    }
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
