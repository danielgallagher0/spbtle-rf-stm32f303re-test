#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
extern crate embedded_hal;
extern crate ble;
extern crate bluenrg;
#[macro_use(block)]
extern crate nb;
extern crate stm32f30x;
extern crate stm32f30x_hal as hal;

use bluenrg::LocalVersionInfoExt;
use core::fmt::Debug;
use core::fmt::Write;
use cortex_m_semihosting::hio;
use hal::flash::FlashExt;
use hal::rcc::RccExt;
use hal::gpio::GpioExt;
use hal::time::U32Ext;
use embedded_hal::digital::OutputPin;
use embedded_hal::timer::CountDown;

fn print_event<Out: Write>(out: &mut Out, event: ble::Event) {
    match event {
        ble::Event::CommandComplete(cmd) => {
            writeln!(out, "Command complete; space left for {} packets", cmd.num_hci_command_packets).unwrap();
            match cmd.return_params {
                ble::ReturnParameters::None => (),
                ble::ReturnParameters::ReadLocalVersion(v) => {
                    writeln!(out, "  hci_version = {}", v.hci_version).unwrap();
                    writeln!(out, "  hci_revision = {}", v.hci_revision).unwrap();
                    writeln!(out, "  lmp_version = {}", v.lmp_version).unwrap();
                    writeln!(out, "  manufacturer_name = {}", v.manufacturer_name).unwrap();
                    writeln!(out, "  lmp_subversion = {}", v.lmp_subversion).unwrap();

                    let exploded = v.bluenrg_version();
                    writeln!(out, "  HW Version = {}", exploded.hw_version).unwrap();
                    writeln!(out, "  FW Version = {}.{}.{}", exploded.major, exploded.minor, exploded.patch).unwrap();
                }
            }
        },
    }
}

fn print_error<Out: Write, E: Debug>(out: &mut Out, error: bluenrg::Error<E>) {
    writeln!(out, "Error: {:?}", error).unwrap();
}

#[inline(never)]
fn main() {
    cortex_m::interrupt::free(|_cs| {
        // Enable I2C1
        let peripherals = stm32f30x::Peripherals::take().unwrap();
        peripherals.RCC.ahbenr.modify(|_, w| w.iopaen().set_bit());

        let mut rcc = peripherals.RCC.constrain();
        let mut gpioa = peripherals.GPIOA.split(&mut rcc.ahb);
        let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
        let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
        let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
        let clocks = rcc.cfgr.freeze(&mut peripherals.FLASH.constrain().acr);
        let mut spi = hal::spi::Spi::spi1(
            peripherals.SPI1,
            (sck, miso, mosi),
            embedded_hal::spi::Mode {
                polarity: embedded_hal::spi::Polarity::IdleLow,
                phase: embedded_hal::spi::Phase::CaptureOnFirstTransition,
            },
            1.mhz(),
            clocks,
            &mut rcc.apb2);

        let data_ready = gpioa.pa0.into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);
        let chip_select = gpioa.pa1.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
        let mut pa8 = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
        let mut tim6 = hal::timer::Timer::tim6(peripherals.TIM6, 200.hz(), clocks, &mut rcc.apb1);
        let mut rx_buffer: [u8; 128] = [0; 128];
        let mut bnrg = bluenrg::BlueNRG::new(&mut rx_buffer, chip_select, data_ready, &mut || {
            pa8.set_high();
            tim6.start(200.hz());
            block!(tim6.wait()).unwrap();

            pa8.set_low();
            tim6.start(200.hz());
            block!(tim6.wait()).unwrap();
        });
        tim6.free();

        let mut stdout = hio::hstdout().unwrap();
        bnrg.with_spi(&mut spi, |controller| {
            block!(ble::hci::read_local_version_information(controller)).unwrap();
        });
        // {
        // Ok(local_version) => {
        //     let version = local_version.bluenrg_version();
        //   writeln!(stdout, "Version Info (HW/FW): {}/{}.{}.{}", version.hw_version,
        //          version.major, version.minor, version.patch).unwrap();
        //            },
        //          Err(e) => writeln!(stdout, "Failed to get version: {:?}", e).unwrap(),
        //    }
        // BlueNRG_RST()
        // getBlueNRGVersion(&hwVersion, &fwVersion);
        // BlueNRG_RST()
        // aci_hal_write_config_data(PUBADDR)
        // aci_gatt_init()
        // aci_gap_init_idb05a1()
        // aci_gatt_update_char_value()
        // aci_gap_set_auth_requirement()
        // add services...
        // aci_hal_set_tx_power_level(1,4)
        loop {
            match block!(bnrg.with_spi(&mut spi, |controller| controller.read())) {
                Ok(e) => print_event(&mut stdout, e),
                Err(e) => print_error(&mut stdout, e),
            }

            cortex_m::asm::wfi();
        }
    });
}