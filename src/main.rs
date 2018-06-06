#![no_std]

extern crate bluenrg;
extern crate bluetooth_hci as hci;
extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
extern crate embedded_hal;
#[macro_use(block)]
extern crate nb;
extern crate panic_semihosting;
extern crate stm32f30x;
extern crate stm32f30x_hal as hal;

use bluenrg::LocalVersionInfoExt;
use core::fmt::Debug;
use core::fmt::Write;
use cortex_m_semihosting::hio;
use hal::flash::FlashExt;
use hal::gpio::GpioExt;
use hal::rcc::RccExt;
use hal::time::U32Ext;

fn print_event<Out: Write>(out: &mut Out, event: hci::Event<bluenrg::BlueNRGEvent>) {
    match event {
        hci::Event::CommandComplete(cmd) => {
            writeln!(
                out,
                "Command complete; space left for {} packets",
                cmd.num_hci_command_packets
            ).unwrap();
            match cmd.return_params {
                hci::event::command::ReturnParameters::None => (),
                hci::event::command::ReturnParameters::ReadLocalVersion(v) => {
                    writeln!(out, "  hci_version = {}", v.hci_version).unwrap();
                    writeln!(out, "  hci_revision = {}", v.hci_revision).unwrap();
                    writeln!(out, "  lmp_version = {}", v.lmp_version).unwrap();
                    writeln!(out, "  manufacturer_name = {}", v.manufacturer_name).unwrap();
                    writeln!(out, "  lmp_subversion = {}", v.lmp_subversion).unwrap();

                    let bnrg = v.bluenrg_version();
                    writeln!(out, "  HW Version = {}", bnrg.hw_version).unwrap();
                    writeln!(
                        out,
                        "  FW Version = {}.{}.{}",
                        bnrg.major, bnrg.minor, bnrg.patch
                    ).unwrap();
                }
            }
        }
        hci::Event::CommandStatus(status) => {
            writeln!(out, "Command status: {:?}", status).unwrap();
        }
        hci::Event::Vendor(event) => {
            writeln!(out, "Vendor event: {:?}", event).unwrap();
        }
    }
}

fn print_error<Out: Write, E: Debug>(out: &mut Out, error: E) {
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
        let mut gpiob = peripherals.GPIOB.split(&mut rcc.ahb);
        let sck = gpiob.pb3.into_af5(&mut gpiob.moder, &mut gpiob.afrl);
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
            &mut rcc.apb2,
        );

        let data_ready = gpioa
            .pa0
            .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);
        let chip_select = gpioa
            .pa1
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        let reset_pin = gpioa
            .pa8
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        let mut tim6 = hal::timer::Timer::tim6(peripherals.TIM6, 200.hz(), clocks, &mut rcc.apb1);
        let mut rx_buffer: [u8; 128] = [0; 128];

        let mut bnrg = bluenrg::BlueNRG::new(&mut rx_buffer, chip_select, data_ready, reset_pin);
        bnrg.reset(&mut tim6, 200.hz());
        tim6.free();

        bnrg.with_spi(&mut spi, |controller| {
            block!(controller.read_local_version_information()).unwrap();
        });
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
        let mut stdout = hio::hstdout().unwrap();
        loop {
            match block!(bnrg.with_spi(&mut spi, |controller| controller.read())) {
                Ok(p) => {
                    let hci::host::uart::Packet::Event(e) = p;
                    print_event(&mut stdout, e.clone());
                    if let hci::Event::CommandComplete(cmd) = e {
                        if let hci::event::command::ReturnParameters::ReadLocalVersion(_) =
                            cmd.return_params
                        {
                            cortex_m::asm::wfi();
                        }
                    }
                }
                Err(e) => print_error(&mut stdout, e),
            }
        }
    });
}
