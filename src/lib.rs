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

use bluenrg::hal::Commands;
use bluenrg::LocalVersionInfoExt;
use core::fmt::Debug;
use core::fmt::Write;
use cortex_m_semihosting::hio;
use hal::time::U32Ext;
use hci::host::uart::Hci;
use hci::host::Hci as Host;

fn print_event<Out: Write>(out: &mut Out, event: hci::Event<bluenrg::event::BlueNRGEvent>) {
    match event {
        hci::Event::CommandComplete(cmd) => {
            writeln!(
                out,
                "Command complete; space left for {} packets",
                cmd.num_hci_command_packets
            ).unwrap();
            match cmd.return_params {
                hci::event::command::ReturnParameters::ReadLocalVersionInformation(v) => {
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
                _ => (),
            }
        }
        hci::Event::CommandStatus(status) => {
            writeln!(out, "Command status: {:?}", status).unwrap();
        }
        hci::Event::Vendor(event) => {
            writeln!(out, "Vendor event: {:?}", event).unwrap();
        }
        _ => (),
    }
}

fn print_error<Out: Write, E: Debug>(out: &mut Out, error: E) {
    writeln!(out, "Error: {:?}", error).unwrap();
}

pub struct EventLoop<'a> {
    state: State,
    data: ProgramState<'a>,
}

impl<'a> EventLoop<'a> {
    pub fn new(
        bnrg: &'a mut BlueNRG<'a>,
        tim6: hal::timer::Timer<stm32f30x::TIM6>,
        spi: Spi,
    ) -> EventLoop<'a> {
        EventLoop {
            state: State::Initializing,
            data: ProgramState { bnrg, tim6, spi },
        }
    }

    pub fn run(&mut self) {
        loop {
            self.state.act(&mut self.data);
            self.state = self.state.react(&mut self.data);
        }
    }
}

type Spi = hal::spi::Spi<
    stm32f30x::SPI1,
    (
        hal::gpio::gpiob::PB3<hal::gpio::AF5>,
        hal::gpio::gpioa::PA6<hal::gpio::AF5>,
        hal::gpio::gpioa::PA7<hal::gpio::AF5>,
    ),
>;

type BlueNRG<'a> = bluenrg::BlueNRG<
    'a,
    Spi,
    hal::gpio::gpioa::PA1<hal::gpio::Output<hal::gpio::PushPull>>,
    hal::gpio::gpioa::PA8<hal::gpio::Output<hal::gpio::PushPull>>,
    hal::gpio::gpioa::PA0<hal::gpio::Input<hal::gpio::PullDown>>,
>;

struct ProgramState<'a> {
    bnrg: &'a mut BlueNRG<'a>,
    tim6: hal::timer::Timer<stm32f30x::TIM6>,
    spi: Spi,
}

#[derive(Copy, Clone)]
enum State {
    Initializing,
    SettingAddress,
    Complete,
}

impl State {
    fn act<'a>(&self, ps: &mut ProgramState<'a>) {
        match self {
            &State::Initializing => {
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(c.read_local_version_information()).unwrap()
                });
            }
            &State::SettingAddress => {
                ps.bnrg.reset(&mut ps.tim6, 200.hz());
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(
                        c.write_config_data(
                            &bluenrg::hal::ConfigData::public_address(hci::BdAddr([
                                0x12, 0x34, 0x00, 0xE1, 0x80, 0x02,
                            ])).build(),
                        )
                    ).unwrap()
                });
            }
            &State::Complete => {
                cortex_m::asm::wfi();
            }
        }
    }

    fn react<'a>(&self, ps: &mut ProgramState<'a>) -> Self {
        let mut stdout = hio::hstdout().unwrap();
        match block!(ps.bnrg.with_spi(&mut ps.spi, |c| c.read())) {
            Ok(p) => {
                let hci::host::uart::Packet::Event(e) = p;
                print_event(&mut stdout, e.clone());
                self.react_to_event(ps, e)
            }
            Err(e) => {
                print_error(&mut stdout, e);
                *self
            }
        }
    }

    fn react_to_event<'a>(
        &self,
        _ps: &ProgramState<'a>,
        event: hci::event::Event<bluenrg::event::BlueNRGEvent>,
    ) -> Self {
        match self {
            &State::Initializing => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::ReadLocalVersionInformation(_) =
                        cmd.return_params
                    {
                        return State::SettingAddress;
                    }
                }
            }
            &State::SettingAddress => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::HalWriteConfigData(_),
                    ) = cmd.return_params
                    {
                        return State::Complete;
                    }
                }
            }
            &State::Complete => (),
        }

        *self
    }
}

/*
// aci_gatt_init()
// aci_gap_init_idb05a1()
// aci_gatt_update_char_value()
// aci_gap_set_auth_requirement()
// add services...
// aci_hal_set_tx_power_level(1,4)
*/
