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

use bluenrg::gap::Commands as GapCommands;
use bluenrg::gatt::Commands as GattCommands;
use bluenrg::hal::Commands as HalCommands;
use bluenrg::LocalVersionInfoExt;
use core::fmt::Debug;
use core::fmt::Write;
use cortex_m_semihosting::hio;
use hal::time::U32Ext;
use hci::host::uart::Hci;
use hci::host::Hci as Host;

const ACC_SERVICE_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0x02, 0x36, 0x6e, 0x80, 0xcf, 0x3a, 0x11, 0xe1, 0x9a, 0xb4, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);
const ACC_FREE_FALL_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0xe2, 0x3e, 0x78, 0xa0, 0xcf, 0x4a, 0x11, 0xe1, 0x8f, 0xfc, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);
const ACC_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0x34, 0x0a, 0x1b, 0x80, 0xcf, 0x4b, 0x11, 0xe1, 0xac, 0x36, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);

const ENVIRONMENTAL_SENSOR_SERVICE_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0x42, 0x82, 0x1a, 0x40, 0xe4, 0x77, 0x11, 0xe2, 0x82, 0xd0, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);
const TEMPERATURE_CHARACTERISTIC_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0xa3, 0x2e, 0x55, 0x20, 0xe4, 0x77, 0x11, 0xe2, 0xa9, 0xe3, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);
const TEMPERATURE_DESCRIPTOR_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid16(0x2904);
const PRESSURE_CHARACTERISTIC_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0xcd, 0x20, 0xc4, 0x80, 0xe4, 0x8b, 0x11, 0xe2, 0x84, 0x0b, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);
const PRESSURE_DESCRIPTOR_UUID: bluenrg::gatt::Uuid = TEMPERATURE_DESCRIPTOR_UUID;
const HUMIDITY_CHARACTERISTIC_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0x01, 0xc5, 0x0b, 0x60, 0xe4, 0x8c, 0x11, 0xe2, 0xa0, 0x73, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);
const HUMIDITY_DESCRIPTOR_UUID: bluenrg::gatt::Uuid = TEMPERATURE_DESCRIPTOR_UUID;

const TIME_SERVICE_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0x08, 0x36, 0x6e, 0x80, 0xcf, 0x3a, 0x11, 0xe1, 0x9a, 0xb4, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);
const TIME_CHARACTERISTIC_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0x09, 0x36, 0x6e, 0x80, 0xcf, 0x3a, 0x11, 0xe1, 0x9a, 0xb4, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);
const MINUTE_CHARACTERISTIC_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0x0a, 0x36, 0x6e, 0x80, 0xcf, 0x3a, 0x11, 0xe1, 0x9a, 0xb4, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);

const LED_SERVICE_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0x0b, 0x36, 0x6e, 0x80, 0xcf, 0x3a, 0x11, 0xe1, 0x9a, 0xb4, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);
const LED_CHARACTERISTIC_UUID: bluenrg::gatt::Uuid = bluenrg::gatt::Uuid::Uuid128([
    0x0c, 0x36, 0x6e, 0x80, 0xcf, 0x3a, 0x11, 0xe1, 0x9a, 0xb4, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
]);

macro_rules! must {
    ($expr:expr) => {
        match $expr {
            Ok(e) => e,
            Err(_) => loop {
                cortex_m::asm::wfi()
            },
        }
    };
}

fn must_succeed<V>(s: &hci::Status<V>) {
    match s {
        &hci::Status::Success => (),
        _ => loop {
            cortex_m::asm::wfi()
        },
    }
}

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
                // _p => writeln!(out, "other command complete").unwrap(),
                p => writeln!(out, "{:?}", p).unwrap(),
            }
        }
        e => writeln!(out, "{:?}", e).unwrap(),
        // _ => writeln!(out, "other event").unwrap(),
    }
}

fn print_error<Out: Write, E: Debug>(out: &mut Out, error: E) {
    writeln!(out, "Error: {:?}", error).unwrap();
    // writeln!(out, "error").unwrap();
    loop {}
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
            state: State::GettingVersionInfo,
            data: ProgramState {
                bnrg: bnrg,
                tim6: tim6,
                spi: spi,

                fw_version: None,

                gap_service_handle: None,
                dev_name_handle: None,
                appearance_handle: None,
                acc_service_handle: None,

                environmental_sensor_service_handle: None,
                temperature_characteristic_handle: None,
                humidity_characteristic_handle: None,
                pressure_characteristic_handle: None,

                time_service_handle: None,
                led_service_handle: None,
            },
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

    fw_version: Option<bluenrg::Version>,

    gap_service_handle: Option<bluenrg::gatt::ServiceHandle>,
    dev_name_handle: Option<bluenrg::gatt::CharacteristicHandle>,
    appearance_handle: Option<bluenrg::gatt::CharacteristicHandle>,

    acc_service_handle: Option<bluenrg::gatt::ServiceHandle>,

    environmental_sensor_service_handle: Option<bluenrg::gatt::ServiceHandle>,
    temperature_characteristic_handle: Option<bluenrg::gatt::CharacteristicHandle>,
    humidity_characteristic_handle: Option<bluenrg::gatt::CharacteristicHandle>,
    pressure_characteristic_handle: Option<bluenrg::gatt::CharacteristicHandle>,

    time_service_handle: Option<bluenrg::gatt::ServiceHandle>,
    led_service_handle: Option<bluenrg::gatt::ServiceHandle>,
}

#[derive(Copy, Clone)]
enum State {
    GettingVersionInfo,
    Resetting,
    SettingAddress,
    InitGatt,
    InitGap,
    SetDeviceName,
    SetAuthenticationRequirement,
    AddAccService,
    AddAccFreeFallCharacteristic,
    AddAccCharacteristic,
    AddEnvironmentalSensorService,
    AddTemperatureCharacteristic,
    AddTemperatureCharacteristicDescriptor,
    AddPressureCharacteristic,
    AddPressureCharacteristicDescriptor,
    AddHumidityCharacteristic,
    AddHumidityCharacteristicDescriptor,
    AddTimeService,
    AddTimeCharacteristic,
    AddMinuteCharacteristic,
    AddLedService,
    AddLedCharacteristic,
    SetTxPowerLevel,
    SetEmptyScanResponse,
    SetDiscoverable,
    Complete,
}

impl State {
    fn act<'a>(&self, ps: &mut ProgramState<'a>) {
        match self {
            &State::GettingVersionInfo => {
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(c.read_local_version_information()).unwrap()
                });
            }
            &State::Resetting => {
                ps.bnrg.reset(&mut ps.tim6, 200.hz());
            }
            &State::SettingAddress => {
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    let config = bluenrg::hal::ConfigData::public_address(hci::BdAddr([
                        0x12, 0x34, 0x00, 0xE1, 0x80, 0x02,
                    ])).build();
                    block!(c.write_config_data(&config)).unwrap()
                });
            }
            &State::InitGatt => ps.bnrg.with_spi(&mut ps.spi, |c| {
                block!(GattCommands::init(c as &mut GattCommands<Error = _>)).unwrap();
            }),
            &State::InitGap => ps.bnrg.with_spi(&mut ps.spi, |c| {
                block!(GapCommands::init(
                    c as &mut GapCommands<Error = _>,
                    bluenrg::gap::Role::PERIPHERAL,
                    false,
                    7,
                )).unwrap();
            }),
            &State::SetDeviceName => {
                let service = ps.gap_service_handle.unwrap();
                let characteristic = ps.dev_name_handle.unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(c.update_characteristic_value(
                        &bluenrg::gatt::UpdateCharacteristicValueParameters {
                            service_handle: service,
                            characteristic_handle: characteristic,
                            offset: 0,
                            value: b"BlueNRG",
                        }
                    ));
                })
            }
            &State::SetAuthenticationRequirement => ps.bnrg.with_spi(&mut ps.spi, |c| {
                block!(c.set_authentication_requirement(
                    &bluenrg::gap::AuthenticationRequirements {
                        mitm_protection_required: true,
                        out_of_band_auth: bluenrg::gap::OutOfBandAuthentication::Disabled,
                        encryption_key_size_range: (7, 16),
                        fixed_pin: bluenrg::gap::Pin::Fixed(123456),
                        bonding_required: true,
                    }
                ));
            }),
            &State::AddAccService => ps.bnrg.with_spi(&mut ps.spi, |c| {
                block!(c.add_service(&bluenrg::gatt::AddServiceParameters {
                    uuid: ACC_SERVICE_UUID,
                    service_type: bluenrg::gatt::ServiceType::Primary,
                    max_attribute_records: 7,
                }));
            }),
            &State::AddAccFreeFallCharacteristic => {
                let acc_service_handle = ps.acc_service_handle.unwrap();
                let fw_version = ps.fw_version.clone().unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(
                        c.add_characteristic(&bluenrg::gatt::AddCharacteristicParameters {
                            service_handle: acc_service_handle,
                            characteristic_uuid: ACC_FREE_FALL_UUID,
                            characteristic_value_len: 1,
                            characteristic_properties:
                                bluenrg::gatt::CharacteristicProperty::NOTIFY,
                            security_permissions: bluenrg::gatt::CharacteristicPermission::empty(),
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::empty(),
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: false,
                            fw_version_before_v72: fw_version.major < 7
                                || (fw_version.major == 7 && fw_version.minor < 2)
                        })
                    );
                });
            }
            &State::AddAccCharacteristic => {
                let acc_service_handle = ps.acc_service_handle.unwrap();
                let fw_version = ps.fw_version.clone().unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(
                        c.add_characteristic(&bluenrg::gatt::AddCharacteristicParameters {
                            service_handle: acc_service_handle,
                            characteristic_uuid: ACC_UUID,
                            characteristic_value_len: 6,
                            characteristic_properties: bluenrg::gatt::CharacteristicProperty::NOTIFY
                                | bluenrg::gatt::CharacteristicProperty::READ,
                            security_permissions: bluenrg::gatt::CharacteristicPermission::empty(),
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::CONFIRM_READ,
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: false,
                            fw_version_before_v72: fw_version.major < 7
                                || (fw_version.major == 7 && fw_version.minor < 2)
                        })
                    );
                });
            }
            &State::AddEnvironmentalSensorService => ps.bnrg.with_spi(&mut ps.spi, |c| {
                block!(c.add_service(&bluenrg::gatt::AddServiceParameters {
                    uuid: ENVIRONMENTAL_SENSOR_SERVICE_UUID,
                    service_type: bluenrg::gatt::ServiceType::Primary,
                    max_attribute_records: 10,
                }));
            }),
            &State::AddTemperatureCharacteristic => {
                let env_service_handle = ps.environmental_sensor_service_handle.unwrap();
                let fw_version = ps.fw_version.clone().unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(
                        c.add_characteristic(&bluenrg::gatt::AddCharacteristicParameters {
                            service_handle: env_service_handle,
                            characteristic_uuid: TEMPERATURE_CHARACTERISTIC_UUID,
                            characteristic_value_len: 2,
                            characteristic_properties: bluenrg::gatt::CharacteristicProperty::READ,
                            security_permissions: bluenrg::gatt::CharacteristicPermission::empty(),
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::CONFIRM_READ,
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: false,
                            fw_version_before_v72: fw_version.major < 7
                                || (fw_version.major == 7 && fw_version.minor < 2)
                        })
                    );
                });
            }
            &State::AddTemperatureCharacteristicDescriptor => {
                let env_service_handle = ps.environmental_sensor_service_handle.unwrap();
                let temperature_characteristic_handle =
                    ps.temperature_characteristic_handle.unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(c.add_characteristic_descriptor(
                        &bluenrg::gatt::AddDescriptorParameters {
                            service_handle: env_service_handle,
                            characteristic_handle: temperature_characteristic_handle,
                            descriptor_uuid: TEMPERATURE_DESCRIPTOR_UUID,
                            descriptor_value_max_len: 7,
                            descriptor_value: &[0x0E, 0xFF, 0x2F, 0x27, 0, 0, 0],
                            security_permissions: bluenrg::gatt::DescriptorPermission::empty(),
                            access_permissions: bluenrg::gatt::AccessPermission::READ,
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::empty(),
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: false,
                        }
                    ));
                });
            }
            &State::AddPressureCharacteristic => {
                let env_service_handle = ps.environmental_sensor_service_handle.unwrap();
                let fw_version = ps.fw_version.clone().unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(
                        c.add_characteristic(&bluenrg::gatt::AddCharacteristicParameters {
                            service_handle: env_service_handle,
                            characteristic_uuid: PRESSURE_CHARACTERISTIC_UUID,
                            characteristic_value_len: 3,
                            characteristic_properties: bluenrg::gatt::CharacteristicProperty::READ,
                            security_permissions: bluenrg::gatt::CharacteristicPermission::empty(),
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::CONFIRM_READ,
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: false,
                            fw_version_before_v72: fw_version.major < 7
                                || (fw_version.major == 7 && fw_version.minor < 2)
                        })
                    );
                });
            }
            &State::AddPressureCharacteristicDescriptor => {
                let env_service_handle = ps.environmental_sensor_service_handle.unwrap();
                let pressure_characteristic_handle = ps.pressure_characteristic_handle.unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(c.add_characteristic_descriptor(
                        &bluenrg::gatt::AddDescriptorParameters {
                            service_handle: env_service_handle,
                            characteristic_handle: pressure_characteristic_handle,
                            descriptor_uuid: PRESSURE_DESCRIPTOR_UUID,
                            descriptor_value_max_len: 7,
                            descriptor_value: &[0x0F, 0xFB, 0x80, 0x27, 0, 0, 0],
                            security_permissions: bluenrg::gatt::DescriptorPermission::empty(),
                            access_permissions: bluenrg::gatt::AccessPermission::READ,
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::empty(),
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: false,
                        }
                    ));
                });
            }
            &State::AddHumidityCharacteristic => {
                let environmental_sensor_service_handle =
                    ps.environmental_sensor_service_handle.unwrap();
                let fw_version = ps.fw_version.clone().unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(
                        c.add_characteristic(&bluenrg::gatt::AddCharacteristicParameters {
                            service_handle: environmental_sensor_service_handle,
                            characteristic_uuid: HUMIDITY_CHARACTERISTIC_UUID,
                            characteristic_value_len: 2,
                            characteristic_properties: bluenrg::gatt::CharacteristicProperty::READ,
                            security_permissions: bluenrg::gatt::CharacteristicPermission::empty(),
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::CONFIRM_READ,
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: false,
                            fw_version_before_v72: fw_version.major < 7
                                || (fw_version.major == 7 && fw_version.minor < 2)
                        })
                    );
                });
            }
            &State::AddHumidityCharacteristicDescriptor => {
                let env_service_handle = ps.environmental_sensor_service_handle.unwrap();
                let humidity_characteristic_handle = ps.humidity_characteristic_handle.unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(c.add_characteristic_descriptor(
                        &bluenrg::gatt::AddDescriptorParameters {
                            service_handle: env_service_handle,
                            characteristic_handle: humidity_characteristic_handle,
                            descriptor_uuid: HUMIDITY_DESCRIPTOR_UUID,
                            descriptor_value_max_len: 7,
                            descriptor_value: &[0x06, 0xFF, 0x00, 0x27, 0, 0, 0],
                            security_permissions: bluenrg::gatt::DescriptorPermission::empty(),
                            access_permissions: bluenrg::gatt::AccessPermission::READ,
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::empty(),
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: false,
                        }
                    ));
                });
            }
            &State::AddTimeService => ps.bnrg.with_spi(&mut ps.spi, |c| {
                block!(c.add_service(&bluenrg::gatt::AddServiceParameters {
                    uuid: TIME_SERVICE_UUID,
                    service_type: bluenrg::gatt::ServiceType::Primary,
                    max_attribute_records: 7,
                }));
            }),
            &State::AddTimeCharacteristic => {
                let time_service_handle = ps.time_service_handle.unwrap();
                let fw_version = ps.fw_version.clone().unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(
                        c.add_characteristic(&bluenrg::gatt::AddCharacteristicParameters {
                            service_handle: time_service_handle,
                            characteristic_uuid: TIME_CHARACTERISTIC_UUID,
                            characteristic_value_len: 4,
                            characteristic_properties: bluenrg::gatt::CharacteristicProperty::READ,
                            security_permissions: bluenrg::gatt::CharacteristicPermission::empty(),
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::empty(),
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: false,
                            fw_version_before_v72: fw_version.major < 7
                                || (fw_version.major == 7 && fw_version.minor < 2)
                        })
                    );
                });
            }
            &State::AddMinuteCharacteristic => {
                let time_service_handle = ps.time_service_handle.unwrap();
                let fw_version = ps.fw_version.clone().unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(
                        c.add_characteristic(&bluenrg::gatt::AddCharacteristicParameters {
                            service_handle: time_service_handle,
                            characteristic_uuid: MINUTE_CHARACTERISTIC_UUID,
                            characteristic_value_len: 4,
                            characteristic_properties: bluenrg::gatt::CharacteristicProperty::READ
                                | bluenrg::gatt::CharacteristicProperty::NOTIFY,
                            security_permissions: bluenrg::gatt::CharacteristicPermission::empty(),
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::CONFIRM_READ,
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: true,
                            fw_version_before_v72: fw_version.major < 7
                                || (fw_version.major == 7 && fw_version.minor < 2)
                        })
                    );
                });
            }
            &State::AddLedService => ps.bnrg.with_spi(&mut ps.spi, |c| {
                block!(c.add_service(&bluenrg::gatt::AddServiceParameters {
                    uuid: LED_SERVICE_UUID,
                    service_type: bluenrg::gatt::ServiceType::Primary,
                    max_attribute_records: 7,
                }));
            }),
            &State::AddLedCharacteristic => {
                let led_service_handle = ps.led_service_handle.unwrap();
                let fw_version = ps.fw_version.clone().unwrap();
                ps.bnrg.with_spi(&mut ps.spi, |c| {
                    block!(
                        c.add_characteristic(&bluenrg::gatt::AddCharacteristicParameters {
                            service_handle: led_service_handle,
                            characteristic_uuid: LED_CHARACTERISTIC_UUID,
                            characteristic_value_len: 4,
                            characteristic_properties: bluenrg::gatt::CharacteristicProperty::WRITE
                                | bluenrg::gatt::CharacteristicProperty::WRITE_WITHOUT_RESPONSE,
                            security_permissions: bluenrg::gatt::CharacteristicPermission::empty(),
                            gatt_event_mask: bluenrg::gatt::CharacteristicEvent::ATTRIBUTE_WRITE,
                            encryption_key_size: must!(
                                bluenrg::gatt::EncryptionKeySize::with_value(16)
                            ),
                            is_variable: true,
                            fw_version_before_v72: fw_version.major < 7
                                || (fw_version.major == 7 && fw_version.minor < 2)
                        })
                    );
                });
            }
            &State::SetTxPowerLevel => ps.bnrg.with_spi(&mut ps.spi, |c| {
                block!(c.set_tx_power_level(bluenrg::hal::PowerLevel::DbmNeg2_1));
            }),
            &State::SetEmptyScanResponse => ps.bnrg.with_spi(&mut ps.spi, |c| {
                block!(c.le_set_scan_response_data(&[]));
            }),
            &State::SetDiscoverable => ps.bnrg.with_spi(&mut ps.spi, |c| {
                block!(c.set_discoverable(&bluenrg::gap::DiscoverableParameters {
                    advertising_type: bluenrg::gap::AdvertisingType::ConnectableUndirected,
                    advertising_interval: None,
                    address_type: bluenrg::gap::OwnAddressType::Public,
                    filter_policy: bluenrg::gap::AdvertisingFilterPolicy::AllowConnectionAndScan,
                    local_name: Some(bluenrg::gap::LocalName::Complete(b"BlueNRG")),
                    advertising_data: &[],
                    conn_interval: (None, None),
                }));
            }),
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
        ps: &mut ProgramState<'a>,
        event: hci::event::Event<bluenrg::event::BlueNRGEvent>,
    ) -> Self {
        match self {
            &State::GettingVersionInfo => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::ReadLocalVersionInformation(p) =
                        cmd.return_params
                    {
                        must_succeed(&p.status);
                        ps.fw_version = Some(p.bluenrg_version());
                        return State::Resetting;
                    }
                }
            }
            &State::Resetting => {
                if let hci::Event::Vendor(bluenrg::event::BlueNRGEvent::HalInitialized(_)) = event {
                    return State::SettingAddress;
                }
            }
            &State::SettingAddress => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::HalWriteConfigData(s),
                    ) = cmd.return_params
                    {
                        must_succeed(&s);
                        return State::InitGatt;
                    }
                }
            }
            &State::InitGatt => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattInit(s),
                    ) = cmd.return_params
                    {
                        must_succeed(&s);
                        return State::InitGap;
                    }
                }
            }
            &State::InitGap => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GapInit(params),
                    ) = cmd.return_params
                    {
                        must_succeed(&params.status);
                        ps.gap_service_handle = Some(params.service_handle);
                        ps.dev_name_handle = Some(params.dev_name_handle);
                        ps.appearance_handle = Some(params.appearance_handle);
                        return State::SetDeviceName;
                    }
                }
            }
            &State::SetDeviceName => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattUpdateCharacteristicValue(s),
                    ) = cmd.return_params
                    {
                        must_succeed(&s);
                        return State::SetAuthenticationRequirement;
                    }
                }
            }
            &State::SetAuthenticationRequirement => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GapSetAuthenticationRequirement(
                            s,
                        ),
                    ) = cmd.return_params
                    {
                        must_succeed(&s);
                        return State::AddAccService;
                    }
                }
            }
            &State::AddAccService => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddService(params),
                    ) = cmd.return_params
                    {
                        must_succeed(&params.status);
                        ps.acc_service_handle = Some(params.service_handle);
                        return State::AddAccFreeFallCharacteristic;
                    }
                }
            }
            &State::AddAccFreeFallCharacteristic => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristic(p),
                    ) = cmd.return_params
                    {
                        must_succeed(&p.status);
                        return State::AddAccCharacteristic;
                    }
                }
            }
            &State::AddAccCharacteristic => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristic(p),
                    ) = cmd.return_params
                    {
                        must_succeed(&p.status);
                        return State::AddEnvironmentalSensorService;
                    }
                }
            }
            &State::AddEnvironmentalSensorService => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddService(params),
                    ) = cmd.return_params
                    {
                        must_succeed(&params.status);
                        ps.environmental_sensor_service_handle = Some(params.service_handle);
                        return State::AddTemperatureCharacteristic;
                    }
                }
            }
            &State::AddTemperatureCharacteristic => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristic(params),
                    ) = cmd.return_params
                    {
                        must_succeed(&params.status);
                        ps.temperature_characteristic_handle = Some(params.characteristic_handle);
                        return State::AddTemperatureCharacteristicDescriptor;
                    }
                }
            }
            &State::AddTemperatureCharacteristicDescriptor => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristicDescriptor(
                            p,
                        ),
                    ) = cmd.return_params
                    {
                        must_succeed(&p.status);
                        return State::AddPressureCharacteristic;
                    }
                }
            }
            &State::AddPressureCharacteristic => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristic(params),
                    ) = cmd.return_params
                    {
                        must_succeed(&params.status);
                        ps.pressure_characteristic_handle = Some(params.characteristic_handle);
                        return State::AddPressureCharacteristicDescriptor;
                    }
                }
            }
            &State::AddPressureCharacteristicDescriptor => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristicDescriptor(
                            p,
                        ),
                    ) = cmd.return_params
                    {
                        must_succeed(&p.status);
                        return State::AddHumidityCharacteristic;
                    }
                }
            }
            &State::AddHumidityCharacteristic => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristic(params),
                    ) = cmd.return_params
                    {
                        must_succeed(&params.status);
                        ps.humidity_characteristic_handle = Some(params.characteristic_handle);
                        return State::AddHumidityCharacteristicDescriptor;
                    }
                }
            }
            &State::AddHumidityCharacteristicDescriptor => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristicDescriptor(
                            p,
                        ),
                    ) = cmd.return_params
                    {
                        must_succeed(&p.status);
                        return State::AddTimeService;
                    }
                }
            }
            &State::AddTimeService => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddService(params),
                    ) = cmd.return_params
                    {
                        must_succeed(&params.status);
                        ps.time_service_handle = Some(params.service_handle);
                        return State::AddTimeCharacteristic;
                    }
                }
            }
            &State::AddTimeCharacteristic => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristic(p),
                    ) = cmd.return_params
                    {
                        must_succeed(&p.status);
                        return State::AddMinuteCharacteristic;
                    }
                }
            }
            &State::AddMinuteCharacteristic => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristic(p),
                    ) = cmd.return_params
                    {
                        must_succeed(&p.status);
                        return State::AddLedService;
                    }
                }
            }
            &State::AddLedService => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddService(params),
                    ) = cmd.return_params
                    {
                        must_succeed(&params.status);
                        ps.led_service_handle = Some(params.service_handle);
                        return State::AddLedCharacteristic;
                    }
                }
            }
            &State::AddLedCharacteristic => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GattAddCharacteristic(p),
                    ) = cmd.return_params
                    {
                        must_succeed(&p.status);
                        return State::SetTxPowerLevel;
                    }
                }
            }
            &State::SetTxPowerLevel => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::HalSetTxPowerLevel(s),
                    ) = cmd.return_params
                    {
                        must_succeed(&s);
                        return State::SetEmptyScanResponse;
                    }
                }
            }
            &State::SetEmptyScanResponse => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::LeSetScanResponseData(s) =
                        cmd.return_params
                    {
                        must_succeed(&s);
                        return State::SetDiscoverable;
                    }
                }
            }
            &State::SetDiscoverable => {
                if let hci::Event::CommandComplete(cmd) = event {
                    if let hci::event::command::ReturnParameters::Vendor(
                        bluenrg::event::command::ReturnParameters::GapSetDiscoverable(s),
                    ) = cmd.return_params
                    {
                        must_succeed(&s);
                        return State::Complete;
                    }
                }
            }
            &State::Complete => (),
        }

        *self
    }
}
