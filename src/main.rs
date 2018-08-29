#![no_std]
#![feature(lang_items)]
#![feature(start)]

extern crate bluenrg;
extern crate bluetooth_hci as hci;
extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
extern crate embedded_hal;
extern crate nb;
extern crate panic_semihosting;
extern crate spbtle_rf_stm32f303re_test;
extern crate stm32f30x;
extern crate stm32f30x_hal as hal;

use hal::flash::FlashExt;
use hal::gpio::GpioExt;
use hal::rcc::RccExt;
use hal::time::U32Ext;
use spbtle_rf_stm32f303re_test as main;

#[start]
#[inline(never)]
fn main(_argc: isize, _argv: *const *const u8) -> isize {
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
        let spi = hal::spi::Spi::spi1(
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

        main::EventLoop::new(&mut bnrg, tim6, spi).run();
    });

    1 // should not be here, so let's return failure.
}

#[lang = "eh_personality"]
extern "C" fn eh_personality() {}
