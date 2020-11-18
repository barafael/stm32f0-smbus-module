//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

use crate::pac::I2C1;
use stm32f0xx_hal::{gpio::gpiob::PB8, rcc};
use stm32f0xx_hal::gpio::gpiob::PB9;
use stm32f0xx_hal::gpio::Alternate;
use stm32f0xx_hal::gpio::AF1;

use stm32f0xx_hal::{
    gpio::gpioa::PA5,
    gpio::gpioc::PC13,
    gpio::{Input, Output, PushPull, Floating},
    pac,
    prelude::*,
    i2c::I2c,
};

use cortex_m::interrupt::free as disable_interrupts;

#[app(device = stm32f0xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        exti: pac::EXTI,
        user_button: PC13<Input<Floating>>,
        led: PA5<Output<PushPull>>,
        i2c: I2C1,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        // RTT handler
        rtt_init_print!();

        // Alias peripherals
        let mut dp: pac::Peripherals = ctx.device;

        rprintln!("Initializing peripherals");
        let mut rcc = dp
            .RCC
            .configure()
            .usbsrc(stm32f0xx_hal::rcc::USBClockSource::HSI48)
            .hsi48()
            .enable_crs(dp.CRS)
            .sysclk(48.mhz())
            .pclk(24.mhz())
            .freeze(&mut dp.FLASH);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let led = disable_interrupts(|cs| gpioa.pa5.into_push_pull_output(cs));

        let gpioc = dp.GPIOC.split(&mut rcc);
        let user_button = disable_interrupts(|cs| gpioc.pc13.into_floating_input(cs));

        dp.SYSCFG.exticr4.write(|w| w.exti13().pc13());

        // Set interrupt mask for all the above
        dp.EXTI.imr.write(|w| w.mr13().set_bit());

        // Set interrupt rising trigger
        dp.EXTI.ftsr.write(|w| w.tr13().set_bit());
        dp.EXTI.rtsr.write(|w| w.tr13().set_bit());

        rprintln!("Instantiating dp.EXTI...");
        let exti = dp.EXTI;
        rprintln!("Defining late resources...");

        let gpiob = dp.GPIOB.split(&mut rcc);

        let (scl, sda): (PB8<Alternate<AF1>>, PB9<Alternate<AF1>>) = disable_interrupts(|cs| {
            (
            gpiob.pb8.into_alternate_af1(cs),
            gpiob.pb9.into_alternate_af1(cs)
            )
        });

        unsafe {
            dp.I2C1.icr.write(|w| w.bits(0xFF));

            dp.I2C1.cr2.modify(|_, w| w.autoend().set_bit());
            dp.I2C1.cr1.modify(|_, w| w.gcen().set_bit());
            dp.I2C1.oar2.modify(|_, w| w.oa2en().clear_bit());
            dp.I2C1.cr1.modify(|_, w| w.nostretch().clear_bit());
            dp.I2C1.cr1.modify(|_, w| w.pe().clear_bit());
            dp.I2C1.cr1.modify(|_, w| w.anfoff().set_bit().dnf().no_filter());
            dp.I2C1.cr1.modify(|_, w| w.pe().set_bit());
            dp.I2C1.oar1.modify(|_, w| w.oa1en().set_bit());
            dp.I2C1.oar1.write(|w| w.oa1().bits(0x4e));
            dp.I2C1.oar1.modify(|_, w| w.oa1en().set_bit());
            //dp.I2C1.cr1.modify(|_, w| w.smbhen().set_bit().smbden().set_bit())
        }

        //I2c<I2C1, PB8<Alternate<AF1>>, PB9<Alternate<AF1>>>,
        init::LateResources {
            exti,
            user_button,
            led,
            i2c: dp.I2C1
        }
    }

    #[idle(resources = [])]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
            // disable wfe for now (no probe-rs)
            //cortex_m::asm::wfe();
        }
    }

    #[task(binds = I2C1, resources = [exti, user_button, led])]
    fn i2c1_interrupt(ctx: i2c1_interrupt::Context) { }

    #[task(binds = EXTI4_15, resources = [exti, user_button, led])]
    fn exti_4_15_interrupt(ctx: exti_4_15_interrupt::Context) {
        match ctx.resources.exti.pr.read().bits() {
            0x2000 => {
                ctx.resources.exti.pr.write(|w| w.pif13().set_bit());
                if ctx.resources.user_button.is_high().unwrap_or_default() {
                    ctx.resources.led.set_high().unwrap();
                    rprintln!("PC13 triggered: high");
                } else {
                    ctx.resources.led.set_low().unwrap();
                    rprintln!("PC13 triggered: low");
                }
            }

            x => rprintln!("{}: Some other bits were pushed around on EXTI4_15 ;)", x),
        }
    }
};
